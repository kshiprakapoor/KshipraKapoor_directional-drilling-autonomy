from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin
from typing import Sequence, Tuple

import numpy as np
from scipy.optimize import minimize


@dataclass(frozen=True)
class MPCSolution:
    steering_pad_force_kn: float
    predicted_dls_deg_per_100ft: float
    objective_value: float
    optimizer_success: bool


class TrajectoryMPC:
    """Finite-horizon steering controller with explicit dogleg-severity constraints."""

    def __init__(
        self,
        max_dls_deg_per_100ft: float = 4.0,
        steering_gain_deg_per_100ft_per_kn: float = 0.12,
        max_pad_force_kn: float = 30.0,
        prediction_horizon: int = 6,
        md_step_ft: float = 50.0,
    ) -> None:
        self.max_dls_deg_per_100ft = float(max_dls_deg_per_100ft)
        self.steering_gain = float(steering_gain_deg_per_100ft_per_kn)
        self.max_pad_force_kn = float(max_pad_force_kn)
        self.prediction_horizon = int(prediction_horizon)
        self.md_step_ft = float(md_step_ft)
        self._previous_force_kn = 0.0

    @staticmethod
    def _target_hd_at_tvd(target_path: np.ndarray, tvd_ft: float) -> float:
        ordered = target_path[np.argsort(target_path[:, 1])]
        return float(
            np.interp(
                tvd_ft,
                ordered[:, 1],
                ordered[:, 0],
                left=ordered[0, 0],
                right=ordered[-1, 0],
            )
        )

    def _simulate_horizon(
        self,
        hd_ft: float,
        tvd_ft: float,
        inclination_deg: float,
        force_sequence_kn: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        hd_prediction = []
        tvd_prediction = []
        inclination_prediction = []

        hd = float(hd_ft)
        tvd = float(tvd_ft)
        inclination = float(inclination_deg)

        for force_kn in force_sequence_kn:
            # --- REDUCED RSS KINEMATICS: steering effort changes inclination before spatial propagation ---
            delta_inclination_deg = (
                self.steering_gain
                * force_kn
                * self.md_step_ft
                / 100.0
            )
            inclination = float(np.clip(inclination + delta_inclination_deg, 0.0, 88.0))

            inclination_rad = radians(inclination)
            hd += self.md_step_ft * sin(inclination_rad)
            tvd += self.md_step_ft * cos(inclination_rad)

            hd_prediction.append(hd)
            tvd_prediction.append(tvd)
            inclination_prediction.append(inclination)

        return (
            np.asarray(hd_prediction),
            np.asarray(tvd_prediction),
            np.asarray(inclination_prediction),
        )

    def _objective(
        self,
        force_sequence_kn: np.ndarray,
        state: dict,
        target_path: np.ndarray,
    ) -> float:
        hd_pred, tvd_pred, _ = self._simulate_horizon(
            state["hd_ft"],
            state["tvd_ft"],
            state["inclination_deg"],
            force_sequence_kn,
        )

        target_hd = np.array(
            [self._target_hd_at_tvd(target_path, tvd) for tvd in tvd_pred]
        )
        cross_track_error_ft = hd_pred - target_hd

        # --- MPC COST FUNCTION: prioritize trajectory capture, then smooth/efficient steering effort ---
        tracking_cost = np.sum(cross_track_error_ft**2)
        effort_cost = 1.8 * np.sum(force_sequence_kn**2)
        force_delta = np.diff(np.r_[self._previous_force_kn, force_sequence_kn])
        smoothness_cost = 3.5 * np.sum(force_delta**2)
        terminal_cost = 2.5 * cross_track_error_ft[-1] ** 2
        return float(tracking_cost + effort_cost + smoothness_cost + terminal_cost)

    def solve(
        self,
        state: dict,
        target_path: Sequence[Tuple[float, float]] | np.ndarray,
    ) -> MPCSolution:
        target_array = np.asarray(target_path, dtype=float)
        if target_array.ndim != 2 or target_array.shape[1] != 2:
            raise ValueError("target_path must be an array-like sequence of (HD, TVD) points.")

        # --- ENFORCE GEOMETRIC DOGLEG CONSTRAINTS: convert DLS ceiling into pad-force bounds ---
        max_force_from_dls = self.max_dls_deg_per_100ft / max(self.steering_gain, 1e-9)
        admissible_force_kn = min(self.max_pad_force_kn, max_force_from_dls)
        bounds = [(-admissible_force_kn, admissible_force_kn)] * self.prediction_horizon

        initial_guess = np.full(self.prediction_horizon, self._previous_force_kn * 0.35)

        solution = minimize(
            self._objective,
            initial_guess,
            args=(state, target_array),
            method="SLSQP",
            bounds=bounds,
            options={"maxiter": 60, "ftol": 1e-5, "disp": False},
        )

        force_sequence = (
            solution.x
            if solution.success and np.all(np.isfinite(solution.x))
            else np.zeros(self.prediction_horizon)
        )
        steering_pad_force_kn = float(
            np.clip(force_sequence[0], -admissible_force_kn, admissible_force_kn)
        )

        predicted_delta_inclination_deg = (
            self.steering_gain
            * steering_pad_force_kn
            * self.md_step_ft
            / 100.0
        )
        predicted_dls = abs(predicted_delta_inclination_deg) / self.md_step_ft * 100.0

        self._previous_force_kn = steering_pad_force_kn
        return MPCSolution(
            steering_pad_force_kn=steering_pad_force_kn,
            predicted_dls_deg_per_100ft=float(predicted_dls),
            objective_value=float(solution.fun) if np.isfinite(solution.fun) else float("nan"),
            optimizer_success=bool(solution.success),
        )
