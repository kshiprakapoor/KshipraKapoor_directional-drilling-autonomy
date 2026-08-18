from __future__ import annotations

from dataclasses import dataclass
from math import cos, exp, radians, sin, sqrt
from typing import Dict, List, Tuple

import numpy as np


@dataclass(frozen=True)
class ObstacleBox:
    hd_min_ft: float = 600.0
    hd_max_ft: float = 1200.0
    tvd_min_ft: float = 5800.0
    tvd_max_ft: float = 6400.0

    def contains(self, hd_ft: float, tvd_ft: float) -> bool:
        return (
            self.hd_min_ft <= hd_ft <= self.hd_max_ft
            and self.tvd_min_ft <= tvd_ft <= self.tvd_max_ft
        )


class WellboreDynamics:
    """Reduced-order directional-drilling plant for an interview-scale simulation."""

    def __init__(
        self,
        start_tvd_ft: float = 5000.0,
        start_hd_ft: float = 0.0,
        initial_inclination_deg: float = 31.0,
        azimuth_deg: float = 90.0,
        obstacle: ObstacleBox | None = None,
    ) -> None:
        self.tvd_ft = float(start_tvd_ft)
        self.hd_ft = float(start_hd_ft)
        self.inclination_deg = float(initial_inclination_deg)
        self.azimuth_deg = float(azimuth_deg)
        self.measured_depth_ft = 0.0

        azimuth_rad = radians(self.azimuth_deg)
        self.easting_ft = self.hd_ft * sin(azimuth_rad)
        self.northing_ft = self.hd_ft * cos(azimuth_rad)

        self.vibration_g = 0.7
        self.rop_ft_hr = 0.0
        self.tool_health_pct = 100.0
        self.fatigue_index = 0.0
        self.last_dls_deg_per_100ft = 0.0
        self.collision_active = False

        self.obstacle = obstacle or ObstacleBox()
        self.history: List[Dict[str, float | bool]] = []
        self._record_state()

    @property
    def position(self) -> Tuple[float, float]:
        return self.hd_ft, self.tvd_ft

    def distance_to_obstacle_ft(self) -> float:
        """Shortest section-view distance from the bit to the obstacle boundary."""
        dx = max(
            self.obstacle.hd_min_ft - self.hd_ft,
            0.0,
            self.hd_ft - self.obstacle.hd_max_ft,
        )
        dz = max(
            self.obstacle.tvd_min_ft - self.tvd_ft,
            0.0,
            self.tvd_ft - self.obstacle.tvd_max_ft,
        )
        return sqrt(dx * dx + dz * dz)

    def _formation_hardness_index(self, tvd_ft: float) -> float:
        """Smooth deterministic lithology surrogate; higher values reduce ROP."""
        long_wave = 0.18 * sin(tvd_ft / 215.0)
        short_wave = 0.08 * sin(tvd_ft / 71.0 + 0.6)
        return float(np.clip(0.95 + long_wave + short_wave, 0.65, 1.35))

    def _compute_rop_ft_hr(self, wob_klbf: float) -> float:
        hardness = self._formation_hardness_index(self.tvd_ft)
        wob_effect = 2.35 * max(wob_klbf, 1.0) ** 0.78
        directional_penalty = 1.0 - 0.0025 * abs(self.inclination_deg - 25.0)
        health_penalty = 0.55 + 0.45 * self.tool_health_pct / 100.0
        return max(8.0, wob_effect * directional_penalty * health_penalty / hardness)

    def _compute_vibration_g(
        self,
        wob_klbf: float,
        dls_deg_per_100ft: float,
        collision: bool,
    ) -> float:
        # --- TORSIONAL RESPONSE MODEL: deterministic forcing from WOB, curvature and lithology ---
        hardness = self._formation_hardness_index(self.tvd_ft)
        base = (
            0.45
            + 0.025 * wob_klbf
            + 0.16 * dls_deg_per_100ft
            + 0.22 * abs(hardness - 1.0)
            + 0.12 * abs(sin(self.measured_depth_ft / 43.0))
        )

        if not collision:
            return float(np.clip(base, 0.4, 4.5))

        # --- OBSTACLE IMPACT ENVELOPE: stick-slip equivalent shock after hazard penetration ---
        hd_penetration = min(
            max(self.hd_ft - self.obstacle.hd_min_ft, 0.0),
            max(self.obstacle.hd_max_ft - self.hd_ft, 0.0),
        )
        tvd_penetration = min(
            max(self.tvd_ft - self.obstacle.tvd_min_ft, 0.0),
            max(self.obstacle.tvd_max_ft - self.tvd_ft, 0.0),
        )
        penetration_ft = max(0.0, min(hd_penetration, tvd_penetration))
        shock = 8.0 + 3.2 * (1.0 - exp(-penetration_ft / 75.0))
        return max(shock, base)

    def _accumulate_fatigue(self, vibration_g: float, collision: bool, md_step_ft: float) -> None:
        # --- EXPONENTIAL DAMAGE LAW: high-G stick-slip consumes structural life nonlinearly ---
        if collision:
            damage_increment = (
                md_step_ft / 50.0
                * 0.28
                * exp(max(vibration_g - 8.0, 0.0) / 1.15)
            )
        else:
            damage_increment = (
                md_step_ft / 50.0
                * 0.0012
                * exp(max(vibration_g - 2.0, 0.0) / 2.2)
            )

        self.fatigue_index += damage_increment
        self.tool_health_pct = float(100.0 * exp(-self.fatigue_index))

    def step(
        self,
        steering_pad_force_kn: float,
        wob_klbf: float,
        md_step_ft: float = 50.0,
        steering_gain_deg_per_100ft_per_kn: float = 0.12,
    ) -> Dict[str, float | bool]:
        """Advance the reduced-order BHA model by one measured-depth increment."""
        previous_inclination_deg = self.inclination_deg

        # --- RSS STEERING RESPONSE: pad force maps to build/drop rate in the vertical section ---
        requested_delta_inclination = (
            steering_gain_deg_per_100ft_per_kn
            * steering_pad_force_kn
            * md_step_ft
            / 100.0
        )
        self.inclination_deg = float(
            np.clip(self.inclination_deg + requested_delta_inclination, 0.0, 88.0)
        )

        delta_inclination_deg = self.inclination_deg - previous_inclination_deg
        self.last_dls_deg_per_100ft = abs(delta_inclination_deg) / md_step_ft * 100.0

        inclination_rad = radians(self.inclination_deg)
        azimuth_rad = radians(self.azimuth_deg)
        horizontal_increment_ft = md_step_ft * sin(inclination_rad)
        vertical_increment_ft = md_step_ft * cos(inclination_rad)

        self.easting_ft += horizontal_increment_ft * sin(azimuth_rad)
        self.northing_ft += horizontal_increment_ft * cos(azimuth_rad)
        self.hd_ft = sqrt(self.easting_ft**2 + self.northing_ft**2)
        self.tvd_ft += vertical_increment_ft
        self.measured_depth_ft += md_step_ft

        self.rop_ft_hr = self._compute_rop_ft_hr(wob_klbf)
        self.collision_active = self.obstacle.contains(self.hd_ft, self.tvd_ft)
        self.vibration_g = self._compute_vibration_g(
            wob_klbf,
            self.last_dls_deg_per_100ft,
            self.collision_active,
        )
        self._accumulate_fatigue(self.vibration_g, self.collision_active, md_step_ft)
        self._record_state()
        return self.snapshot()

    def snapshot(self) -> Dict[str, float | bool]:
        return {
            "md_ft": self.measured_depth_ft,
            "tvd_ft": self.tvd_ft,
            "hd_ft": self.hd_ft,
            "inclination_deg": self.inclination_deg,
            "azimuth_deg": self.azimuth_deg,
            "vibration_g": self.vibration_g,
            "rop_ft_hr": self.rop_ft_hr,
            "tool_health_pct": self.tool_health_pct,
            "fatigue_index": self.fatigue_index,
            "dls_deg_per_100ft": self.last_dls_deg_per_100ft,
            "distance_to_obstacle_ft": self.distance_to_obstacle_ft(),
            "collision_active": self.collision_active,
        }

    def _record_state(self) -> None:
        self.history.append(self.snapshot())
