from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin
from typing import Dict, List, Sequence, Tuple

import numpy as np

from physics_engine import ObstacleBox


@dataclass
class AgentDecision:
    target_path: List[Tuple[float, float]]
    override_active: bool
    events: List[str]


class AutonomousSuperintendent:
    """Deterministic supervisory agent for hazard look-ahead and target-path re-planning."""

    def __init__(
        self,
        obstacle: ObstacleBox | None = None,
        trigger_distance_ft: float = 400.0,
        detour_dls_deg_per_100ft: float = 3.5,
        clearance_ft: float = 90.0,
    ) -> None:
        self.obstacle = obstacle or ObstacleBox()
        self.trigger_distance_ft = float(trigger_distance_ft)
        self.detour_dls_deg_per_100ft = float(detour_dls_deg_per_100ft)
        self.clearance_ft = float(clearance_ft)
        self.override_active = False
        self.detour_path: List[Tuple[float, float]] = []
        self.audit_log: List[str] = [
            "[SYSTEM] Autonomous Superintendent online; deterministic telemetry watch enabled."
        ]
        self._hazard_clear_event_emitted = False

    def calculate_dynamic_detour(
        self,
        state: Dict[str, float | bool],
        final_target: Tuple[float, float],
        md_step_ft: float = 50.0,
    ) -> List[Tuple[float, float]]:
        """Generate a DLS-bounded left-side bypass, then rebuild angle toward the final target."""
        hd = float(state["hd_ft"])
        tvd = float(state["tvd_ft"])
        inclination = float(state["inclination_deg"])
        target_hd, target_tvd = map(float, final_target)

        safe_left_hd = self.obstacle.hd_min_ft - self.clearance_ft
        bypass_tvd = self.obstacle.tvd_max_ft + 140.0
        dls_step_deg = self.detour_dls_deg_per_100ft * md_step_ft / 100.0
        path: List[Tuple[float, float]] = [(hd, tvd)]

        # --- HAZARD BYPASS GEOMETRY: rate-limited steering toward a clearance waypoint below the obstacle ---
        while tvd < bypass_tvd and tvd < target_tvd:
            remaining_vertical = max(bypass_tvd - tvd, 1.0)
            remaining_horizontal = safe_left_hd - hd
            desired_inclination = np.degrees(
                np.arctan2(max(remaining_horizontal, 0.0), remaining_vertical)
            )

            inclination_error = desired_inclination - inclination
            inclination += float(np.clip(inclination_error, -dls_step_deg, dls_step_deg))
            inclination = float(np.clip(inclination, 0.0, 88.0))

            hd += md_step_ft * sin(radians(inclination))
            tvd += md_step_ft * cos(radians(inclination))
            path.append((hd, tvd))

            if len(path) > 80:
                break

        # --- TARGET RECAPTURE: continuously point toward the target while rate-limiting inclination change ---
        while tvd < target_tvd - 20.0:
            remaining_vertical = max(target_tvd - tvd, 1.0)
            remaining_horizontal = max(target_hd - hd, 0.0)
            desired_inclination = np.degrees(np.arctan2(remaining_horizontal, remaining_vertical))

            inclination_error = desired_inclination - inclination
            inclination += float(np.clip(inclination_error, -dls_step_deg, dls_step_deg))
            inclination = float(np.clip(inclination, 0.0, 88.0))

            hd += md_step_ft * sin(radians(inclination))
            tvd += md_step_ft * cos(radians(inclination))
            path.append((hd, tvd))

            if len(path) > 160:
                break

        path.append((target_hd, target_tvd))
        return path

    def evaluate(
        self,
        telemetry: Dict[str, float | bool],
        nominal_target_path: Sequence[Tuple[float, float]],
        final_target: Tuple[float, float],
        agentic_mode: bool,
    ) -> AgentDecision:
        """Evaluate telemetry and return the active path plus a public engineering audit trail."""
        new_events: List[str] = []

        if not agentic_mode:
            self.override_active = False
            return AgentDecision(list(nominal_target_path), False, new_events)

        distance_ft = float(telemetry["distance_to_obstacle_ft"])
        vibration_g = float(telemetry["vibration_g"])
        rop_ft_hr = float(telemetry["rop_ft_hr"])

        if distance_ft <= self.trigger_distance_ft and not self.override_active:
            # --- SUPERVISORY HAZARD GATE: explicit trigger, tool invocation and control handoff ---
            new_events.extend(
                [
                    (
                        f"[ASSESSMENT] Hazard proximity threshold crossed: "
                        f"{distance_ft:.0f} ft to exclusion zone; vibration={vibration_g:.2f} g; "
                        f"ROP={rop_ft_hr:.1f} ft/hr."
                    ),
                    (
                        "[TOOL] calculate_dynamic_detour() invoked with "
                        f"DLS ceiling={self.detour_dls_deg_per_100ft:.1f}°/100 ft."
                    ),
                ]
            )
            self.detour_path = self.calculate_dynamic_detour(telemetry, final_target)
            self.override_active = True

            maximum_hd_before_release = max(
                point[0]
                for point in self.detour_path
                if point[1] <= self.obstacle.tvd_max_ft
            )
            new_events.extend(
                [
                    (
                        "[RESULT] DLS-bounded bypass generated; "
                        f"maximum HD inside hazard-depth interval={maximum_hd_before_release:.0f} ft."
                    ),
                    "[CONTROL] Supervisory path setpoints replaced; local MPC retains steering authority.",
                ]
            )

        if (
            self.override_active
            and not self._hazard_clear_event_emitted
            and float(telemetry["tvd_ft"]) > self.obstacle.tvd_max_ft + 220.0
        ):
            new_events.append(
                "[RESULT] Hazard cleared; target-recapture segment active under the same DLS constraint."
            )
            self._hazard_clear_event_emitted = True

        self.audit_log.extend(new_events)
        active_path = self.detour_path if self.override_active else list(nominal_target_path)
        return AgentDecision(active_path, self.override_active, new_events)
