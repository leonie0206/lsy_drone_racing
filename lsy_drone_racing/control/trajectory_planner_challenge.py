"""Trajectory planner for Level 2 drone racing (randomized gates and obstacles).

Extends the basic TrajectoryPlanner with:
- Gate threading: pre/post approach waypoints aligned to gate normal.
- Obstacle repulsion: iteratively nudges spline away from obstacles.
- Acceleration capping: extends t_total until trajectory is physically feasible.
- Online replanning: patches waypoints when sensor reveals true gate/obstacle positions.

Public interface is identical to TrajectoryPlanner so it can be used as a drop-in replacement
inside any MPC controller (e.g. attitude_mpc.py).
"""

from __future__ import annotations

import numpy as np
from scipy.interpolate import CubicSpline

# ── Nominal track layout (from level2.toml) ───────────────────────────────────
_REPLAN_THRESHOLD = 0.05   # m — minimum shift to trigger a replan
_GATE_MARGIN = 0.160       # m — pre/post waypoint distance from gate centre
_OBSTACLE_MARGIN = 0.250   # m — minimum horizontal clearance from obstacle axis

_NOMINAL_GATE_POS = np.array(
    [[0.5, 0.25, 0.7], [1.05, 0.75, 1.2], [-1.0, -0.25, 0.65], [0.0, -0.75, 1.2]],
    dtype=np.float64,
)
_NOMINAL_GATE_YAW = np.array([-0.78, 2.35, 3.14, 0.0], dtype=np.float64)

_NOMINAL_OBSTACLE_POS = np.array(
    [[0.0, 0.75, 1.55], [1.0, 0.25, 1.55], [-1.5, -0.25, 1.55], [-0.5, -0.75, 1.55]],
    dtype=np.float64,
)
# ─────────────────────────────────────────────────────────────────────────────


class TrajectoryPlannerChallenge:
    """Trajectory planner for Level 2 with online replanning.

    Public interface matches TrajectoryPlanner:
        - ``waypoints_pos``  : (N, 3) sampled position array
        - ``max_ticks``      : int — length of waypoints_pos minus 1
        - ``get_references`` : returns (pos_ref, vel_ref, yaw_ref, pos_e, vel_e, yaw_e)

    Additional interface for replanning:
        - ``check_and_replan(obs, current_tick)`` : call every control step; returns new tick or None
        - ``reset()``                             : call from episode_callback
    """

    def __init__(self, freq: int, t_total: float = 7):
        """Build gate-threaded trajectory from nominal track layout.

        Args:
            freq: Environment step frequency (Hz).
            t_total: Initial trajectory duration (s). May be extended by acceleration capping.
        """
        self._freq = freq
        self._t_total = t_total

        self._waypoints_list: list = []
        self._gate_indices: dict[int, tuple[int, int]] = {}

        self._waypoints_list.append([-1.5, 0.75, 0.05])            # Start
        self._waypoints_list.append([-1.0, 0.55, 0.4])             # Intermediate
        self._add_gate_waypoints(gate_id=0)
        self._add_gate_waypoints(gate_id=1, intermediate_point=[1.3, -0.15, 0.88])
        self._add_gate_waypoints(gate_id=2, intermediate_point=[-0.5, -0.05, 0.45])
        self._waypoints_list.append([-1.2, -0.2, 1.18])            # Intermediate
        self._add_gate_waypoints(gate_id=3, intermediate_point=[-0.6, -0.2, 1.2])
        self._waypoints_list.append([0.5, -0.75, 1.2])             # End

        self._base_waypoints = np.array(self._waypoints_list, dtype=np.float64)
        self._waypoints = self._base_waypoints.copy()

        self._planned_gates_pos = _NOMINAL_GATE_POS.copy()
        self._planned_obstacles_pos = _NOMINAL_OBSTACLE_POS.copy()
        self._replanned_gates: set[int] = set()

        self._des_pos_spline: CubicSpline | None = None
        self._des_vel_spline: CubicSpline | None = None

        # Public arrays (set by _build_spline)
        self.waypoints_pos: np.ndarray = np.empty((0, 3))
        self.waypoints_vel: np.ndarray = np.empty((0, 3))
        self.waypoints_yaw: np.ndarray = np.empty((0,))
        self.max_ticks: int = 0

        self._build_spline()

    def _add_gate_waypoints(self, gate_id: int, intermediate_point: list[float] | None = None) -> None:
        """Append pre/post approach waypoints for a gate, aligned to its normal.

        Args:
            gate_id: Index into _NOMINAL_GATE_POS / _NOMINAL_GATE_YAW.
            intermediate_point: Optional routing waypoint inserted before the gate pair.
        """
        if intermediate_point is not None:
            self._waypoints_list.append(intermediate_point)

        pos = _NOMINAL_GATE_POS[gate_id]
        yaw = _NOMINAL_GATE_YAW[gate_id]
        normal = np.array([np.cos(yaw), np.sin(yaw), 0.0])

        # Flip normal so drone always approaches from the correct side
        prev_wp = np.array(self._waypoints_list[-1])
        if np.dot(pos - prev_wp, normal) < 0:
            normal = -normal

        pre_idx = len(self._waypoints_list)
        self._waypoints_list.append((pos - _GATE_MARGIN * normal).tolist())
        post_idx = len(self._waypoints_list)
        self._waypoints_list.append((pos + _GATE_MARGIN * normal).tolist())

        self._gate_indices[gate_id] = (pre_idx, post_idx)

    def _build_spline(self) -> None:
        """Build spline with obstacle repulsion and acceleration capping, then resample arrays.

        Steps:
        1. Prune waypoints closer than 15 cm to avoid spline oscillation.
        2. Iteratively push spline away from obstacles (up to 4 passes).
        3. Remove newly-clustered waypoints (30 cm threshold).
        4. Extend t_total until max spline acceleration <= 4 m/s².
        5. Resample waypoints_pos, waypoints_vel, waypoints_yaw at freq * t_total points.
        6. Update max_ticks.
        """
        # Step 1 — prune tight clusters (< 15 cm)
        wps: list[np.ndarray] = [self._waypoints[0].copy()]
        for i in range(1, len(self._waypoints)):
            if np.linalg.norm(self._waypoints[i] - wps[-1]) > 0.15 or i == len(self._waypoints) - 1:
                wps.append(self._waypoints[i].copy())

        # Step 2 — obstacle repulsion (up to 4 iterations)
        for _ in range(4):
            wps_arr = np.array(wps)
            distances = np.linalg.norm(np.diff(wps_arr, axis=0), axis=1)
            cum_dist = np.concatenate(([0.0], np.cumsum(distances)))
            total_dist = cum_dist[-1]
            if total_dist == 0:
                break

            t_wps = (cum_dist / total_dist) * self._t_total
            temp_spline = CubicSpline(t_wps, wps_arr)
            t_samples = np.linspace(0, self._t_total, 200)
            spline_pts = temp_spline(t_samples)
            collision_found = False

            for obs_pos in self._planned_obstacles_pos:
                dist_xy = np.linalg.norm(spline_pts[:, :2] - obs_pos[:2], axis=1)
                min_idx = int(np.argmin(dist_xy))
                if dist_xy[min_idx] < _OBSTACLE_MARGIN:
                    p_coll = spline_pts[min_idx]
                    t_coll = t_samples[min_idx]
                    push_vec = p_coll[:2] - obs_pos[:2]
                    if np.linalg.norm(push_vec) < 1e-3:
                        push_vec = np.array([1.0, 0.0])
                    push_vec /= np.linalg.norm(push_vec)
                    nudged_wp = p_coll.copy()
                    nudged_wp[:2] = obs_pos[:2] + push_vec * (_OBSTACLE_MARGIN + 0.05)
                    insert_idx = int(np.searchsorted(t_wps, t_coll))
                    wps.insert(insert_idx, nudged_wp)
                    collision_found = True
                    break
            if not collision_found:
                break

        # Step 3 — remove re-clustered waypoints (< 30 cm)
        final_wps: list[np.ndarray] = [wps[0]]
        for wp in wps[1:-1]:
            if np.linalg.norm(wp - final_wps[-1]) > 0.3:
                final_wps.append(wp)
        if np.linalg.norm(final_wps[-1] - wps[-1]) > 0.05:
            final_wps.append(wps[-1])

        active_wps = np.array(final_wps)
        distances = np.linalg.norm(np.diff(active_wps, axis=0), axis=1)
        cum_dist = np.concatenate(([0.0], np.cumsum(distances)))
        total_dist = cum_dist[-1]

        # Step 4 — acceleration cap: extend t_total until max_acc <= 4 m/s²
        for _ in range(10):
            t_wps = (cum_dist / total_dist) * self._t_total
            self._des_pos_spline = CubicSpline(t_wps, active_wps)
            self._des_vel_spline = self._des_pos_spline.derivative(nu=1)
            acc_spline = self._des_pos_spline.derivative(nu=2)
            t_samples = np.linspace(0, self._t_total, 200)
            max_acc = float(np.max(np.linalg.norm(acc_spline(t_samples), axis=1)))
            if max_acc > 4.0:
                self._t_total += 0.15
            else:
                break

        # Step 5 — resample into fixed-length arrays for tick-indexing
        n_steps = int(self._freq * self._t_total)
        t_lin = np.linspace(0, self._t_total, n_steps)
        self.waypoints_pos = self._des_pos_spline(t_lin)
        self.waypoints_vel = self._des_vel_spline(t_lin)
        self.waypoints_yaw = np.zeros(n_steps)

        # Step 6 — update ceiling tick
        self.max_ticks = len(self.waypoints_pos) - 1

    def check_and_replan(self, obs: dict, current_tick: int) -> int | None:
        """Check if new gate/obstacle positions require a trajectory rebuild.

        Should be called once per control step before get_references.

        Args:
            obs: Current environment observation dict (needs target_gate, gates_pos,
                 optionally gates_rpy, obstacles_pos, pos).
            current_tick: Current integer trajectory tick (used for re-sync window).

        Returns:
            Updated tick index if a replan occurred, or None if no replan was needed.
        """
        needs_rebuild = False

        # Gate check
        target_gate = int(obs.get("target_gate", -1))
        if target_gate >= 0 and target_gate not in self._replanned_gates:
            new_gate_pos = np.asarray(obs["gates_pos"][target_gate], dtype=np.float64)
            new_yaw = (
                float(obs["gates_rpy"][target_gate][2])
                if "gates_rpy" in obs
                else _NOMINAL_GATE_YAW[target_gate]
            )
            if np.linalg.norm(new_gate_pos - self._planned_gates_pos[target_gate]) > _REPLAN_THRESHOLD:
                normal = np.array([np.cos(new_yaw), np.sin(new_yaw), 0.0])
                pre_idx, post_idx = self._gate_indices[target_gate]
                prev_wp = self._waypoints[pre_idx - 1]
                if np.dot(new_gate_pos - prev_wp, normal) < 0:
                    normal = -normal
                self._waypoints[pre_idx] = new_gate_pos - _GATE_MARGIN * normal
                self._waypoints[post_idx] = new_gate_pos + _GATE_MARGIN * normal
                self._planned_gates_pos[target_gate] = new_gate_pos
                self._replanned_gates.add(target_gate)
                needs_rebuild = True

        # Obstacle check
        if "obstacles_pos" in obs:
            current_obs_pos = np.asarray(obs["obstacles_pos"], dtype=np.float64)
            for i in range(len(current_obs_pos)):
                if np.linalg.norm(current_obs_pos[i] - self._planned_obstacles_pos[i]) > _REPLAN_THRESHOLD:
                    self._planned_obstacles_pos[i] = current_obs_pos[i]
                    needs_rebuild = True

        if not needs_rebuild:
            return None

        # Rebuild spline
        dt = 1.0 / self._freq
        old_t_track = current_tick * dt
        ref_pos = np.asarray(obs["pos"], dtype=np.float64)

        self._build_spline()

        # Re-sync: find drone position on new spline within ±1 s of old time
        t_start = max(0.0, old_t_track - 1.0)
        t_end = min(self._t_total, old_t_track + 1.0)
        t_samples = np.linspace(t_start, t_end, 200)
        path_pts = self._des_pos_spline(t_samples)
        closest_idx = int(np.argmin(np.linalg.norm(path_pts - ref_pos, axis=1)))
        new_t_track = float(t_samples[closest_idx])
        new_tick = int(np.clip(new_t_track / dt, 0, self.max_ticks))
        return new_tick

    def get_references(
        self, current_tick: int, horizon: int
    ) -> tuple[
        np.ndarray,  # pos_ref  (horizon, 3)
        np.ndarray,  # vel_ref  (horizon, 3)
        np.ndarray,  # yaw_ref  (horizon,)
        np.ndarray,  # pos_e    (3,)
        np.ndarray,  # vel_e    (3,)
        float,       # yaw_e
    ]:
        """Return the reference slice for the MPC prediction horizon.

        Args:
            current_tick: Current integer time step.
            horizon: MPC horizon length N.

        Returns:
            Tuple of (pos_ref, vel_ref, yaw_ref, pos_e, vel_e, yaw_e).
        """
        i = min(current_tick, max(0, self.max_ticks - horizon))
        pos_ref = self.waypoints_pos[i : i + horizon]
        vel_ref = self.waypoints_vel[i : i + horizon]
        yaw_ref = self.waypoints_yaw[i : i + horizon]
        end_idx = min(i + horizon, self.max_ticks)
        pos_e = self.waypoints_pos[end_idx]
        vel_e = self.waypoints_vel[end_idx]
        yaw_e = float(self.waypoints_yaw[end_idx])
        return pos_ref, vel_ref, yaw_ref, pos_e, vel_e, yaw_e

    def reset(self) -> None:
        """Reset all replanning state for a new episode."""
        self._t_total = 6.00
        self._waypoints = self._base_waypoints.copy()
        self._replanned_gates = set()
        self._planned_gates_pos = _NOMINAL_GATE_POS.copy()
        self._planned_obstacles_pos = _NOMINAL_OBSTACLE_POS.copy()
        self._build_spline()

