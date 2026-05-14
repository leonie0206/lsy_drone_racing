"""This module implements an example MPC using attitude control for a quadrotor.

It utilizes the collective thrust interface for drone control to compute control commands based on
current state observations and desired waypoints.

The waypoints are generated using cubic spline interpolation from a set of predefined waypoints.
Gate threading, obstacle repulsion, and online replanning are used to handle randomized tracks.
"""

from __future__ import annotations  # Python 3.10 type hints

from typing import TYPE_CHECKING

import numpy as np
import scipy
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from drone_models.core import load_params
from drone_models.so_rpy import symbolic_dynamics_euler
from drone_models.utils.rotation import ang_vel2rpy_rates
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R

from lsy_drone_racing.control import Controller

if TYPE_CHECKING:
    from numpy.typing import NDArray

# ── Replanning constants ──────────────────────────────────────────────────────
_REPLAN_THRESHOLD = 0.04
_GATE_MARGIN = 0.160
_OBSTACLE_MARGIN = 0.250

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


def create_acados_model(parameters: dict) -> AcadosModel:
    """Creates an acados model from a symbolic drone_model."""
    # For more info on the models, check out https://github.com/learnsyslab/drone-models
    X_dot, X, U, _ = symbolic_dynamics_euler(
        mass=parameters["mass"],
        gravity_vec=parameters["gravity_vec"],
        J=parameters["J"],
        J_inv=parameters["J_inv"],
        acc_coef=parameters["acc_coef"],
        cmd_f_coef=parameters["cmd_f_coef"],
        rpy_coef=parameters["rpy_coef"],
        rpy_rates_coef=parameters["rpy_rates_coef"],
        cmd_rpy_coef=parameters["cmd_rpy_coef"],
    )

    # Initialize the nonlinear model for NMPC formulation
    model = AcadosModel()
    model.name = "basic_example_mpc"
    model.f_expl_expr = X_dot
    model.f_impl_expr = None
    model.x = X
    model.u = U

    return model


def create_ocp_solver(
    Tf: float, N: int, parameters: dict, verbose: bool = False
) -> tuple[AcadosOcpSolver, AcadosOcp]:
    """Creates an acados Optimal Control Problem and Solver."""
    ocp = AcadosOcp()

    # Set model
    ocp.model = create_acados_model(parameters)

    # Get Dimensions
    nx = ocp.model.x.rows()
    nu = ocp.model.u.rows()
    ny = nx + nu
    ny_e = nx

    # Set dimensions
    ocp.solver_options.N_horizon = N

    ## Set Cost
    # For more Information regarding Cost Function Definition in Acados:
    # https://github.com/acados/acados/blob/main/docs/problem_formulation/problem_formulation_ocp_mex.pdf
    #

    # Cost Type
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    # Weights
    # State weights
    Q = np.diag(
        [
            50.0,  # pos
            50.0,  # pos
            400.0,  # pos
            1.0,  # rpy
            1.0,  # rpy
            1.0,  # rpy
            10.0,  # vel
            10.0,  # vel
            10.0,  # vel
            5.0,  # drpy
            5.0,  # drpy
            5.0,  # drpy
        ]
    )
    # Input weights (reference is upright orientation and hover thrust)
    R = np.diag(
        [
            1.0,  # rpy
            1.0,  # rpy
            1.0,  # rpy
            50.0,  # thrust
        ]
    )

    Q_e = Q.copy()
    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Q_e

    Vx = np.zeros((ny, nx))
    Vx[0:nx, 0:nx] = np.eye(nx)  # Select all states
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, :] = np.eye(nu)  # Select all actions
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[0:nx, 0:nx] = np.eye(nx)  # Select all states
    ocp.cost.Vx_e = Vx_e

    # Set initial references. We will overwrite these later to track the trajectory
    ocp.cost.yref, ocp.cost.yref_e = np.zeros((ny,)), np.zeros((ny_e,))

    # Set State Constraints (rpy < 30°)
    ocp.constraints.lbx = np.array([-0.5, -0.5, -0.5])
    ocp.constraints.ubx = np.array([0.5, 0.5, 0.5])
    ocp.constraints.idxbx = np.array([3, 4, 5])

    # Set Input Constraints (rpy < 30°)
    ocp.constraints.lbu = np.array([-0.5, -0.5, -0.5, parameters["thrust_min"] * 4])
    ocp.constraints.ubu = np.array([0.5, 0.5, 0.5, parameters["thrust_max"] * 4])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    # We have to set x0 even though we will overwrite it later on.
    ocp.constraints.x0 = np.zeros((nx))

    # Solver Options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_, PARTIAL_ ,_HPIPM, _QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP"  # SQP, SQP_RTI
    ocp.solver_options.tol = 1e-6

    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.qp_solver_warm_start = 1

    ocp.solver_options.qp_solver_iter_max = 20
    ocp.solver_options.nlp_solver_max_iter = 50

    # set prediction horizon
    ocp.solver_options.tf = Tf

    acados_ocp_solver = AcadosOcpSolver(
        ocp,
        json_file="c_generated_code/lsy_example_mpc.json",
        verbose=verbose,
        build=True,
        generate=True,
    )

    return acados_ocp_solver, ocp


class AttitudeMPC(Controller):
    """MPC using collective thrust and attitude interface with online replanning."""

    def __init__(self, obs: dict[str, NDArray[np.floating]], info: dict, config: dict):
        """Initialize the attitude MPC controller.

        Args:
            obs: The initial observation of the environment's state. See the environment's
                observation space for details.
            info: Additional environment information from the reset.
            config: The configuration of the environment.
        """
        super().__init__(obs, info, config)
        self._N = 25
        self._dt = 1 / config.env.freq
        self._T_HORIZON = self._N * self._dt
        self._freq = config.env.freq
        self._t_total = 6.0  # initial trajectory duration; may extend due to acc cap

        # ── Gate-threaded waypoint builder (from controller_push) ─────────────
        self._waypoints_list: list = []
        self._gate_indices: dict = {}

        self._waypoints_list.append([-1.5, 0.75, 0.05])       # Start
        self._waypoints_list.append([-1.0, 0.55, 0.4])        # Intermediate
        self._add_gate_waypoints(gate_id=0)
        self._add_gate_waypoints(gate_id=1, intermediate_point=[1.3, -0.15, 0.9])
        self._add_gate_waypoints(gate_id=2, intermediate_point=[-0.5, -0.05, 0.5])
        self._waypoints_list.append([-1.2, -0.2, 1.18])       # Intermediate
        self._add_gate_waypoints(gate_id=3, intermediate_point=[-0.6, -0.2, 1.2])
        self._waypoints_list.append([0.5, -0.75, 1.2])        # End

        self._base_waypoints = np.array(self._waypoints_list, dtype=np.float64)
        self._waypoints = self._base_waypoints.copy()

        # Track which gate/obstacle positions we last planned for
        self._planned_gates_pos = np.array(
            obs.get("gates_pos", _NOMINAL_GATE_POS), dtype=np.float64
        )
        self._planned_obstacles_pos = np.array(
            obs.get("obstacles_pos", _NOMINAL_OBSTACLE_POS), dtype=np.float64
        )
        self._replanned_gates: set[int] = set()

        # Continuous trajectory time (used only for replan re-sync)
        self._t_track = 0.0

        # Spline objects (set by _build_spline)
        self._des_pos_spline: CubicSpline | None = None
        self._des_vel_spline: CubicSpline | None = None

        # Build spline → samples self._waypoints_pos / _vel / _yaw
        self._build_spline()
        # ─────────────────────────────────────────────────────────────────────

        self.drone_params = load_params("so_rpy", config.sim.drone_model)
        self._acados_ocp_solver, self._ocp = create_ocp_solver(
            self._T_HORIZON, self._N, self.drone_params
        )
        self._nx = self._ocp.model.x.rows()
        self._nu = self._ocp.model.u.rows()
        self._ny = self._nx + self._nu
        self._ny_e = self._nx

        self._tick = 0
        self._config = config
        self._finished = False

    # ── Waypoint / spline helpers (ported from controller_push) ──────────────

    def _add_gate_waypoints(self, gate_id: int, intermediate_point: list[float] | None = None):
        """Add pre/post gate waypoints aligned with the gate normal."""
        if intermediate_point:
            self._waypoints_list.append(intermediate_point)

        pos = _NOMINAL_GATE_POS[gate_id]
        yaw = _NOMINAL_GATE_YAW[gate_id]
        normal = np.array([np.cos(yaw), np.sin(yaw), 0.0])

        prev_wp = np.array(self._waypoints_list[-1])
        if np.dot(pos - prev_wp, normal) < 0:
            normal = -normal

        pre_idx = len(self._waypoints_list)
        self._waypoints_list.append((pos - _GATE_MARGIN * normal).tolist())
        post_idx = len(self._waypoints_list)
        self._waypoints_list.append((pos + _GATE_MARGIN * normal).tolist())

        self._gate_indices[gate_id] = (pre_idx, post_idx)

    def _build_spline(self) -> None:
        """Build spline with obstacle repulsion and acceleration capping.

        After building, resamples self._waypoints_pos / _vel / _yaw at
        freq * t_total points so the MPC tick-indexing keeps working unchanged.
        """
        # Prune tight waypoint clusters
        wps = [self._waypoints[0].copy()]
        for i in range(1, len(self._waypoints)):
            if np.linalg.norm(self._waypoints[i] - wps[-1]) > 0.15 or i == len(self._waypoints) - 1:
                wps.append(self._waypoints[i].copy())

        # Iteratively nudge waypoints away from obstacles
        for _ in range(4):
            wps_arr = np.array(wps)
            distances = np.linalg.norm(np.diff(wps_arr, axis=0), axis=1)
            cum_distances = np.concatenate(([0], np.cumsum(distances)))
            total_distance = cum_distances[-1]
            if total_distance == 0:
                break

            t_wps = (cum_distances / total_distance) * self._t_total
            temp_spline = CubicSpline(t_wps, wps_arr)
            t_samples = np.linspace(0, self._t_total, 200)
            spline_pts = temp_spline(t_samples)
            collision_found = False

            for obs_pos in self._planned_obstacles_pos:
                dist_xy = np.linalg.norm(spline_pts[:, :2] - obs_pos[:2], axis=1)
                min_idx = np.argmin(dist_xy)
                if dist_xy[min_idx] < _OBSTACLE_MARGIN:
                    p_coll = spline_pts[min_idx]
                    t_coll = t_samples[min_idx]
                    push_vec = p_coll[:2] - obs_pos[:2]
                    if np.linalg.norm(push_vec) < 1e-3:
                        push_vec = np.array([1.0, 0.0])
                    push_vec /= np.linalg.norm(push_vec)
                    nudged_wp = p_coll.copy()
                    nudged_wp[:2] = obs_pos[:2] + push_vec * (_OBSTACLE_MARGIN + 0.05)
                    insert_idx = np.searchsorted(t_wps, t_coll)
                    wps.insert(insert_idx, nudged_wp)
                    collision_found = True
                    break
            if not collision_found:
                break

        # Remove clustered waypoints to prevent "W" shapes
        final_wps = [wps[0]]
        for wp in wps[1:-1]:
            if np.linalg.norm(wp - final_wps[-1]) > 0.3:
                final_wps.append(wp)
        if np.linalg.norm(final_wps[-1] - wps[-1]) > 0.05:
            final_wps.append(wps[-1])

        active_wps = np.array(final_wps)
        distances = np.linalg.norm(np.diff(active_wps, axis=0), axis=1)
        cum_distances = np.concatenate(([0], np.cumsum(distances)))
        total_distance = cum_distances[-1]

        # Enforce max acceleration by extending trajectory time if needed
        for _ in range(10):
            t_wps = (cum_distances / total_distance) * self._t_total
            self._des_pos_spline = CubicSpline(t_wps, active_wps)
            self._des_vel_spline = self._des_pos_spline.derivative(nu=1)
            acc_spline = self._des_pos_spline.derivative(nu=2)

            t_samples = np.linspace(0, self._t_total, 200)
            max_acc = np.max(np.linalg.norm(acc_spline(t_samples), axis=1))
            if max_acc > 4.0:
                self._t_total += 0.15
            else:
                break

        # Resample into arrays for the MPC tick-indexer
        n_steps = int(self._freq * self._t_total)
        t_lin = np.linspace(0, self._t_total, n_steps)
        self._waypoints_pos = self._des_pos_spline(t_lin)
        self._waypoints_vel = self._des_vel_spline(t_lin)
        self._waypoints_yaw = np.zeros(n_steps)

        # Update tick ceiling (MPC needs N steps ahead)
        self._tick_max = max(0, len(self._waypoints_pos) - 1 - self._N)

    def _check_and_replan(self, obs: dict[str, NDArray[np.floating]]) -> None:
        """Replan trajectory when gates or obstacles are observed at new positions."""
        needs_rebuild = False

        target_gate = int(obs["target_gate"])
        if target_gate >= 0 and target_gate not in self._replanned_gates:
            new_gate_pos = np.asarray(obs["gates_pos"][target_gate], dtype=np.float64)
            new_yaw = (
                obs["gates_rpy"][target_gate][2]
                if "gates_rpy" in obs
                else _NOMINAL_GATE_YAW[target_gate]
            )

            delta_pos = new_gate_pos - self._planned_gates_pos[target_gate]
            if np.linalg.norm(delta_pos) > _REPLAN_THRESHOLD:
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

        if "obstacles_pos" in obs:
            current_obs_pos = np.asarray(obs["obstacles_pos"], dtype=np.float64)
            for i in range(len(current_obs_pos)):
                if np.linalg.norm(current_obs_pos[i] - self._planned_obstacles_pos[i]) > _REPLAN_THRESHOLD:
                    self._planned_obstacles_pos[i] = current_obs_pos[i]
                    needs_rebuild = True

        if needs_rebuild:
            # Remember current continuous time before rebuild
            old_t_track = self._t_track

            self._build_spline()

            # Re-sync: find closest point on new spline near old position
            t_start = max(0.0, old_t_track - 1.0)
            t_end = min(self._t_total, old_t_track + 1.0)
            t_samples = np.linspace(t_start, t_end, 200)
            old_des_pos = self._des_pos_spline(old_t_track)  # pos on old spline (now rebuilt)
            # Use actual drone position for re-sync if available
            ref_pos = obs.get("pos", old_des_pos)
            path_pts = self._des_pos_spline(t_samples)
            closest_idx = np.argmin(np.linalg.norm(path_pts - ref_pos, axis=1))
            self._t_track = t_samples[closest_idx]

            # Convert back to tick
            self._tick = int(np.clip(self._t_track / self._dt, 0, self._tick_max))

    # ── MPC control loop (unchanged logic, new reference source) ─────────────

    def compute_control(
        self, obs: dict[str, NDArray[np.floating]], info: dict | None = None
    ) -> NDArray[np.floating]:
        """Compute the next desired collective thrust and roll/pitch/yaw of the drone.

        Args:
            obs: The current observation of the environment. See the environment's observation space
                for details.
            info: Optional additional information as a dictionary.

        Returns:
            The orientation as roll, pitch, yaw angles, and the collective thrust
            [r_des, p_des, y_des, t_des] as a numpy array.
        """
        # Update continuous time tracker (used for replan re-sync)
        self._t_track = self._tick * self._dt

        # Online replanning when gate/obstacle positions are updated
        self._check_and_replan(obs)

        i = min(self._tick, self._tick_max)
        if self._tick >= self._tick_max:
            self._finished = True

        # Setting initial state
        obs["rpy"] = R.from_quat(obs["quat"]).as_euler("xyz")
        obs["drpy"] = ang_vel2rpy_rates(obs["quat"], obs["ang_vel"])
        x0 = np.concatenate((obs["pos"], obs["rpy"], obs["vel"], obs["drpy"]))
        self._acados_ocp_solver.set(0, "lbx", x0)
        self._acados_ocp_solver.set(0, "ubx", x0)

        # Setting state reference from resampled spline arrays
        yref = np.zeros((self._N, self._ny))
        yref[:, 0:3] = self._waypoints_pos[i : i + self._N]
        yref[:, 5] = self._waypoints_yaw[i : i + self._N]
        yref[:, 6:9] = self._waypoints_vel[i : i + self._N]
        yref[:, 15] = self.drone_params["mass"] * -self.drone_params["gravity_vec"][-1]
        for j in range(self._N):
            self._acados_ocp_solver.set(j, "yref", yref[j])

        # Setting terminal reference
        yref_e = np.zeros((self._ny_e,))
        yref_e[0:3] = self._waypoints_pos[i + self._N]
        yref_e[5] = self._waypoints_yaw[i + self._N]
        yref_e[6:9] = self._waypoints_vel[i + self._N]
        self._acados_ocp_solver.set(self._N, "y_ref", yref_e)

        # Solving problem and getting first input
        self._acados_ocp_solver.solve()
        u0 = self._acados_ocp_solver.get(0, "u")

        return u0

    def step_callback(
        self,
        action: NDArray[np.floating],
        obs: dict[str, NDArray[np.floating]],
        reward: float,
        terminated: bool,
        truncated: bool,
        info: dict,
    ) -> bool:
        """Increment the tick counter."""
        self._tick += 1

        return self._finished

    def episode_callback(self):
        """Reset all trajectory state for a new episode."""
        self._tick = 0
        self._t_track = 0.0
        self._finished = False
        self._t_total = 6.0
        self._waypoints = self._base_waypoints.copy()
        self._replanned_gates = set()
        self._planned_gates_pos = _NOMINAL_GATE_POS.copy()
        self._planned_obstacles_pos = _NOMINAL_OBSTACLE_POS.copy()
        self._build_spline()
