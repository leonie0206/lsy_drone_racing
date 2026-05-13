"""Obstacle manager for drone racing."""

import casadi as ca
import numpy as np
from crazyflow.sim import Sim
from crazyflow.sim.visualize import draw_capsule, draw_points


class ObstacleManager:
    """Manager for drone obstacles."""
    def __init__(self, safety_margin: float = 0.15):
        """Initialize the obstacle manager.

        Args:
            safety_margin: Extra buffer distance (in meters) around obstacles.
        """
        self.safety_margin = safety_margin
        # Each entry: {"type": "sphere/cylinder", "p1": np.array, "p2": np.array, "r": float}
        self.obstacles = []

    def add_sphere(self, center: np.ndarray, radius: float):
        """Add a spherical obstacle."""
        # Force the array to be float64 even if integers are passed
        p = np.array(center, dtype=np.float64) 
        self.obstacles.append({
            "type": "sphere",
            "p1": p,
            "p2": p.copy(), # p1=p2 for a sphere capsule
            "r": radius
        })

    def add_cylinder(self, start: np.ndarray, end: np.ndarray, radius: float):
        """Add a cylinder (capsule) at any orientation."""
        # Force start and end to be float64
        self.obstacles.append({
            "type": "cylinder",
            "p1": np.array(start, dtype=np.float64),
            "p2": np.array(end, dtype=np.float64),
            "r": radius
        })

    def add_gate(
        self, 
        pos: list | np.ndarray, 
        rpy: list | np.ndarray, 
        inner_width: float = 0.4, 
        outer_width: float = 0.72
    ) -> None:
        """Models the solid banner of a gate as 4 cylinders.
        
        Args:
            pos: [x, y, z] center of the gate.
            rpy: [roll, pitch, yaw] in radians.
            inner_width: Width/height of the opening (default 0.4m).
            outer_width: Outer width/height of the frame (default 0.72m).
        """
        center = np.array(pos, dtype=np.float64)
        yaw = rpy[2]
        
        # Calculate banner midpoint (where the cylinder centerline goes)
        # Offset = (0.2 + 0.36) / 2 = 0.28m
        banner_offset = (inner_width / 4.0) + (outer_width / 4.0)
        # Thickness covers the banner: (0.36 - 0.2) / 2 = 0.08m
        thickness = (outer_width - inner_width) / 4.0

        # Define 4 corners at the banner's centerline
        local_corners = [
            np.array([0, -banner_offset,  banner_offset]), # Top-Left
            np.array([0,  banner_offset,  banner_offset]), # Top-Right
            np.array([0,  banner_offset, -banner_offset]), # Bottom-Right
            np.array([0, -banner_offset, -banner_offset])  # Bottom-Left
        ]

        # Rotation Matrix (Yaw)
        R = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0,            0,           1]
        ])

        world_corners = [(R @ p) + center for p in local_corners]

        # Add cylinders covering the 4 solid banner segments
        self.add_cylinder(world_corners[0], world_corners[1], thickness) # Top
        self.add_cylinder(world_corners[1], world_corners[2], thickness) # Right
        self.add_cylinder(world_corners[2], world_corners[3], thickness) # Bottom
        self.add_cylinder(world_corners[3], world_corners[0], thickness) # Left

        
    def initialize_nominal_track(self) -> None:
        """Hardcodes the nominal positions of gates and obstacles for Level 0.
        
        Args:
            manager: The ObstacleManager instance to populate.
        """
        # --- 1. GATES ---
        # Standard dimensions: outer_width = 0.72m
        # Heights provided are from ground to the center of the gate.
        gates_data = [
            {"pos": [0.5, 0.25, 0.7], "rpy": [0.0, 0.0, -0.78]},
            {"pos": [1.05, 0.75, 1.2], "rpy": [0.0, 0.0, 2.35]},
            {"pos": [-1.0, -0.25, 0.7], "rpy": [0.0, 0.0, 3.14]},
            {"pos": [0.0, -0.75, 1.2], "rpy": [0.0, 0.0, 0.0]},
        ]

        for gate in gates_data:
            self.add_gate(
                pos=gate["pos"],
                rpy=gate["rpy"],
                inner_width=0.4,
                outer_width=0.72
            )

        # --- 2. OBSTACLES (Vertical Poles) ---
        # From TOML: diameter = 0.03m (radius = 0.015m)
        # Height is ground (z=0) to the top (z=1.55)
        poles_data = [
            [0.0, 0.75, 1.55],
            [1.0, 0.25, 1.55],
            [-1.5, -0.25, 1.55],
            [-0.5, -0.75, 1.55],
        ]

        for pole_pos in poles_data:
            # We define a cylinder from ground to top height
            start_point = np.array([pole_pos[0], pole_pos[1], 0.0], dtype=np.float64)
            end_point = np.array([pole_pos[0], pole_pos[1], pole_pos[2]], dtype=np.float64)
            
            self.add_cylinder(
                start=start_point,
                end=end_point,
                radius=0.015 # Half of 0.03m diameter
            )


    def get_collision_expressions(self, x_sym: ca.MX) -> ca.MX:
        """Generates symbolic CasADi expressions for the solver."""
        constraints = []
        drone_pos = x_sym[0:3]

        for obs in self.obstacles:
            p1 = ca.MX(obs["p1"])
            p2 = ca.MX(obs["p2"])
            r_total = obs["r"] + self.safety_margin

            if obs["type"] == "sphere":
                # Formula: dist^2 - r^2 >= 0
                dist_sq = ca.sumsqr(drone_pos - p1)
                constraints.append(dist_sq - r_total**2)
            
            else: # Cylinder / Capsule
                # Formula: Distance from point to line segment
                v = p2 - p1
                w = drone_pos - p1
                
                # Project w onto v, clamped between 0 and 1
                t = ca.dot(w, v) / (ca.sumsqr(v) + 1e-9)
                t_clamped = ca.fmax(0, ca.fmin(1, t))
                
                # Closest point on the segment to the drone
                closest_point = p1 + t_clamped * v
                dist_sq = ca.sumsqr(drone_pos - closest_point)
                constraints.append(dist_sq - r_total**2)

        return ca.vcat(constraints)

    def render(
        self,
        sim: Sim,
        rgba: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.3)
    ) -> None:
        """Draws all obstacles in the simulation."""
        for obs in self.obstacles:
            if obs["type"] == "sphere":
                # draw_points expects an [N, 3] array
                point = obs["p1"].reshape(1, 3)
                # size is diameter, so we use 2 * radius
                draw_points(
                    sim, 
                    points=point, 
                    rgba=np.array(rgba), 
                    size=obs["r"] * 2.0
                )
            else:
                draw_capsule(
                    sim,
                    p1=obs["p1"],
                    p2=obs["p2"],
                    radius=obs["r"],
                    rgba=rgba
                )