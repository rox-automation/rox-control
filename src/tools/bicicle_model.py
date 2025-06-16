#!/usr/bin/env python3
"""
Bicycle kinematics simulator.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RobotState:
    """Represents the state of the robot in the simulation"""

    x: float = 0.0  # X position (rear wheel)
    y: float = 0.0  # Y position (rear wheel)
    theta: float = 0.0  # Orientation angle in radians
    v: float = 0.0  # Linear velocity
    steering_angle: float = 0.0  # Steering angle in radians
    time: float = 0.0  # Simulation time in seconds
    front_x: float = 0.0  # X position of front wheel
    front_y: float = 0.0  # Y position of front wheel


class LinearModel:
    """Simple linear model for rate-limited changes"""

    def __init__(
        self, roc: float, val: float = 0.0, min_val: float = -0.5, max_val: float = 0.5
    ) -> None:
        """
        Args:
            roc: rate of change per second
            val: initial value
            min_val: minimum value limit
            max_val: maximum value limit
        """
        self.val: float = val
        self._setpoint: float = val
        self.roc: float = roc
        self.min_val: float = min_val
        self.max_val: float = max_val

    @property
    def setpoint(self) -> float:
        """Get the current setpoint value"""
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value: float) -> None:
        """Set a new setpoint value"""
        self._setpoint = max(self.min_val, min(self.max_val, value))

    def step(self, dt: float) -> None:
        """Update value toward setpoint at limited rate"""
        error: float = self._setpoint - self.val
        step: float = math.copysign(1, error) * self.roc * dt

        if abs(step) > abs(error):
            self.val += error
        else:
            self.val += step

        # Apply limits
        self.val = max(self.min_val, min(self.max_val, self.val))


class BicycleModel:
    """
    Bicycle kinematics model

    Models a bicycle with rate-limited steering and velocity changes.

    Note: steering angle is modeled as if it moves with a constant angular velocity. This is a simplification (implies infinite angular acceleration), but makes the simulation a lot easier because we don't need to implement a controller for steering angle.

    """

    def __init__(
        self,
        wheelbase: float = 2.5,
        accel: float = 1.0,
        steering_speed: float = math.radians(45),
        max_steering_angle: float = math.radians(45),
        max_velocity: float = 10.0,
    ) -> None:
        """
        Args:
            wheelbase: distance between front and rear axles in meters
            accel: maximum linear acceleration in m/s^2
            steering_speed: maximum steering angle change in radians per second
            max_steering_angle: maximum steering angle in radians
            max_velocity: maximum velocity in m/s
        """

        self.wheelbase: float = wheelbase
        self.state: RobotState = RobotState()
        self.velocity_model: LinearModel = LinearModel(
            roc=accel, val=self.state.v, min_val=-max_velocity, max_val=max_velocity
        )
        self.steering_model: LinearModel = LinearModel(
            roc=steering_speed,
            val=self.state.steering_angle,
            min_val=-max_steering_angle,
            max_val=max_steering_angle,
        )

    def set_target_velocity(self, target_velocity: float) -> None:
        """Set target linear velocity"""
        self.velocity_model.setpoint = target_velocity

    def set_target_steering_angle(self, target_angle: float) -> None:
        """Set target steering angle"""
        self.steering_model.setpoint = target_angle

    def set_control_command(self, curvature: float, velocity: float) -> None:
        """Set control command using curvature and velocity.

        Converts curvature to steering angle using bicycle kinematics:
        steering_angle = arctan(curvature * wheelbase)

        Args:
            curvature: Desired path curvature (1/radius) in rad/m
            velocity: Desired linear velocity in m/s
        """
        # Convert curvature to steering angle using bicycle kinematics
        # κ = tan(δ) / L  =>  δ = arctan(κ * L)
        steering_angle = math.atan(curvature * self.wheelbase)

        # Set both commands
        self.set_target_velocity(velocity)
        self.set_target_steering_angle(steering_angle)

    def step(self, dt: float) -> RobotState:
        """Perform simulation step using bicycle kinematics

        Args:
            dt: Time step in seconds

        Returns:
            Updated robot state after the time step
        """
        # Update linear models with time step
        self.velocity_model.step(dt)
        self.steering_model.step(dt)

        # Get current state values
        v = self.velocity_model.val
        phi = self.steering_model.val
        x, y, theta = self.state.x, self.state.y, self.state.theta

        # Apply bicycle kinematics (forward integration)
        # ẋ = v cos(θ)
        # ẏ = v sin(θ)
        # θ̇ = (v/l) tan(φ)
        x_dot = v * math.cos(theta)
        y_dot = v * math.sin(theta)
        theta_dot = (v / self.wheelbase) * math.tan(phi)

        # Integrate position and orientation
        new_x = x + x_dot * dt
        new_y = y + y_dot * dt
        new_theta = theta + theta_dot * dt

        # Update simulation time
        new_time = self.state.time + dt

        # Calculate front wheel position
        front_x = new_x + self.wheelbase * math.cos(new_theta)
        front_y = new_y + self.wheelbase * math.sin(new_theta)

        # Create new state
        self.state = RobotState(
            x=new_x,
            y=new_y,
            theta=new_theta,
            v=v,
            steering_angle=phi,
            time=new_time,
            front_x=front_x,
            front_y=front_y,
        )

        return self.state

    def get_front_wheel_pos(self) -> tuple[float, float]:
        """Get front wheel position based on current state and wheelbase."""
        front_x: float = self.state.x + self.wheelbase * math.cos(self.state.theta)
        front_y: float = self.state.y + self.wheelbase * math.sin(self.state.theta)
        return front_x, front_y

    def get_projected_path(
        self, distance: float = 5.0, num_points: int = 20
    ) -> tuple[list[float], list[float]]:
        """Get projected path based on current steering angle.

        Args:
            distance: How far ahead to project the path in meters
            num_points: Number of points to generate along the path

        Returns:
            Tuple of (x_coords, y_coords) lists representing the projected path
        """
        # Start from front wheel position
        front_x, front_y = self.get_front_wheel_pos()

        # Create parameter array
        t: np.ndarray = np.linspace(0, 1, num_points)

        if abs(self.state.steering_angle) < 0.01:  # Going straight
            d: np.ndarray = t * distance
            proj_x: np.ndarray = front_x + d * math.cos(self.state.theta)
            proj_y: np.ndarray = front_y + d * math.sin(self.state.theta)
            return proj_x.tolist(), proj_y.tolist()

        # Calculate turning radius for rear wheel
        R_rear: float = self.wheelbase / math.tan(self.state.steering_angle)

        # Instantaneous center of rotation (ICR)
        icr_x: float = self.state.x - R_rear * math.sin(self.state.theta)
        icr_y: float = self.state.y + R_rear * math.cos(self.state.theta)

        # Calculate radius for front wheel (distance from ICR to front wheel)
        R_front: float = math.sqrt((front_x - icr_x) ** 2 + (front_y - icr_y) ** 2)

        # Calculate projected path starting from front wheel
        start_angle: float = math.atan2(front_y - icr_y, front_x - icr_x)
        arc_length: float = distance / R_front

        angles: np.ndarray = start_angle + t * arc_length * np.sign(
            self.state.steering_angle
        )
        proj_x = icr_x + R_front * np.cos(angles)
        proj_y = icr_y + R_front * np.sin(angles)

        return proj_x.tolist(), proj_y.tolist()
