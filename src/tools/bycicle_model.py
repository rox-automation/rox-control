#!/usr/bin/env python3
"""
Bicycle kinematics simulator.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math
from typing import NamedTuple


class RobotState(NamedTuple):
    """Represents the state of the robot in the simulation"""

    x: float = 0.0  # X position
    y: float = 0.0  # Y position
    theta: float = 0.0  # Orientation angle in radians
    v: float = 0.0  # Linear velocity
    steering_angle: float = 0.0  # Steering angle in radians
    time: float = 0.0  # Simulation time in seconds


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
    Bicycle kinematics simulator

    Semi-realistic simulation of a bicycle model with rate-limited steering and velocity changes.

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
        self.states: list[RobotState] = []

    def set_target_velocity(self, target_velocity: float) -> None:
        """Set target linear velocity"""
        self.velocity_model.setpoint = target_velocity

    def set_target_steering_angle(self, target_angle: float) -> None:
        """Set target steering angle"""
        self.steering_model.setpoint = target_angle

    def reset(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        v: float = 0.0,
        steering_angle: float = 0.0,
    ) -> None:
        """Reset simulator to initial state"""
        self.state = RobotState(
            x=x, y=y, theta=theta, v=v, steering_angle=steering_angle
        )
        self.velocity_model.val = v
        self.velocity_model.setpoint = v
        self.steering_model.val = steering_angle
        self.steering_model.setpoint = steering_angle
        self.states = []

    def step(self, dt: float) -> None:
        """Perform simulation step using bicycle kinematics"""
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

        # Create new state
        self.state = RobotState(
            x=new_x,
            y=new_y,
            theta=new_theta,
            v=v,
            steering_angle=phi,
            time=new_time,
        )

        # Store state history
        self.states.append(self.state)
