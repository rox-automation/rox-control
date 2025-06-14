#!/usr/bin/env python3
"""
Bycicle kinematics simulator.

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


class BycicleSim:
    """
    Bycicle kinematics simulator

    Semi-realistic simulation of a bycicle model with rate-limited steering and velocity changes.

    Note: steering angle is modeled as if it moeves with a constant angular velocity. This is a simplification (implies infinite angular acceleration), but makes the simulation a lot easier because we don't need to implement a controller for steering angle.

    """

    def __init__(
        self,
        accel: float = 1.0,
        steering_speed: float = math.radians(45),
        max_steering_angle: float = math.radians(45),
    ) -> None:
        """
        Args:
            accel: maximum linear acceleration in m/s^2
            steering_speed: maximum steering angle change in radians per second
            max_steering_angle: maximum steering angle in radians
        """

        self.state: RobotState = RobotState()
        self.velocity_model: LinearModel = LinearModel(roc=accel, val=self.state.v)
        self.steering_model: LinearModel = LinearModel(
            roc=steering_speed,
            val=self.state.steering_angle,
            min_val=-max_steering_angle,
            max_val=max_steering_angle,
        )
        self.states: list[RobotState] = []

    def set_target_velocity(self, target_velocity: float) -> None:
        """Set target linear velocity"""
        self.velocity_model._setpoint = target_velocity

    def set_target_steering_angle(self, target_angle: float) -> None:
        """Set target steering angle"""
        self.steering_model.setpoint = target_angle

    def step(self, dt: float) -> None:
        """Perform simulation step"""
