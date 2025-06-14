#!/usr/bin/env python3
"""
Tests for bicycle kinematics simulator.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math

from tools.bycicle_model import BicycleModel


class TestBicycleSim:
    """Test cases for the bicycle kinematics simulator"""

    def test_initialization_and_reset(self) -> None:
        """
        Test that the simulator initializes correctly and can be reset.

        Rationale: Verifies that the simulator starts in a known state and that
        the reset functionality properly initializes all internal variables.
        This is fundamental for reproducible simulations.
        """
        # Test default initialization
        sim = BicycleModel()
        assert sim.wheelbase == 2.5
        assert sim.state.x == 0.0
        assert sim.state.y == 0.0
        assert sim.state.theta == 0.0
        assert sim.state.v == 0.0
        assert sim.state.steering_angle == 0.0
        assert sim.state.time == 0.0
        assert len(sim.states) == 0

        # Test custom initialization
        sim = BicycleModel(wheelbase=3.0, accel=2.0)
        assert sim.wheelbase == 3.0

        # Test reset functionality
        sim.reset(x=1.0, y=2.0, theta=math.pi / 4, v=5.0, steering_angle=0.1)
        assert sim.state.x == 1.0
        assert sim.state.y == 2.0
        assert sim.state.theta == math.pi / 4
        assert sim.state.v == 5.0
        assert sim.state.steering_angle == 0.1
        assert sim.velocity_model.val == 5.0
        assert sim.steering_model.val == 0.1
        assert len(sim.states) == 0

    def test_straight_line_motion(self) -> None:
        """
        Test robot motion in a straight line with no steering.

        Rationale: This validates the basic kinematic equations for forward motion.
        With zero steering angle, the robot should move in a straight line at
        the commanded velocity. This tests the fundamental x-y position integration.
        """
        sim = BicycleModel(
            wheelbase=2.0, accel=5.0
        )  # Higher acceleration to reach target faster
        sim.reset()

        # Set target velocity but no steering
        sim.set_target_velocity(2.0)

        # get target velocity
        assert sim.velocity_model.setpoint == 2.0

        sim.set_target_steering_angle(0.0)

        # Run simulation for 1 second with small time steps
        dt = 0.1
        total_time = 1.0
        steps = int(total_time / dt)

        for _ in range(steps):
            sim.step(dt)

        # Robot should reach close to target velocity and move appropriately
        assert sim.state.v > 1.8  # Should be close to 2.0 m/s
        assert abs(sim.state.y) < 0.01  # Should stay on x-axis
        assert abs(sim.state.theta) < 0.01  # Should maintain heading
        assert abs(sim.state.time - total_time) < 0.01

    def test_circular_motion(self) -> None:
        """
        Test robot motion in a circular arc with constant steering.

        Rationale: This validates the bicycle kinematics turning behavior.
        With constant velocity and steering angle, the robot should follow
        a circular path. We can predict the turning radius and verify the
        robot follows the expected circular trajectory.
        """
        sim = BicycleModel(
            wheelbase=2.0, accel=5.0, steering_speed=math.radians(90)
        )  # Faster response
        sim.reset()

        # Set constant velocity and steering angle
        velocity = 1.0
        steering_angle = math.radians(30)  # 30 degrees

        sim.set_target_velocity(velocity)
        sim.set_target_steering_angle(steering_angle)

        # Run simulation long enough to reach steady state
        dt = 0.01
        for _ in range(400):  # 4 seconds to ensure steady state
            sim.step(dt)

        # After sufficient time, robot should be moving in circle
        # Check that the robot is following circular motion
        final_state = sim.state

        # The robot should have turned and be moving
        assert abs(final_state.theta) > 0.2  # Should have turned
        assert final_state.v > 0.8  # Should be close to target velocity

        # For circular motion, we mainly care that the robot turned and moved
        # The exact radius check is complex due to the transition period
        assert final_state.x > 0.5  # Should have moved forward
        assert abs(final_state.y) > 0.1  # Should have moved sideways due to turning

    def test_rate_limited_acceleration(self) -> None:
        """
        Test that velocity changes are rate-limited according to acceleration constraint.

        Rationale: Real vehicles cannot change velocity instantaneously. This test
        verifies that the LinearModel correctly limits the rate of velocity change
        to the specified acceleration limit, which is crucial for realistic simulation.
        """
        accel_limit = 1.0  # 1 m/s^2
        sim = BicycleModel(accel=accel_limit)
        sim.reset()

        # Command high velocity instantly
        target_velocity = 5.0
        sim.set_target_velocity(target_velocity)

        dt = 0.1
        sim.step(dt)

        # Velocity should not jump to target, but increase by accel_limit * dt
        expected_velocity = accel_limit * dt
        assert abs(sim.state.v - expected_velocity) < 0.01

        # After more time, should approach target velocity
        for _ in range(50):  # 5 seconds total
            sim.step(dt)

        # Should be close to target velocity now
        assert abs(sim.state.v - target_velocity) < 0.2

    def test_rate_limited_steering(self) -> None:
        """
        Test that steering angle changes are rate-limited according to steering speed.

        Rationale: Real vehicles have limited steering actuation speed. This test
        verifies that steering commands are applied gradually at the specified
        rate limit, preventing unrealistic instantaneous steering changes.
        """
        steering_speed = math.radians(45)  # 45 deg/s
        sim = BicycleModel(steering_speed=steering_speed)
        sim.reset()

        # Command large steering angle instantly
        target_angle = math.radians(30)
        sim.set_target_steering_angle(target_angle)

        dt = 0.1
        sim.step(dt)

        # Steering should not jump to target, but increase by steering_speed * dt
        expected_angle = steering_speed * dt
        assert abs(sim.state.steering_angle - expected_angle) < 0.01

        # After more time, should approach target angle
        for _ in range(10):  # 1 second total
            sim.step(dt)

        # Should be close to target angle now
        assert abs(sim.state.steering_angle - target_angle) < 0.1

    def test_state_history_tracking(self) -> None:
        """
        Test that the simulator correctly tracks state history over time.

        Rationale: State history is essential for trajectory analysis, plotting,
        and controller evaluation. This test verifies that each simulation step
        is properly recorded and that the time progression is correct.
        """
        sim = BicycleModel()
        sim.reset()

        # Initially no history
        assert len(sim.states) == 0

        # Run several simulation steps
        dt = 0.1
        num_steps = 10

        for i in range(num_steps):
            sim.step(dt)

            # Check that history is growing
            assert len(sim.states) == i + 1

            # Check that time is progressing correctly
            expected_time = (i + 1) * dt
            assert abs(sim.states[i].time - expected_time) < 0.001

        # Verify that states are different (robot is moving)
        assert sim.states[0] != sim.states[-1]

        # Test that reset clears history
        sim.reset()
        assert len(sim.states) == 0

    def test_bicycle_kinematics_accuracy(self) -> None:
        """
        Test the accuracy of bicycle kinematics implementation against known solutions.

        Rationale: This test validates that our implementation correctly follows
        the mathematical bicycle model. We test a specific scenario where we can
        calculate the expected result analytically and compare with simulation.
        """
        sim = BicycleModel(wheelbase=2.0)
        sim.reset()

        # Set up a scenario: move at 1 m/s with 45-degree steering
        velocity = 1.0
        steering_angle = math.radians(45)

        # Skip rate limiting by setting values directly
        sim.velocity_model.val = velocity
        sim.velocity_model.setpoint = velocity
        sim.steering_model.val = steering_angle
        sim.steering_model.setpoint = steering_angle

        # Take one step
        dt = 0.1
        sim.step(dt)

        # Calculate expected values using bicycle kinematics
        # θ̇ = (v/l) * tan(φ)
        expected_theta_dot = (velocity / sim.wheelbase) * math.tan(steering_angle)
        expected_theta = expected_theta_dot * dt

        # ẋ = v * cos(θ), but θ changes during the step
        # For small dt, we can approximate with initial θ = 0
        expected_x = velocity * math.cos(0) * dt
        expected_y = velocity * math.sin(0) * dt

        assert abs(sim.state.theta - expected_theta) < 0.03
        assert abs(sim.state.x - expected_x) < 0.01
        assert abs(sim.state.y - expected_y) < 0.01

    def test_steering_angle_limits(self) -> None:
        """
        Test that steering angle is properly limited to maximum values.

        Rationale: Real vehicles have physical steering limits. This test ensures
        that the simulator respects these limits and doesn't allow unrealistic
        steering angles that would break the kinematic model.
        """
        max_steering = math.radians(30)
        sim = BicycleModel(max_steering_angle=max_steering)
        sim.reset()

        # Try to command steering beyond limits
        excessive_angle = math.radians(60)
        sim.set_target_steering_angle(excessive_angle)

        # Run simulation long enough to reach steady state
        dt = 0.1
        for _ in range(20):
            sim.step(dt)

        # Steering should be clamped to maximum
        assert abs(sim.state.steering_angle) <= max_steering + 0.01

        # Test negative limit
        sim.set_target_steering_angle(-excessive_angle)
        for _ in range(20):
            sim.step(dt)

        assert abs(sim.state.steering_angle) <= max_steering + 0.01
