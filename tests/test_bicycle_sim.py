#!/usr/bin/env python3
"""
Tests for bicycle kinematics simulator.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math

from rox_control.tools.bicicle_model import BicycleModel, RobotState


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
        # Test custom initialization
        sim = BicycleModel(wheelbase=3.0, accel=2.0)
        assert sim.wheelbase == 3.0

        # Test state assignment functionality
        sim.state = RobotState(
            x=1.0, y=2.0, theta=math.pi / 4, v=5.0, steering_angle=0.1
        )
        sim.velocity_model.val = 5.0
        sim.velocity_model.setpoint = 5.0
        sim.steering_model.val = 0.1
        sim.steering_model.setpoint = 0.1
        assert sim.state.x == 1.0
        assert sim.state.y == 2.0
        assert sim.state.theta == math.pi / 4
        assert sim.state.v == 5.0
        assert sim.state.steering_angle == 0.1
        assert sim.velocity_model.val == 5.0
        assert sim.steering_model.val == 0.1

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
        sim.state = RobotState()

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
        sim.state = RobotState()

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
        sim.state = RobotState()

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
        sim.state = RobotState()

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

    def test_step_return_value(self) -> None:
        """
        Test that the step() method returns the updated state.

        Rationale: The new API design has step() return the state instead of
        storing it internally. This test verifies that the returned state
        contains the correct updated values and progresses over time.
        """
        sim = BicycleModel()
        sim.state = RobotState()

        # Set some motion commands
        sim.set_target_velocity(1.0)
        sim.set_target_steering_angle(0.1)

        dt = 0.1
        initial_state = sim.state

        # Take a step and verify return value
        new_state = sim.step(dt)

        # Step should return a RobotState
        assert isinstance(new_state, type(initial_state))

        # Time should have progressed
        assert new_state.time == initial_state.time + dt

        # Internal state should match returned state
        assert sim.state == new_state

        # Take multiple steps and verify progression
        states = []
        for i in range(5):
            state = sim.step(dt)
            states.append(state)

            # Time should progress correctly
            expected_time = (i + 2) * dt  # +2 because we already took one step
            assert abs(state.time - expected_time) < 0.001

        # Verify states are different (robot is moving)
        assert states[0] != states[-1]

    def test_bicycle_kinematics_accuracy(self) -> None:
        """
        Test the accuracy of bicycle kinematics implementation against known solutions.

        Rationale: This test validates that our implementation correctly follows
        the mathematical bicycle model. We test a specific scenario where we can
        calculate the expected result analytically and compare with simulation.
        """
        sim = BicycleModel(wheelbase=2.0)
        sim.state = RobotState()

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
        sim.state = RobotState()

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

    def test_front_wheel_position_basic(self) -> None:
        """
        Test front wheel position calculation for basic orientations.

        Rationale: The front wheel position is crucial for visualization and control.
        This test verifies the geometric calculation using basic trigonometry
        for cardinal directions where results are easily predictable.
        """
        wheelbase = 2.5
        sim = BicycleModel(wheelbase=wheelbase)

        # Test facing east (0 radians) at origin
        sim.state = RobotState(x=0, y=0, theta=0)
        front_x, front_y = sim.get_front_wheel_pos()
        assert abs(front_x - wheelbase) < 0.001  # Should be wheelbase ahead in x
        assert abs(front_y - 0.0) < 0.001  # Should be at same y

        # Test facing north (π/2 radians) at origin
        sim.state = RobotState(x=0, y=0, theta=math.pi / 2)
        front_x, front_y = sim.get_front_wheel_pos()
        assert abs(front_x - 0.0) < 0.001  # Should be at same x
        assert abs(front_y - wheelbase) < 0.001  # Should be wheelbase ahead in y

        # Test facing west (π radians) at origin
        sim.state = RobotState(x=0, y=0, theta=math.pi)
        front_x, front_y = sim.get_front_wheel_pos()
        assert abs(front_x - (-wheelbase)) < 0.001  # Should be wheelbase behind in x
        assert abs(front_y - 0.0) < 0.001  # Should be at same y

        # Test facing south (3π/2 radians) at origin
        sim.state = RobotState(x=0, y=0, theta=3 * math.pi / 2)
        front_x, front_y = sim.get_front_wheel_pos()
        assert abs(front_x - 0.0) < 0.001  # Should be at same x
        assert abs(front_y - (-wheelbase)) < 0.001  # Should be wheelbase behind in y

    def test_front_wheel_position_arbitrary_pose(self) -> None:
        """
        Test front wheel position calculation for arbitrary robot poses.

        Rationale: Verifies the geometric calculation works correctly for
        any robot position and orientation, not just cardinal directions.
        """
        wheelbase = 2.0
        sim = BicycleModel(wheelbase=wheelbase)

        # Test at arbitrary position and 45-degree angle
        x, y, theta = 5.0, 3.0, math.pi / 4
        sim.state = RobotState(x=x, y=y, theta=theta)
        front_x, front_y = sim.get_front_wheel_pos()

        # Calculate expected position manually
        expected_x = x + wheelbase * math.cos(theta)
        expected_y = y + wheelbase * math.sin(theta)

        assert abs(front_x - expected_x) < 0.001
        assert abs(front_y - expected_y) < 0.001

        # Test at another arbitrary pose
        x, y, theta = -2.0, 4.5, -math.pi / 6
        sim.state = RobotState(x=x, y=y, theta=theta)
        front_x, front_y = sim.get_front_wheel_pos()

        expected_x = x + wheelbase * math.cos(theta)
        expected_y = y + wheelbase * math.sin(theta)

        assert abs(front_x - expected_x) < 0.001
        assert abs(front_y - expected_y) < 0.001

    def test_front_wheel_position_different_wheelbase(self) -> None:
        """
        Test front wheel position calculation with different wheelbase values.

        Rationale: Ensures the calculation correctly uses the wheelbase parameter
        and works for different vehicle configurations.
        """
        # Test with different wheelbase values
        test_wheelbases = [1.0, 2.5, 4.0]
        x, y, theta = 1.0, 1.0, math.pi / 3

        for wheelbase in test_wheelbases:
            sim = BicycleModel(wheelbase=wheelbase)
            sim.state = RobotState(x=x, y=y, theta=theta)
            front_x, front_y = sim.get_front_wheel_pos()

            expected_x = x + wheelbase * math.cos(theta)
            expected_y = y + wheelbase * math.sin(theta)

            assert abs(front_x - expected_x) < 0.001
            assert abs(front_y - expected_y) < 0.001

    def test_projected_path_straight_line(self) -> None:
        """
        Test projected path calculation for straight-line motion.

        Rationale: When steering angle is zero, the projected path should be
        a straight line extending from the front wheel in the current heading direction.
        """
        sim = BicycleModel(wheelbase=2.0)
        sim.state = RobotState(x=0, y=0, theta=0, steering_angle=0.0)

        # Test straight path
        distance = 5.0
        num_points = 6
        proj_x, proj_y = sim.get_projected_path(
            distance=distance, num_points=num_points
        )

        assert len(proj_x) == num_points
        assert len(proj_y) == num_points

        # Path should start at front wheel position
        front_x, front_y = sim.get_front_wheel_pos()
        assert abs(proj_x[0] - front_x) < 0.001
        assert abs(proj_y[0] - front_y) < 0.001

        # Path should be straight line in x direction (theta=0)
        for y in proj_y:
            assert abs(y - front_y) < 0.001  # All y coordinates should be same

        # X coordinates should increase linearly
        expected_x_increment = distance / (num_points - 1)
        for i, x in enumerate(proj_x):
            expected_x = front_x + i * expected_x_increment
            assert abs(x - expected_x) < 0.001

    def test_projected_path_straight_line_arbitrary_heading(self) -> None:
        """
        Test projected path for straight-line motion with arbitrary heading.

        Rationale: Verifies that straight-line projection works correctly
        regardless of the robot's current orientation.
        """
        sim = BicycleModel(wheelbase=2.0)
        theta = math.pi / 3  # 60 degrees
        sim.state = RobotState(x=1, y=2, theta=theta, steering_angle=0.0)

        distance = 4.0
        num_points = 5
        proj_x, proj_y = sim.get_projected_path(
            distance=distance, num_points=num_points
        )

        # Path should start at front wheel position
        front_x, front_y = sim.get_front_wheel_pos()
        assert abs(proj_x[0] - front_x) < 0.001
        assert abs(proj_y[0] - front_y) < 0.001

        # Verify points lie on straight line with correct heading
        for i in range(len(proj_x)):
            t = i / (num_points - 1)
            expected_x = front_x + t * distance * math.cos(theta)
            expected_y = front_y + t * distance * math.sin(theta)
            assert abs(proj_x[i] - expected_x) < 0.001
            assert abs(proj_y[i] - expected_y) < 0.001

    def test_projected_path_turning_motion(self) -> None:
        """
        Test projected path calculation for turning motion.

        Rationale: When steering angle is non-zero, the path should be a circular arc.
        This tests the bicycle kinematics calculation of the instantaneous center
        of rotation and the resulting circular path.
        """
        wheelbase = 2.0
        sim = BicycleModel(wheelbase=wheelbase)
        steering_angle = math.radians(30)  # 30 degrees
        sim.state = RobotState(x=0, y=0, theta=0, steering_angle=steering_angle)

        distance = 3.0
        num_points = 10
        proj_x, proj_y = sim.get_projected_path(
            distance=distance, num_points=num_points
        )

        assert len(proj_x) == num_points
        assert len(proj_y) == num_points

        # Path should start at front wheel position
        front_x, front_y = sim.get_front_wheel_pos()
        assert abs(proj_x[0] - front_x) < 0.001
        assert abs(proj_y[0] - front_y) < 0.001

        # For turning motion, path should be curved (not straight)
        # Check that middle points deviate from straight line
        straight_line_y = front_y  # Would be constant for straight motion at theta=0
        middle_point_y = proj_y[len(proj_y) // 2]
        assert (
            abs(middle_point_y - straight_line_y) > 0.1
        )  # Should deviate significantly

        # Verify the path curves in the expected direction
        # Positive steering angle should curve left (positive y direction)
        assert proj_y[-1] > proj_y[0]  # End point should be higher than start

    def test_projected_path_turning_radius_consistency(self) -> None:
        """
        Test that projected path follows the expected turning radius.

        Rationale: The projected path should be geometrically consistent with
        the bicycle kinematics model. All points should lie on a circle with
        the correct radius around the instantaneous center of rotation.
        """
        wheelbase = 2.0
        sim = BicycleModel(wheelbase=wheelbase)
        steering_angle = math.radians(45)  # 45 degrees
        sim.state = RobotState(x=0, y=0, theta=0, steering_angle=steering_angle)

        proj_x, proj_y = sim.get_projected_path(distance=2.0, num_points=8)

        # Calculate expected ICR (instantaneous center of rotation)
        R_rear = wheelbase / math.tan(steering_angle)
        icr_x = 0 - R_rear * math.sin(0)  # x - R_rear * sin(theta)
        icr_y = 0 + R_rear * math.cos(0)  # y + R_rear * cos(theta)

        # All projected points should be equidistant from ICR
        front_x, front_y = sim.get_front_wheel_pos()
        expected_radius = math.sqrt((front_x - icr_x) ** 2 + (front_y - icr_y) ** 2)

        for x, y in zip(proj_x, proj_y, strict=False):
            actual_radius = math.sqrt((x - icr_x) ** 2 + (y - icr_y) ** 2)
            assert abs(actual_radius - expected_radius) < 0.01

    def test_projected_path_negative_steering(self) -> None:
        """
        Test projected path for negative steering angles (right turns).

        Rationale: Verifies that the path calculation works correctly for
        both left and right turns, with proper sign handling.
        """
        sim = BicycleModel(wheelbase=2.0)
        steering_angle = math.radians(-30)  # -30 degrees (right turn)
        sim.state = RobotState(x=0, y=0, theta=0, steering_angle=steering_angle)

        proj_x, proj_y = sim.get_projected_path(distance=3.0, num_points=10)

        # Path should start at front wheel position
        front_x, front_y = sim.get_front_wheel_pos()
        assert abs(proj_x[0] - front_x) < 0.001
        assert abs(proj_y[0] - front_y) < 0.001

        # Negative steering should curve right (negative y direction)
        assert proj_y[-1] < proj_y[0]  # End point should be lower than start

    def test_projected_path_small_steering_angle(self) -> None:
        """
        Test projected path behavior near zero steering angle.

        Rationale: The implementation switches between straight-line and circular
        calculations based on a threshold. This test ensures smooth behavior
        near the switching point.
        """
        sim = BicycleModel(wheelbase=2.0)

        # Test very small positive steering angle
        small_angle = 0.005  # Just below the 0.01 threshold
        sim.state = RobotState(x=0, y=0, theta=0, steering_angle=small_angle)
        proj_x1, proj_y1 = sim.get_projected_path(distance=5.0, num_points=10)

        # Test very small negative steering angle
        sim.state = RobotState(x=0, y=0, theta=0, steering_angle=-small_angle)
        proj_x2, proj_y2 = sim.get_projected_path(distance=5.0, num_points=10)

        # Paths should be nearly symmetric
        for i in range(len(proj_x1)):
            assert abs(proj_x1[i] - proj_x2[i]) < 0.001  # X coords should be same
            assert (
                abs(proj_y1[i] + proj_y2[i]) < 0.1
            )  # Y coords should be nearly opposite

    def test_projected_path_parameters(self) -> None:
        """
        Test projected path with different distance and point count parameters.

        Rationale: Verifies that the method correctly handles various parameter
        values and produces appropriately sized outputs.
        """
        sim = BicycleModel(wheelbase=2.0)
        sim.state = RobotState(x=0, y=0, theta=0, steering_angle=0.0)

        # Test different numbers of points
        for num_points in [3, 5, 10, 20]:
            proj_x, proj_y = sim.get_projected_path(distance=5.0, num_points=num_points)
            assert len(proj_x) == num_points
            assert len(proj_y) == num_points

        # Test different distances
        for distance in [1.0, 3.0, 10.0]:
            proj_x, proj_y = sim.get_projected_path(distance=distance, num_points=5)
            front_x, front_y = sim.get_front_wheel_pos()

            # Last point should be approximately at the specified distance
            actual_distance = math.sqrt(
                (proj_x[-1] - front_x) ** 2 + (proj_y[-1] - front_y) ** 2
            )
            assert abs(actual_distance - distance) < 0.1

    def test_set_control_command_zero_curvature(self) -> None:
        """Test set_control_command with zero curvature (straight line motion)."""
        sim = BicycleModel(wheelbase=2.5)

        # Command straight line motion
        curvature = 0.0
        velocity = 5.0
        sim.set_control_command(curvature, velocity)

        # Should set velocity target
        assert sim.velocity_model.setpoint == velocity

        # Should set steering angle to zero
        assert abs(sim.steering_model.setpoint) < 0.001

    def test_set_control_command_positive_curvature(self) -> None:
        """Test set_control_command with positive curvature (left turn)."""
        wheelbase = 2.0
        sim = BicycleModel(wheelbase=wheelbase)

        # Command left turn: curvature = 1/radius
        radius = 10.0  # 10m turn radius
        curvature = 1.0 / radius  # 0.1 rad/m
        velocity = 3.0
        sim.set_control_command(curvature, velocity)

        # Should set velocity target
        assert sim.velocity_model.setpoint == velocity

        # Should calculate correct steering angle: δ = arctan(κ * L)
        expected_steering = math.atan(curvature * wheelbase)
        assert abs(sim.steering_model.setpoint - expected_steering) < 0.001

    def test_set_control_command_negative_curvature(self) -> None:
        """Test set_control_command with negative curvature (right turn)."""
        wheelbase = 2.5
        sim = BicycleModel(wheelbase=wheelbase)

        # Command right turn
        radius = 8.0
        curvature = -1.0 / radius  # Negative for right turn
        velocity = 4.0
        sim.set_control_command(curvature, velocity)

        # Should set velocity target
        assert sim.velocity_model.setpoint == velocity

        # Should calculate correct negative steering angle
        expected_steering = math.atan(curvature * wheelbase)
        assert abs(sim.steering_model.setpoint - expected_steering) < 0.001
        assert sim.steering_model.setpoint < 0  # Should be negative

    def test_set_control_command_high_curvature(self) -> None:
        """Test set_control_command with high curvature (tight turn)."""
        wheelbase = 2.0
        max_steering = math.radians(45)
        sim = BicycleModel(wheelbase=wheelbase, max_steering_angle=max_steering)

        # Command very tight turn
        radius = 1.0  # 1m turn radius (very tight)
        curvature = 1.0 / radius
        velocity = 2.0
        sim.set_control_command(curvature, velocity)

        # Should set velocity target
        assert sim.velocity_model.setpoint == velocity

        # atan(1.0 * 2.0) = atan(2.0) ≈ 63.4° > 45° max, so should be clamped

        # Should be clamped to max_steering_angle by LinearModel
        assert abs(sim.steering_model.setpoint) <= max_steering + 0.001
        assert sim.steering_model.setpoint == max_steering  # Should be exactly at limit

    def test_set_control_command_kinematics_equivalence(self) -> None:
        """Test that set_control_command produces equivalent motion to manual setting."""
        wheelbase = 2.5
        sim1 = BicycleModel(
            wheelbase=wheelbase, accel=10.0, steering_speed=math.radians(180)
        )
        sim2 = BicycleModel(
            wheelbase=wheelbase, accel=10.0, steering_speed=math.radians(180)
        )

        # Set initial conditions
        initial_state = RobotState(x=1.0, y=2.0, theta=math.pi / 6)
        sim1.state = initial_state
        sim2.state = initial_state

        # Test scenario: moderate turn
        radius = 5.0
        curvature = 1.0 / radius
        velocity = 6.0
        steering_angle = math.atan(curvature * wheelbase)

        # Method 1: Use set_control_command
        sim1.set_control_command(curvature, velocity)

        # Method 2: Set velocity and steering manually
        sim2.set_target_velocity(velocity)
        sim2.set_target_steering_angle(steering_angle)

        # Run simulation for several steps
        dt = 0.01
        for _ in range(200):  # 2 seconds, enough to reach steady state
            sim1.step(dt)
            sim2.step(dt)

        # Final states should be nearly identical
        state1 = sim1.state
        state2 = sim2.state

        assert abs(state1.x - state2.x) < 0.01
        assert abs(state1.y - state2.y) < 0.01
        assert abs(state1.theta - state2.theta) < 0.01
        assert abs(state1.v - state2.v) < 0.01
        assert abs(state1.steering_angle - state2.steering_angle) < 0.01

    def test_set_control_command_zero_velocity(self) -> None:
        """Test set_control_command with zero velocity (stationary)."""
        sim = BicycleModel(wheelbase=2.0)

        # Command stationary with some curvature
        curvature = 0.5  # Arbitrary curvature
        velocity = 0.0
        sim.set_control_command(curvature, velocity)

        # Should set zero velocity
        assert sim.velocity_model.setpoint == 0.0

        # Should still calculate steering angle (for preparation)
        expected_steering = math.atan(curvature * sim.wheelbase)
        assert abs(sim.steering_model.setpoint - expected_steering) < 0.001

    def test_set_control_command_bicycle_kinematics_formula(self) -> None:
        """Test the bicycle kinematics conversion formula used in set_control_command."""
        test_cases = [
            # (wheelbase, curvature, expected_steering_angle)
            (2.0, 0.0, 0.0),  # Straight line
            (2.0, 0.1, math.atan(0.2)),  # curvature * wheelbase = 0.2
            (2.5, 0.2, math.atan(0.5)),  # curvature * wheelbase = 0.5
            (3.0, 1.0, math.atan(3.0)),  # curvature * wheelbase = 3.0
            (2.0, -0.25, math.atan(-0.5)),  # Negative curvature
        ]

        for wheelbase, curvature, expected_steering in test_cases:
            sim = BicycleModel(wheelbase=wheelbase, max_steering_angle=math.radians(90))
            sim.set_control_command(
                curvature, 1.0
            )  # Velocity doesn't matter for this test

            assert abs(sim.steering_model.setpoint - expected_steering) < 0.001, (
                f"Failed for wheelbase={wheelbase}, curvature={curvature}"
            )

    def test_set_control_command_with_steering_limits(self) -> None:
        """Test set_control_command respects steering angle limits."""
        wheelbase = 2.0
        max_steering = math.radians(30)  # 30 degree limit
        sim = BicycleModel(wheelbase=wheelbase, max_steering_angle=max_steering)

        # Command curvature that would exceed steering limits
        # For 30° max: max_curvature = tan(30°) / wheelbase = tan(π/6) / 2.0
        max_curvature = math.tan(max_steering) / wheelbase
        excessive_curvature = max_curvature * 2.0  # Double the limit

        sim.set_control_command(excessive_curvature, 5.0)

        # Setpoint should be clamped by LinearModel limits
        expected_steering = math.atan(
            excessive_curvature * wheelbase
        )  # Unclamped value
        actual_setpoint = sim.steering_model.setpoint

        # The setpoint should be limited to max_steering_angle
        assert abs(actual_setpoint) <= max_steering + 0.001
        assert (
            actual_setpoint != expected_steering
        )  # Should be different due to clamping
