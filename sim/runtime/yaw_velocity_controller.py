"""Yaw-rate feedback around the physical Lite3 RL gait (no state overrides)."""

import math


class YawVelocityController:
    def __init__(self, policy_limit: float = 0.8) -> None:
        if not math.isfinite(policy_limit) or policy_limit < 0.24:
            raise ValueError("policy yaw limit must be finite and at least 0.24 rad/s")
        self.policy_limit = policy_limit
        self.gait_floor = 0.24
        self.kp = 0.4
        # Continuous turning uses rate feedback. The intermittent low-speed
        # gait uses integrated turn-angle error below instead of this gain.
        self.ki = 1.2
        self.angle_gain = 1.2
        self.filter_time = 0.2
        self.slew_rate = 2.0
        self.integral_limit = 0.12
        self.speed_integral_limit = 0.3
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.filtered_rate = 0.0
        self.output = 0.0
        self.previous_target = 0.0
        self.initialized = False
        self.angle_error = 0.0
        self.holding = False

    def update(self, target: float, measured: float, dt: float) -> float:
        if not all(math.isfinite(v) for v in (target, measured, dt)):
            self.reset()
            return 0.0
        if abs(target) < 1e-6:
            self.reset()
            return 0.0
        dt = max(0.0, min(dt, 0.05))
        creeping = abs(target) <= 0.10
        if (target * self.previous_target < 0.0
                or creeping != (abs(self.previous_target) <= 0.10)):
            self.integral = 0.0
            self.angle_error = 0.0
            self.holding = False
        self.previous_target = target
        if not self.initialized:
            self.filtered_rate = measured
            self.initialized = True
        self.filtered_rate += dt / (self.filter_time + dt) * (measured - self.filtered_rate)
        error = target - self.filtered_rate
        self.angle_error = max(-0.5, min(0.5,
            self.angle_error + (target - measured) * dt))
        # A nonzero gait command is needed to leave the stationary policy
        # equilibrium. Feedback adapts this seed instead of assuming a fixed
        # scale factor is an accurate inverse of the learned gait.
        sign = math.copysign(1.0, target)
        if creeping:
            # Use the true integrated rate error here; filtering or discarding
            # a start-up lag would lose requested angle at very low speeds.
            self.integral = max(-self.integral_limit, min(
                self.integral_limit, self.angle_gain * self.angle_error))
            # At creeping rates one gait initiation can consume more angle
            # than requested. Stop the policy command while ahead of the
            # integrated target; resume after the reference catches up.
            if sign * self.angle_error < -max(0.003, 0.04 * abs(target)):
                self.holding = True
            elif sign * self.angle_error >= 0.0:
                self.holding = False
            if self.holding:
                self.output = 0.0
                return 0.0
        else:
            self.holding = False
        # The pinned gait breaks away earlier clockwise than counterclockwise.
        # Keep the lower correction boundary below the measured response in
        # each direction so the feedback can also reduce very small rates.
        gait_floor = self.gait_floor if sign > 0.0 else 0.20
        feedforward = sign * (0.28 + 0.55 * abs(target))
        candidate = max(-self.speed_integral_limit, min(
            self.speed_integral_limit, self.integral + self.ki * error * dt))
        unconstrained = sign * (feedforward + self.kp * error + candidate)
        # Anti-windup at both limits: overshoot must not build a large
        # negative bias that strands the gait below its breakaway region.
        if not creeping and (gait_floor <= unconstrained <= self.policy_limit
                or (unconstrained > self.policy_limit and sign * error < 0.0)
                or (unconstrained < gait_floor and sign * error > 0.0)):
            self.integral = candidate
        requested = sign * (feedforward + self.kp * error + self.integral)
        requested = sign * max(gait_floor, min(self.policy_limit, requested))
        # Keep gait command changes bounded, including direction reversals.
        delta = self.slew_rate * dt
        self.output += max(-delta, min(delta, requested - self.output))
        return self.output
