"""ROS-independent gamepad mapping, rate limits and input loss handling."""
from dataclasses import dataclass
import math


def clamp(value, low, high):
    return max(low, min(high, value))


@dataclass
class Settings:
    steer_axis: int = 2
    arm_yaw_axis: int = 0
    arm_lift_axis: int = 1
    reverse_axis: int = 4
    forward_axis: int = 5
    trigger_rest: float = 0.0
    trigger_pressed: float = -1.0
    stop_button: int = 1
    deadzone: float = 0.08
    max_forward_speed: float = 0.6
    max_reverse_speed: float = 0.35
    max_yaw_rate: float = 0.8
    acceleration: float = 0.4
    angular_acceleration: float = 1.2
    joy_timeout: float = 0.75
    arm_speed: float = 0.3
    arm_yaw_lower: float = -0.90
    arm_yaw_upper: float = 0.90
    arm_lift_lower: float = -0.20
    arm_lift_upper: float = 0.55


class GamepadControl:
    def __init__(self, settings=None):
        self.s = settings or Settings()
        if not 0 <= self.s.deadzone < 1 or self.s.trigger_rest == self.s.trigger_pressed:
            raise ValueError('Invalid deadzone or trigger endpoints')
        if any(getattr(self.s, key) < 0 for key in (
                'steer_axis', 'arm_yaw_axis', 'arm_lift_axis',
                'reverse_axis', 'forward_axis', 'stop_button')):
            raise ValueError('Axis/button indices must be nonnegative')
        for key in ('max_forward_speed', 'max_reverse_speed',
                    'max_yaw_rate', 'acceleration', 'angular_acceleration', 'joy_timeout', 'arm_speed'):
            if not math.isfinite(getattr(self.s, key)) or getattr(self.s, key) <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not self.s.arm_yaw_lower < self.s.arm_yaw_upper or not self.s.arm_lift_lower < self.s.arm_lift_upper:
            raise ValueError('Invalid arm joint limits')
        self.axes = None
        self.buttons = None
        self.last_input = -math.inf
        self.armed = False
        self.arm_armed = False
        self.v = self.w = 0.0
        self.yaw = self.lift = 0.0
        self.arm_ready = False
        self.estop_input = False
        self.estop_latched = False
        self.drive_state = 'awaiting_input'

    def set_estop(self, active):
        """Latch any asserted external stop; deassertion alone cannot restart."""
        self.estop_input = bool(active)
        if self.estop_input:
            self.estop_latched = True
            self.armed = False
            self.arm_armed = False
            self.v = self.w = 0.0
            self.drive_state = 'estop_latched'

    def reset_estop(self):
        """Reset only after the physical/input stop signal has been released."""
        if self.estop_input:
            return False
        self.estop_latched = False
        self.armed = False
        self.arm_armed = False
        self.v = self.w = 0.0
        self.drive_state = 'awaiting_neutral'
        return True

    def receive(self, axes, buttons, now):
        required = max(self.s.steer_axis, self.s.arm_yaw_axis,
                       self.s.arm_lift_axis, self.s.reverse_axis, self.s.forward_axis)
        if (len(axes) <= required or len(buttons) <= self.s.stop_button
                or not all(math.isfinite(v) and abs(v) <= 1.001 for v in axes)):
            self.axes = None
            self.armed = False
            self.arm_armed = False
            self.drive_state = 'invalid_input'
            return False
        self.axes, self.buttons, self.last_input = list(axes), list(buttons), now
        return True

    def initialize_arm(self, yaw, lift):
        if not self.arm_ready and math.isfinite(yaw) and math.isfinite(lift):
            self.yaw = clamp(yaw, self.s.arm_yaw_lower, self.s.arm_yaw_upper)
            self.lift = clamp(lift, self.s.arm_lift_lower, self.s.arm_lift_upper)
            self.arm_ready = True

    def stick(self, index):
        value = self.axes[index]
        return math.copysign(max(0.0, (abs(value) - self.s.deadzone) / (1.0 - self.s.deadzone)), value)

    def trigger(self, index):
        value = (self.axes[index] - self.s.trigger_rest) / (self.s.trigger_pressed - self.s.trigger_rest)
        return clamp((value - self.s.deadzone) / (1.0 - self.s.deadzone), 0.0, 1.0)

    def step(self, now, dt):
        dt = clamp(dt, 0.0, 0.1)
        if self.estop_latched:
            self.armed = False
            self.arm_armed = False
            self.v = self.w = 0.0
            self.drive_state = 'estop_latched'
            return self.v, self.w, self.yaw, self.lift
        if self.axes is None:
            self.armed = False
            self.arm_armed = False
            self.v = self.w = 0.0
            if self.drive_state != 'invalid_input':
                self.drive_state = 'no_input'
            return self.v, self.w, self.yaw, self.lift
        if now - self.last_input > self.s.joy_timeout:
            self.armed = False
            self.arm_armed = False
            self.v = self.w = 0.0
            self.drive_state = 'joy_timeout'
            return self.v, self.w, self.yaw, self.lift
        if self.buttons[self.s.stop_button]:
            self.armed = False
            self.arm_armed = False
            self.v = self.w = 0.0
            self.drive_state = 'stop_button'
            return self.v, self.w, self.yaw, self.lift
        steer = self.stick(self.s.steer_axis)
        yaw, lift = self.stick(self.s.arm_yaw_axis), self.stick(self.s.arm_lift_axis)
        forward, reverse = self.trigger(self.s.forward_axis), self.trigger(self.s.reverse_axis)
        # Arm and base have independent neutral gates. A displaced/faulty arm
        # stick must not prevent driving; held arm input still cannot start it.
        if not self.arm_armed:
            if self.arm_ready and not any((yaw, lift)):
                self.arm_armed = True
        elif self.arm_ready:
            self.yaw = clamp(self.yaw + yaw * self.s.arm_speed * dt, self.s.arm_yaw_lower, self.s.arm_yaw_upper)
            self.lift = clamp(self.lift + lift * self.s.arm_speed * dt, self.s.arm_lift_lower, self.s.arm_lift_upper)
        if not self.armed:
            self.v = self.w = 0.0
            if not any((steer, forward, reverse)):
                self.armed = True
                self.drive_state = 'idle'
            else:
                self.drive_state = 'awaiting_neutral'
            return self.v, self.w, self.yaw, self.lift
        if forward and reverse:
            # Both triggers act as a brake, not competing forward/reverse commands.
            self.v = self.w = 0.0
            self.drive_state = 'both_triggers'
        else:
            # Linear motion is trigger-only. Right-stick Y is intentionally
            # ignored; right-stick X controls only the rover yaw rate.
            target = forward * self.s.max_forward_speed - reverse * self.s.max_reverse_speed
            # Stop before changing direction; acceleration remains bounded.
            if target * self.v < 0:
                target = 0.0
            self.v += clamp(target - self.v, -self.s.acceleration * dt, self.s.acceleration * dt)
            self.w += clamp(steer * self.s.max_yaw_rate - self.w,
                            -self.s.angular_acceleration * dt, self.s.angular_acceleration * dt)
            self.drive_state = 'active' if any((forward, reverse, steer, self.v, self.w)) else 'idle'
        return self.v, self.w, self.yaw, self.lift
