import math
from rover_bringup.control import GamepadControl


def ready():
    c = GamepadControl()
    c.initialize_arm(0.0, 0.0)
    c.receive([0.] * 6, [0] * 15, 0.)
    c.step(0., .02)
    assert c.armed
    return c


def test_neutral_start_and_trigger_ramp():
    c = GamepadControl()
    c.receive([0, 0, 0, 0, 0, -1], [0]*15, 0.)
    assert c.step(0., .1)[0] == 0  # held trigger at connection cannot start motion
    c = ready()
    for i in range(20):
        c.receive([0, 0, 0, 0, 0, -1], [0]*15, i*.1)
        v, w, _, _ = c.step(i*.1, .1)
    assert math.isclose(v, .6) and w == 0


def test_reverse_trigger_and_steering():
    c = ready()
    c.receive([0, 0, 1, 0, -1, 0], [0]*15, .1)
    v, w, _, _ = c.step(.1, .1)
    assert v < 0 and w > 0
    c = ready()
    c.receive([0, 0, 0, 1, 0, 0], [0]*15, .1)
    assert c.step(.1, .1)[:2] == (0., 0.)


def test_timeout_stops_and_requires_neutral_again():
    c = ready()
    c.receive([0, 0, 0, 0, 0, -1], [0]*15, .1)
    assert c.step(.1, .1)[0] > 0
    assert c.step(1., .1)[:2] == (0., 0.)
    assert c.drive_state == 'joy_timeout'
    c.receive([0, 0, 0, 0, 0, -1], [0]*15, 1.1)
    assert c.step(1.1, .1)[0] == 0
    assert c.drive_state == 'awaiting_neutral'


def test_stop_and_both_trigger_brake():
    c = ready()
    c.receive([0, 0, 1, 0, -1, -1], [0]*15, .1)
    assert c.step(.1, .1)[:2] == (0., 0.)
    assert c.drive_state == 'both_triggers'
    c.receive([1, 1, 1, 1, 0, -1], [0, 1] + [0]*13, .2)
    assert c.step(.2, .1) == (0., 0., 0., 0.)
    assert c.drive_state == 'stop_button'


def test_arm_limits_hold_and_paused_simulation():
    c = ready()
    for i in range(100):
        c.receive([1, 1, 0, 0, 0, 0], [0]*15, i*.1)
        c.step(i*.1, .1)
    assert c.yaw == c.s.arm_yaw_upper and c.lift == c.s.arm_lift_upper
    c.receive([-1, -1, 0, 0, 0, 0], [0]*15, 10.)
    c.step(10., 0.)
    assert c.yaw == c.s.arm_yaw_upper
    c.receive([0]*6, [0]*15, 10.1)
    c.step(10.1, .1)
    assert c.lift == c.s.arm_lift_upper


def test_bad_input_stops():
    c = ready()
    for axes in ([0], [float('nan')]*6, [2.]*6):
        assert not c.receive(axes, [0]*15, .1)
        assert c.step(.1, .1)[:2] == (0., 0.)


def test_displaced_arm_stick_does_not_block_drive():
    c = GamepadControl()
    c.initialize_arm(0., 0.)
    c.receive([1, 0, 0, 0, 0, 0], [0]*15, 0.)
    c.step(0., .1)
    assert c.armed and not c.arm_armed
    c.receive([1, 0, 0, 0, 0, -1], [0]*15, .1)
    v, _, yaw, lift = c.step(.1, .1)
    assert v > 0 and yaw == 0 and lift == 0


def test_held_throttle_does_not_block_independent_arm():
    c = GamepadControl()
    c.initialize_arm(0., 0.)
    c.receive([0, 0, 0, 0, 0, -1], [0]*15, 0.)
    c.step(0., .1)
    assert not c.armed and c.arm_armed
    c.receive([.5, 0, 0, 0, 0, -1], [0]*15, .1)
    v, _, yaw, _ = c.step(.1, .1)
    assert v == 0 and yaw > 0


def test_sticks_are_independent_and_right_stick_only_steers():
    c = ready()
    for i in range(30):
        c.receive([0, 0, 1, 1, 0, 0], [0]*15, i*.1)
        v, w, yaw, lift = c.step(i*.1, .1)
    assert v == 0 and math.isclose(w, .8)
    assert yaw == lift == 0
    c = ready()
    for i in range(30):
        c.receive([0, 0, 0, -1, 0, 0], [0]*15, i*.1)
        v, _, _, _ = c.step(i*.1, .1)
    assert v == 0
    c = ready()
    c.receive([1, 1, 0, 0, 0, 0], [0]*15, .1)
    v, w, yaw, lift = c.step(.1, .1)
    assert v == w == 0 and yaw > 0 and lift > 0


def test_right_stick_y_does_not_block_arming_or_trigger_throttle():
    c = GamepadControl()
    c.initialize_arm(0., 0.)
    c.receive([0, 0, 0, 1, 0, 0], [0]*15, 0.)
    assert c.step(0., .1)[:2] == (0., 0.)
    assert c.armed
    c.receive([0, 0, 0, 1, 0, -1], [0]*15, .1)
    v, w, _, _ = c.step(.1, .1)
    assert v > 0 and w == 0


def test_external_estop_latches_until_released_and_reset():
    c = ready()
    c.receive([0, 0, 0, 0, 0, -1], [0]*15, .1)
    assert c.step(.1, .1)[0] > 0
    c.set_estop(True)
    assert c.step(.2, .1)[:2] == (0., 0.)
    assert c.estop_latched and c.drive_state == 'estop_latched'
    assert not c.reset_estop()
    c.set_estop(False)
    assert c.step(.3, .1)[:2] == (0., 0.)  # false does not silently restart
    assert c.estop_latched
    assert c.reset_estop()
    assert not c.estop_latched and c.drive_state == 'awaiting_neutral'
