"""Game-style manual operation. Sole cmd_vel owner in carto_gamepad.launch.py."""
from dataclasses import fields
import json
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from .control import GamepadControl, Settings


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__('gamepad_teleop')
        defaults = Settings()
        values = {f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value for f in fields(defaults)}
        self.control = GamepadControl(Settings(**values))
        self.drive = self.create_publisher(Twist, '/cmd_vel', 10)
        self.yaw_pub = self.create_publisher(Float64, '/arm/yaw/command', 10)
        self.lift_pub = self.create_publisher(Float64, '/arm/lift/command', 10)
        self.status_pub = self.create_publisher(String, '/teleop/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_status_time = -1.0
        self.last_status = None
        self.create_subscription(Joy, '/joy', self.on_joy, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, 10)
        self.create_service(Trigger, '/safety/reset_estop', self.reset_estop)
        self.last_ros_time = self.get_clock().now().nanoseconds
        # Wall-time watchdog continues to work when simulated time is paused.
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Release right-stick X and both triggers to enable drive; right-stick Y is unused. Centre left stick to enable arm independently. Circle/B stops. /safety/estop latches; reset with /safety/reset_estop only after release. Diagnostics: /teleop/status')

    def on_estop(self, msg):
        was_latched = self.control.estop_latched
        self.control.set_estop(msg.data)
        if msg.data and not was_latched:
            self.get_logger().error('EXTERNAL E-STOP LATCHED: drive and arm commands forced to zero')

    def reset_estop(self, _request, response):
        response.success = self.control.reset_estop()
        response.message = ('E-stop reset; centre controls to re-arm' if response.success
                            else 'Cannot reset while /safety/estop is asserted')
        return response

    def on_joy(self, msg):
        if not self.control.receive(msg.axes, msg.buttons, time.monotonic()):
            self.get_logger().warning('Invalid /joy layout/value: stopped. Check game_controller_node and axis configuration.', throttle_duration_sec=5)

    def on_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if 'arm_yaw_joint' in positions and 'arm_lift_joint' in positions:
            self.control.initialize_arm(positions['arm_yaw_joint'], positions['arm_lift_joint'])

    def tick(self):
        stamp = self.get_clock().now().nanoseconds
        dt = (stamp - self.last_ros_time) / 1e9
        self.last_ros_time = stamp
        v, w, yaw, lift = self.control.step(time.monotonic(), dt)
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.drive.publish(msg)
        if self.control.arm_ready:
            self.yaw_pub.publish(Float64(data=yaw))
            self.lift_pub.publish(Float64(data=lift))
        now = time.monotonic()
        c = self.control
        live = c.axes is not None and now - c.last_input <= c.s.joy_timeout
        state = (live, c.armed, c.arm_armed, c.arm_ready, c.estop_input,
                 c.estop_latched, c.drive_state)
        if state != self.last_status or now - self.last_status_time > 1.0:
            status = {'input_live': live, 'drive_armed': c.armed,
                      'arm_armed': c.arm_armed, 'arm_joint_states_received': c.arm_ready,
                      'estop_input': c.estop_input, 'estop_latched': c.estop_latched,
                      'drive_state': c.drive_state,
                      'joy_age_seconds': None if c.axes is None else now - c.last_input,
                      'axes': c.axes, 'linear_command': v, 'angular_command': w}
            self.status_pub.publish(String(data=json.dumps(status)))
            if state != self.last_status:
                self.get_logger().info(f'Controller status: input_live={live}, drive_armed={c.armed}, arm_armed={c.arm_armed}, arm_feedback={c.arm_ready}, drive_state={c.drive_state}')
            self.last_status, self.last_status_time = state, now


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.drive.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
