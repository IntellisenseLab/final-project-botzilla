import math

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, UInt8

from .navigation import (
    DifferentialOdometry,
    GoToPoseController,
    normalize_angle,
    quadrant_targets,
    room_span_from_wall_hits,
)


# --- TUNABLE PARAMETERS ---
ALIGNMENT_THRESHOLD = 0.03
CAPTURE_TIME_S = 2.5
CAPTURE_SPEED = 0.12
KP_ANGULAR = 1.2
APPROACH_SPEED = 0.15
CUBE_LOST_TIMEOUT_S = 1.5

DIMENSION_SPEED = 0.10
SEARCH_ANGULAR_SPEED = 0.30
TURN_ANGULAR_SPEED = 0.40
TURN_KP = 1.4
TURN_TOLERANCE_RAD = 0.08
SPIN_SEARCH_RAD = 2.0 * math.pi - 0.12
BUMPER_IGNORE_S = 0.5
VISION_ODOM_TIMEOUT_S = 3.0


class CubeCollector(Node):
    def __init__(self):
        super().__init__('cube_collector')

        self.state = 'IDLE'
        self.target_cube = None
        self._phase_timer = None
        self._cube_last_seen = self.get_clock().now()
        self._cube_lost_time = None
        self._blind_spot_frames = 0
        self._vision_ready = False

        self.odom = DifferentialOdometry()
        self.navigator = GoToPoseController()
        self._pose_initialized = False
        self._bumper_mask = 0
        self._bumper_clear = True

        self.room_length = None
        self.room_width = None
        self._length_wall_a = 0.0
        self._width_wall_a = 0.0
        self._nav_target = (0.0, 0.0)
        self._turn_target = 0.0
        self._quadrant_targets = []
        self._quadrant_index = 0
        self._spin_start_theta = 0.0
        self._spin_after = 'RETURN_CENTER'

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Point, 'detected_cube', self.cube_callback, 10)
        self.create_subscription(Image, '/perception/yolo_image',
                                 self.vision_heartbeat_cb, 10)
        self.create_subscription(Int32MultiArray, '/sensors/encoders',
                                 self.encoder_callback, 10)
        self.create_subscription(UInt8, '/events/bumper',
                                 self.bumper_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            'Cube Collector started. Waiting for vision and encoder telemetry...'
        )

    def vision_heartbeat_cb(self, _msg):
        if not self._vision_ready:
            self.get_logger().info('Vision pipeline detected.')
            self._vision_ready = True
            if self.state == 'IDLE':
                self._transition('WAITING_FOR_ODOM', 'Preparing center-out search.')

    def encoder_callback(self, msg):
        if len(msg.data) < 2:
            return
        self.odom.update(msg.data[0], msg.data[1])
        self._pose_initialized = True

    def bumper_callback(self, msg):
        self._bumper_mask = int(msg.data)
        if self._bumper_mask == 0:
            self._bumper_clear = True
            return

        if self.state == 'NAV_TO_QUADRANT':
            self._start_spin_search(
                'Bumper hit while traveling to quadrant. Searching here.',
                after_spin='RETURN_CENTER',
            )

    def cube_callback(self, msg):
        self.target_cube = msg
        self._cube_last_seen = self.get_clock().now()
        self._vision_ready = True
        if self.state not in ('TARGETING', 'APPROACHING', 'CAPTURING', 'DONE'):
            self.get_logger().info('Cube detected! Transitioning to TARGETING.')
            self._transition('TARGETING', 'Cube visible.')

    def control_loop(self):
        msg = Twist()
        now = self.get_clock().now()

        if self.state == 'IDLE':
            pass

        elif self.state == 'WAITING_FOR_ODOM':
            if self._pose_initialized:
                self._start_dimensioning()
            elif self._elapsed_since(now) > VISION_ODOM_TIMEOUT_S:
                self._transition(
                    'SEARCHING',
                    'No encoder telemetry received; falling back to spin search.',
                )

        elif self.state == 'DIM_LENGTH_FORWARD':
            self._drive_until_bumper(
                msg,
                on_hit=lambda: self._finish_length_wall_a(),
            )

        elif self.state == 'DIM_LENGTH_TURN_BACK':
            self._turn_to_heading(msg, self._turn_target, 'DIM_LENGTH_BACKWARD')

        elif self.state == 'DIM_LENGTH_BACKWARD':
            self._drive_until_bumper(
                msg,
                on_hit=lambda: self._finish_length_wall_b(),
            )

        elif self.state == 'DIM_LENGTH_RETURN_CENTER':
            if self._go_to_target(msg):
                self.odom.reset(0.0, 0.0, self.odom.pose.theta)
                self._start_turn(math.pi / 2.0, 'DIM_WIDTH_TURN_FORWARD')

        elif self.state == 'DIM_WIDTH_TURN_FORWARD':
            self._turn_to_heading(msg, self._turn_target, 'DIM_WIDTH_FORWARD')

        elif self.state == 'DIM_WIDTH_FORWARD':
            self._drive_until_bumper(
                msg,
                on_hit=lambda: self._finish_width_wall_a(),
            )

        elif self.state == 'DIM_WIDTH_TURN_BACK':
            self._turn_to_heading(msg, self._turn_target, 'DIM_WIDTH_BACKWARD')

        elif self.state == 'DIM_WIDTH_BACKWARD':
            self._drive_until_bumper(
                msg,
                on_hit=lambda: self._finish_width_wall_b(),
            )

        elif self.state == 'DIM_WIDTH_RETURN_CENTER':
            if self._go_to_target(msg):
                self.odom.reset(0.0, 0.0, self.odom.pose.theta)
                self._start_turn(0.0, 'DIM_FINAL_TURN')

        elif self.state == 'DIM_FINAL_TURN':
            self._turn_to_heading(msg, self._turn_target, 'START_QUADRANTS')

        elif self.state == 'START_QUADRANTS':
            self._start_quadrant_search()

        elif self.state == 'NAV_TO_QUADRANT':
            if self._go_to_target(msg):
                self._start_spin_search('Quadrant center reached.')

        elif self.state == 'SPIN_SEARCH':
            msg.angular.z = SEARCH_ANGULAR_SPEED
            if abs(self.odom.total_theta - self._spin_start_theta) >= SPIN_SEARCH_RAD:
                self._finish_spin_search()

        elif self.state == 'RETURN_CENTER':
            if self._go_to_target(msg):
                self._quadrant_index += 1
                self._start_next_quadrant()

        elif self.state == 'SEARCHING':
            msg.angular.z = SEARCH_ANGULAR_SPEED

        elif self.state == 'TARGETING':
            if self._cube_timed_out(now):
                self._start_spin_search('Cube lost during targeting.')
            elif self.target_cube is not None:
                error_x = self.target_cube.x
                if abs(error_x) > ALIGNMENT_THRESHOLD:
                    msg.angular.z = -KP_ANGULAR * error_x
                else:
                    self._transition('APPROACHING', 'Aligned. Moving in.')

        elif self.state == 'APPROACHING':
            if self._cube_timed_out(now):
                self._start_spin_search('Cube lost during approach.')
            elif self.target_cube is not None:
                msg.angular.z = -KP_ANGULAR * 0.5 * self.target_cube.x
                if self.target_cube.z == 0.0:
                    self._blind_spot_frames += 1
                else:
                    self._blind_spot_frames = 0

                if self._blind_spot_frames >= 3:
                    self._transition('CAPTURING', 'Confirmed blind spot. Final push.')
                else:
                    msg.linear.x = APPROACH_SPEED

        elif self.state == 'CAPTURING':
            self._run_capture(msg, now)

        elif self.state == 'DONE':
            pass

        self.cmd_pub.publish(msg)

    def _start_dimensioning(self):
        self.odom.reset(0.0, 0.0, 0.0)
        self._transition('DIM_LENGTH_FORWARD', 'Measuring room length.')

    def _finish_length_wall_a(self):
        self._length_wall_a = self.odom.pose.x
        self._start_turn(math.pi, 'DIM_LENGTH_TURN_BACK')

    def _finish_length_wall_b(self):
        self.room_length, center_x = room_span_from_wall_hits(
            self._length_wall_a,
            self.odom.pose.x,
        )
        self._nav_target = (center_x, 0.0)
        self.get_logger().info(f'Room length estimated: {self.room_length:.2f} m')
        self._transition('DIM_LENGTH_RETURN_CENTER', 'Returning to length center.')

    def _finish_width_wall_a(self):
        self._width_wall_a = self.odom.pose.y
        self._start_turn(-math.pi / 2.0, 'DIM_WIDTH_TURN_BACK')

    def _finish_width_wall_b(self):
        self.room_width, center_y = room_span_from_wall_hits(
            self._width_wall_a,
            self.odom.pose.y,
        )
        self._nav_target = (0.0, center_y)
        self.get_logger().info(f'Room width estimated: {self.room_width:.2f} m')
        self._transition('DIM_WIDTH_RETURN_CENTER', 'Returning to room center.')

    def _start_quadrant_search(self):
        if self.room_length is None or self.room_width is None:
            self._transition('SEARCHING', 'Room dimensions unavailable.')
            return
        self._quadrant_targets = quadrant_targets(self.room_length, self.room_width)
        self._quadrant_index = 0
        self._start_next_quadrant()

    def _start_next_quadrant(self):
        if not self._quadrant_targets:
            self._transition('SEARCHING', 'No quadrant targets configured.')
            return
        if self._quadrant_index >= len(self._quadrant_targets):
            self._quadrant_index = 0
        self._nav_target = self._quadrant_targets[self._quadrant_index]
        self._transition(
            'NAV_TO_QUADRANT',
            f'Navigating to quadrant {self._quadrant_index + 1}.',
        )

    def _finish_spin_search(self):
        if self._spin_after == 'RETURN_CENTER':
            self._nav_target = (0.0, 0.0)
            self._transition('RETURN_CENTER', 'Search complete. Resetting at center.')
        else:
            self._start_next_quadrant()

    def _start_spin_search(self, reason, after_spin='RETURN_CENTER'):
        if not self._pose_initialized:
            self._transition('SEARCHING', f'{reason} Encoder telemetry unavailable.')
            return
        self._spin_after = after_spin
        self._spin_start_theta = self.odom.total_theta
        self._transition('SPIN_SEARCH', reason)

    def _drive_until_bumper(self, msg, on_hit):
        now = self.get_clock().now()
        if self._bumper_hit_ready(now):
            on_hit()
            return
        msg.linear.x = DIMENSION_SPEED

    def _bumper_hit_ready(self, now):
        if self._elapsed_since(now) < BUMPER_IGNORE_S:
            return False
        if self._bumper_mask == 0:
            self._bumper_clear = True
            return False
        return self._bumper_clear

    def _start_turn(self, target_heading, state):
        self._turn_target = normalize_angle(target_heading)
        self._transition(state, f'Turning to {self._turn_target:.2f} rad.')

    def _turn_to_heading(self, msg, target_heading, next_state):
        error = normalize_angle(target_heading - self.odom.pose.theta)
        if abs(error) <= TURN_TOLERANCE_RAD:
            self._transition(next_state, 'Turn complete.')
            return

        angular = max(-TURN_ANGULAR_SPEED, min(TURN_ANGULAR_SPEED, TURN_KP * error))
        if abs(angular) < 0.18:
            angular = math.copysign(0.18, angular)
        msg.angular.z = angular

    def _go_to_target(self, msg):
        linear, angular, arrived = self.navigator.command(
            self.odom.pose,
            self._nav_target[0],
            self._nav_target[1],
        )
        if arrived:
            return True
        msg.linear.x = linear
        msg.angular.z = angular
        return False

    def _run_capture(self, msg, now):
        cube_timeout_s = 0.4
        extra_push_s = max(0.0, CAPTURE_TIME_S - cube_timeout_s)

        if (now - self._cube_last_seen).nanoseconds / 1e9 < cube_timeout_s:
            self._cube_lost_time = None
            msg.linear.x = CAPTURE_SPEED
            return

        if self._cube_lost_time is None:
            self.get_logger().info('Cube lost from view. Performing final push...')
            self._cube_lost_time = now

        elapsed_since_lost = (now - self._cube_lost_time).nanoseconds / 1e9
        if elapsed_since_lost < extra_push_s:
            msg.linear.x = CAPTURE_SPEED
        else:
            self._transition('DONE', 'Capture complete.')

    def _transition(self, new_state, reason=''):
        self.get_logger().info(f'[{self.state}] -> [{new_state}] | {reason}')
        self.state = new_state
        self._phase_timer = self.get_clock().now()
        self._bumper_clear = self._bumper_mask == 0
        if new_state != 'APPROACHING':
            self._blind_spot_frames = 0

    def _elapsed_since(self, now):
        if self._phase_timer is None:
            return 0.0
        return (now - self._phase_timer).nanoseconds * 1e-9

    def _cube_timed_out(self, now):
        return (now - self._cube_last_seen).nanoseconds * 1e-9 > CUBE_LOST_TIMEOUT_S


def main(args=None):
    rclpy.init(args=args)
    node = CubeCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals() and rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
