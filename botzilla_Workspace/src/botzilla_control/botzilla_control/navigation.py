import math
from dataclasses import dataclass


WHEEL_RADIUS_M = 0.035
WHEEL_BASE_M = 0.230
ENCODER_TICKS_PER_REV = 2578.33
ENCODER_MODULO = 65536
METERS_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS_M) / ENCODER_TICKS_PER_REV


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def encoder_delta_ticks(current, previous):
    """Return signed delta for Kobuki's wrapping 16-bit wheel encoders."""
    delta = (int(current) - int(previous)) % ENCODER_MODULO
    if delta > ENCODER_MODULO / 2:
        delta -= ENCODER_MODULO
    return delta


def ticks_to_meters(ticks):
    return ticks * METERS_PER_TICK


def meters_to_ticks(distance_m):
    return int(round(distance_m / METERS_PER_TICK))


def room_span_from_wall_hits(first_hit, second_hit):
    length = abs(first_hit - second_hit)
    center = (first_hit + second_hit) / 2.0
    return length, center


def quadrant_targets(room_length, room_width):
    return [
        (room_length / 4.0, room_width / 4.0),
        (room_length / 4.0, -room_width / 4.0),
        (-room_length / 4.0, -room_width / 4.0),
        (-room_length / 4.0, room_width / 4.0),
    ]


def star_search_waypoints(room_length, room_width):
    waypoints = []
    for target in quadrant_targets(room_length, room_width):
        waypoints.append(target)
        waypoints.append((0.0, 0.0))
    return waypoints


class DifferentialOdometry:
    def __init__(self, wheel_base_m=WHEEL_BASE_M):
        self.wheel_base_m = wheel_base_m
        self.pose = Pose2D()
        self.total_theta = 0.0
        self._last_left_ticks = None
        self._last_right_ticks = None

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.pose = Pose2D(x, y, normalize_angle(theta))
        self.total_theta = theta

    def update(self, left_ticks, right_ticks):
        if self._last_left_ticks is None or self._last_right_ticks is None:
            self._last_left_ticks = int(left_ticks)
            self._last_right_ticks = int(right_ticks)
            return self.pose

        left_delta = encoder_delta_ticks(left_ticks, self._last_left_ticks)
        right_delta = encoder_delta_ticks(right_ticks, self._last_right_ticks)
        self._last_left_ticks = int(left_ticks)
        self._last_right_ticks = int(right_ticks)

        left_m = ticks_to_meters(left_delta)
        right_m = ticks_to_meters(right_delta)
        center_delta = (left_m + right_m) / 2.0
        theta_delta = (right_m - left_m) / self.wheel_base_m
        mid_theta = self.pose.theta + theta_delta / 2.0

        self.pose.x += center_delta * math.cos(mid_theta)
        self.pose.y += center_delta * math.sin(mid_theta)
        self.total_theta += theta_delta
        self.pose.theta = normalize_angle(self.pose.theta + theta_delta)
        return self.pose


class GoToPoseController:
    def __init__(
        self,
        max_linear=0.15,
        max_angular=0.45,
        linear_kp=0.8,
        angular_kp=1.8,
        distance_tolerance=0.06,
        heading_tolerance=0.16,
    ):
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.linear_kp = linear_kp
        self.angular_kp = angular_kp
        self.distance_tolerance = distance_tolerance
        self.heading_tolerance = heading_tolerance

    def command(self, pose, target_x, target_y):
        dx = target_x - pose.x
        dy = target_y - pose.y
        distance = math.hypot(dx, dy)
        if distance <= self.distance_tolerance:
            return 0.0, 0.0, True

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - pose.theta)
        angular = _clamp(
            self.angular_kp * heading_error,
            -self.max_angular,
            self.max_angular,
        )

        if abs(heading_error) > self.heading_tolerance:
            return 0.0, angular, False

        linear = min(self.max_linear, max(0.05, self.linear_kp * distance))
        return linear, angular, False


def _clamp(value, low, high):
    return max(low, min(high, value))
