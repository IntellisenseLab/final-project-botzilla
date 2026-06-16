import pytest

from botzilla_control.navigation import (
    GoToPoseController,
    Pose2D,
    quadrant_targets,
    room_span_from_wall_hits,
    star_search_waypoints,
)


def test_dimensioning_uses_opposite_wall_hits():
    room_length, center_x = room_span_from_wall_hits(1.75, -1.25)
    room_width, center_y = room_span_from_wall_hits(0.90, -1.10)

    assert room_length == pytest.approx(3.0)
    assert center_x == pytest.approx(0.25)
    assert room_width == pytest.approx(2.0)
    assert center_y == pytest.approx(-0.10)


def test_star_search_visits_quadrants_and_returns_to_center():
    targets = quadrant_targets(4.0, 2.0)
    waypoints = star_search_waypoints(4.0, 2.0)

    assert targets == [
        (1.0, 0.5),
        (1.0, -0.5),
        (-1.0, -0.5),
        (-1.0, 0.5),
    ]
    assert waypoints == [
        (1.0, 0.5),
        (0.0, 0.0),
        (1.0, -0.5),
        (0.0, 0.0),
        (-1.0, -0.5),
        (0.0, 0.0),
        (-1.0, 0.5),
        (0.0, 0.0),
    ]


def test_go_to_pose_turns_before_driving_when_target_is_behind():
    controller = GoToPoseController()
    linear, angular, arrived = controller.command(Pose2D(), -1.0, 0.0)

    assert arrived is False
    assert linear == 0.0
    assert abs(angular) > 0.0
