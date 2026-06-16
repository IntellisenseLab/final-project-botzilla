import math

import pytest

from botzilla_control.navigation import (
    DifferentialOdometry,
    ENCODER_MODULO,
    WHEEL_BASE_M,
    meters_to_ticks,
    ticks_to_meters,
)


def _advance(left_ticks, right_ticks, left_delta, right_delta):
    return (
        (left_ticks + left_delta) % ENCODER_MODULO,
        (right_ticks + right_delta) % ENCODER_MODULO,
    )


def test_encoder_ticks_translate_to_distance():
    ticks = meters_to_ticks(1.0)
    assert ticks_to_meters(ticks) == pytest.approx(1.0, abs=0.001)


def test_square_path_returns_to_origin():
    odom = DifferentialOdometry()
    left_ticks = 0
    right_ticks = 0
    odom.update(left_ticks, right_ticks)

    side_ticks = meters_to_ticks(0.5)
    turn_arc_ticks = meters_to_ticks((math.pi / 2.0) * WHEEL_BASE_M / 2.0)

    for _ in range(4):
        left_ticks, right_ticks = _advance(
            left_ticks,
            right_ticks,
            side_ticks,
            side_ticks,
        )
        odom.update(left_ticks, right_ticks)

        left_ticks, right_ticks = _advance(
            left_ticks,
            right_ticks,
            -turn_arc_ticks,
            turn_arc_ticks,
        )
        odom.update(left_ticks, right_ticks)

    assert odom.pose.x == pytest.approx(0.0, abs=0.03)
    assert odom.pose.y == pytest.approx(0.0, abs=0.03)
    assert math.sin(odom.pose.theta) == pytest.approx(0.0, abs=0.03)
