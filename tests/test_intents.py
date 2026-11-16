import pytest

from bipe.intents import NOMINAL_POSTURE, SIT_LABEL, Intents


def test_drive_clips_to_trained_ranges():
    i = Intents()
    i.drive(3.0, -5.0)
    c = i.step(0.02)
    assert (c.behaviour, c.speed, c.yaw_rate) == ("drive", 1.2, -1.0)
    i.set_posture(90)
    assert i.step(0.02).posture == 65.0


def test_sit_stand_and_handover():
    i = Intents()
    i.drive(0.5)
    i.sit()
    c = i.step(0.02)
    assert (c.behaviour, c.speed, c.posture) == ("sit_stand", 0.0, SIT_LABEL)
    i.stand()
    assert i.step(0.02).posture == NOMINAL_POSTURE
    i.handed_over()
    assert i.step(0.02).behaviour == "drive"


def test_stop_does_not_abort_a_get_up():
    i = Intents()
    i.get_up()
    i.stop()
    assert i.step(0.02).behaviour == "getup"
    i.handed_over()
    i.drive(0.4)
    i.stop()
    assert (i.step(0.02).behaviour, i.step(0.02).speed) == ("drive", 0.0)


def test_move_plays_and_ends():
    i = Intents()
    i.move("crouch_bob")
    postures = [i.step(0.02).posture for _ in range(2000)]
    assert min(postures) == pytest.approx(35.0) and max(postures) == pytest.approx(60.0)
    assert i.move_name is None and postures[-1] == NOMINAL_POSTURE
    with pytest.raises(ValueError):
        i.move("backflip")


def test_drive_and_move_wait_for_sit_and_get_up():
    i = Intents()
    i.sit()
    assert not i.drive(0.5) and not i.move("wiggle")
    assert i.step(0.02).behaviour == "sit_stand" and i.step(0.02).posture == SIT_LABEL
    assert i.stand()
    assert not i.drive(0.5)
    i.handed_over()
    assert i.drive(0.5) and i.step(0.02).speed == 0.5
    i.get_up()
    assert not i.drive(0.3) and not i.stand() and i.step(0.02).behaviour == "getup"
