"""M16 phase I: the intent layer, one small vocabulary for everything that commands BiPe.

The follower, the moves, the interactive sim and (at the hardware build) the Pi send
intents; the intent layer turns them into the policy contract's command (speed, yaw rate,
posture label: docs/policy-contract.md) and names the behaviour whose policy should run
(config/behaviours.json). The supervisor still owns safety: a fault or a fall overrides
any intent.

    drive(speed, yaw_rate)   ride at a speed and yaw rate (the drive policy)
    posture(label)           crouch or stand taller, label 30 to 65 degrees
    move(name)               play a move from bipe.m15.moves (crouch_bob, spin_stop, wiggle)
    sit() / stand()          sit on the shell / rise again (the sit_stand policy)
    get_up()                 the get-up policy, from a fall
    stop()                   speed and yaw rate to zero, keep the posture

Drive and move are ignored while a sit, a stand or a get-up runs (they return False):
a sitting robot rises with stand(), a fallen one with get_up(), and the supervisor's
handed_over() returns to drive. stand() acts only from a sit.

``Intents.step(dt)`` is called every policy step and returns a ``Command``. Ranges are
clipped to the trained ones, so an out-of-range request cannot leave the contract. The
firmware twin of this table belongs to the hardware build's command path.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

SPEED = (-0.5, 1.2)
YAW_RATE = (-1.0, 1.0)
POSTURE = (30.0, 65.0)
NOMINAL_POSTURE = 40.0
SIT_LABEL = 100.0
"""The sit request in the posture slot (bipe.m13.rl.mdp.SIT_LABEL)."""


@dataclass(frozen=True)
class Command:
    behaviour: str
    """Which catalogue entry runs: drive, sit_stand or getup."""
    speed: float = 0.0
    yaw_rate: float = 0.0
    posture: float = NOMINAL_POSTURE


def _clip(x, lo_hi):
    return float(np.clip(x, *lo_hi))


class Intents:
    def __init__(self):
        self.speed, self.yaw_rate, self.posture = 0.0, 0.0, NOMINAL_POSTURE
        self.mode = "drive"
        self.move_name, self.move_t, self._move = None, 0.0, None

    # The vocabulary.

    def busy(self):
        """A sit, a stand or a get-up is in progress: drive and move wait for it."""
        return self.mode != "drive"

    def drive(self, speed, yaw_rate=0.0):
        """Ignored during a sit, a stand or a get-up (stand from a sit first)."""
        if self.busy():
            return False
        self._end_move()
        self.speed, self.yaw_rate = _clip(speed, SPEED), _clip(yaw_rate, YAW_RATE)
        return True

    def set_posture(self, label):
        self.posture = _clip(label, POSTURE)

    def stop(self):
        self._end_move()
        self.speed = self.yaw_rate = 0.0  # a get-up or a sit carries on: stop is not a mode

    def sit(self):
        self._end_move()
        self.speed = self.yaw_rate = 0.0
        self.mode = "sit"

    def stand(self):
        """Rise from a sit; ignored in any other mode."""
        if self.mode != "sit":
            return False
        self.mode = "stand"
        return True

    def get_up(self):
        self._end_move()
        self.speed = self.yaw_rate = 0.0
        self.mode = "getup"

    def move(self, name):
        from .m15.moves import MOVES

        if name not in MOVES:
            raise ValueError(f"unknown move {name!r}; known: {sorted(MOVES)}")
        if self.busy():
            return False
        self.move_name, self.move_t, self._move = name, 0.0, MOVES[name]()
        return True

    def handed_over(self):
        """The supervisor's note that a get-up or a stand has handed over to drive."""
        if self.mode in ("getup", "stand"):
            self.mode = "drive"
            self.posture = NOMINAL_POSTURE

    # Each policy step.

    def step(self, dt):
        if self.mode == "getup":
            return Command("getup")
        if self.mode == "sit":
            return Command("sit_stand", posture=SIT_LABEL)
        if self.mode == "stand":
            return Command("sit_stand", posture=NOMINAL_POSTURE)
        if self._move is not None:
            m, t = self._move, self.move_t
            self.move_t += dt
            if t >= m["duration"]:
                self._end_move()
            else:
                k = np.asarray(m["knots"], float)
                speed, yaw = (float(np.interp(t, k[:, 0], k[:, i])) for i in (1, 2))
                posture = self.posture
                if "posture_knots" in m:
                    p = np.asarray(m["posture_knots"], float)
                    posture = float(np.interp(t, p[:, 0], p[:, 1]))
                return Command("drive", speed, yaw, posture)
        return Command("drive", self.speed, self.yaw_rate, self.posture)

    def _end_move(self):
        self.move_name, self._move = None, None
