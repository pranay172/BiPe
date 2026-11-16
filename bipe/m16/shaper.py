"""M16 phase B: the command shaper between the operator (or the follower) and the policy.

A jerk-limited second-order limiter on the speed command and an acceleration limit on
the yaw-rate command, stepped at the policy rate. The speed reference accelerates at most
``acceleration`` and changes its acceleration at most ``jerk`` per second, and it brakes
so as to arrive at the target without overshoot (the rate allowed at a distance e from
the target is sqrt(2 jerk |e|), capped at ``acceleration``). The yaw-rate command
changes by at most ``yaw_acceleration`` per second.

Microduck and most commercial robots filter operator input this way; the M15 follower's
1.1 m/s^2 limit, which fixed following, is the same idea. The C twin goes into the
firmware command path once the setting is chosen.
"""

from __future__ import annotations

import math


class Shaper:
    def __init__(self, acceleration=1.0, jerk=4.0, yaw_acceleration=3.0):
        self.a, self.j, self.ya = float(acceleration), float(jerk), float(yaw_acceleration)
        self.v = self.rate = self.w = 0.0

    def step(self, velocity, yaw_rate, dt):
        """(shaped speed, shaped yaw rate) after one step of dt towards the targets."""
        e = velocity - self.v
        # Rate that still stops on the target, with the rate's own change limited by jerk.
        want = math.copysign(min(self.a, math.sqrt(2 * self.j * abs(e))), e)
        if abs(e) < 1e-9:
            want = 0.0
        self.rate += max(-self.j * dt, min(self.j * dt, want - self.rate))
        step = self.rate * dt
        if (e > 0 and step > e) or (e < 0 and step < e) or e == 0:
            self.v, self.rate = velocity, 0.0 if abs(want) < self.j * dt else self.rate
        else:
            self.v += step
        dw = yaw_rate - self.w
        self.w += max(-self.ya * dt, min(self.ya * dt, dw))
        return self.v, self.w
