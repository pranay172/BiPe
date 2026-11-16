import numpy as np
from bipe.m13 import handover as H

class Smooth(H.RuntimeController):
    def __init__(self, *a, seed=False, ramp_s=0.0, zero=False, bumpless=False, stop=False, **k):
        self.stop = stop
        self.bumpless = bumpless
        super().__init__(*a, **k)
        self.zero = zero
        import os
        if os.environ.get("GAINS"):
            import json
            from bipe.robustness_control import build_candidate
            from bipe.m13 import policy_control as pc
            from pathlib import Path
            p, gains = a[0], json.loads(json.dumps(a[1]))
            gains.update(json.loads(os.environ["GAINS"]))
            tag = "".join(c for c in os.environ["GAINS"] if c.isalnum() or c in "._")
            lib2 = build_candidate(p, gains, Path(".venv/m8-build/hand_" + tag))
            self.classical = pc.Classical(p, gains, lib2)
        self.seed, self.ramp_s, self.start = seed, ramp_s, None
        self.ramp = None
    def command(self, s, velocity, yaw_rate, qref, qdref, dt):
        if self.handover_s is not None or not self._will_hand(s):
            pass
        return self._cmd(s, velocity, yaw_rate, qref, qdref, dt)
    def _will_hand(self, s): return True
    def _cmd(self, s, velocity, yaw_rate, qref, qdref, dt):
        before = self.handover_s
        last_legs = np.array(self.learned.leg_output[:])
        # run the parent up to handover detection; replicate parent's flow
        if before is None:
            # parent command does everything; intercept classical.command via wrapper
            orig = self.classical.command
            def wrapped(s_, v_, y_, qr_, qdr_, dt_):
                self._on_handover(s_, v_, last_legs)
                qr2, qdr2 = self._ramped(qr_, qdr_)
                if self.bumpless:
                    self._bump(orig, s_, v_, y_, qr2, qdr2, dt_)
                out = orig(s_, v_, y_, qr2, qdr2, dt_)
                self._log(s_, out)
                return out
            self.classical.command = wrapped
            try:
                return super().command(s, velocity, yaw_rate, qref, qdref, dt)
            finally:
                self.classical.command = orig
        qr2, qdr2 = self._ramped(qref, qdref)
        if self.stop: velocity = 0.0; yaw_rate = 0.0
        out = super().command(s, velocity, yaw_rate, qr2, qdr2, dt)
        self._log(s, out)
        return out
    log = []
    def _log(self, s, out):
        st = self.classical.state
        Smooth.log.append([self.time, s["pitch"], s["velocity"], out[1], st.vref, st.integral, st.slope_lean, st.acceleration_lean, st.slope_force, out[0][4], out[0][5]])
    def _bump(self, orig, s, v, y, qr, qdr, dt):
        from bipe.firmware import Balance
        st = self.classical.state
        st.vref = v; st.previous_velocity = v; st.heading = s["yaw"]; st.command_acceleration = 0
        saved = Balance.from_buffer_copy(st)
        prev = self.classical.previous_request.copy(); ff = self.classical.feedforward.copy()
        def ref(I):
            import ctypes
            ctypes.memmove(ctypes.addressof(st), ctypes.addressof(saved), ctypes.sizeof(st))
            st.integral = I
            return orig(s, v, y, qr, qdr, dt)[1]
        r0, r1 = ref(0.0), ref(0.01)
        k = (r1 - r0) / 0.01
        I = (s["pitch"] - r0) / k if abs(k) > 1e-9 else 0.0
        import ctypes
        ctypes.memmove(ctypes.addressof(st), ctypes.addressof(saved), ctypes.sizeof(st))
        st.integral = I
        self.classical.previous_request = prev; self.classical.feedforward = ff
        print("bumpless", self.time, "k", k, "I", I, "r0", r0, "pitch", s["pitch"])
    def _on_handover(self, s, velocity, legs):
        st = self.classical.state
        if self.seed:
            st.vref = s["velocity"]; st.previous_velocity = velocity; st.heading = s["yaw"]
        self.start = (self.time - 0.0, legs.copy())
        self.t0 = self.time
    def _ramped(self, qref, qdref):
        qref, qdref = np.asarray(qref, float), np.asarray(qdref, float)
        if self.ramp_s <= 0 or self.start is None:
            self.ramp = None; return qref, qdref
        u = (self.time - self.t0) / self.ramp_s
        if u >= 1:
            self.ramp = None; return qref, qdref
        w = 0.5 - 0.5 * np.cos(np.pi * u); dw = 0.5 * np.pi * np.sin(np.pi * u) / self.ramp_s
        legs = self.start[1]
        self.ramp = (legs + w * (qref - legs), qdref + dw * (qref - legs))
        return self.ramp
    legmode = ""
    def leg_torques(self, s, qref, qdref, feedforward=None):
        import os
        if self.handover_s is not None and self.ramp is not None:
            qref, qdref = self.ramp
        if self.handover_s is not None and os.environ.get("LEGPD"):
            return self.cfg.joint_kp * (np.asarray(qref) - s["q"]) - self.cfg.joint_kd * s["qd"]
        k = float(os.environ.get("KNEEFF", "0"))
        if self.handover_s is not None and k and feedforward is not None:
            feedforward = np.array(feedforward, float).copy()
            feedforward[1] += k * self.classical.previous_request[4]
            feedforward[3] += k * self.classical.previous_request[5]
        if self.handover_s is not None and os.environ.get("FFREF"):
            from bipe.firmware_control import D4
            out = D4()
            self.classical.lib.bipe_feedforward(D4(*qref), s["pitch"], s["roll"], out)
            feedforward = np.array(out)
        if self.handover_s is not None and os.environ.get("FFCONST"):
            from bipe.firmware_control import D4
            out = D4()
            self.classical.lib.bipe_feedforward(D4(*qref), 0.0, 0.0, out)
            feedforward = np.array(out)
        if self.handover_s is not None and os.environ.get("FFLOCAL"):
            from bipe.firmware_control import D4
            out = D4()
            self.classical.lib.bipe_feedforward(D4(*s["q"]), s["pitch"], s["roll"], out)
            feedforward = np.array(out)
        if self.handover_s is not None and os.environ.get("FF0"):
            feedforward = None
            ff = self.classical.feedforward
            import ctypes as c
            from bipe.firmware_control import D4
            out = D4()
            self.classical.lib.bipe_feedforward(D4(*s["q"]), 0.0, 0.0, out)
            feedforward = np.array(out)
        return super().leg_torques(s, qref, qdref, feedforward)

    def _forward(self, observation, action):
        if self.zero and not (self.fault_s is not None and self.time >= self.fault_s):
            for i in range(6): action[i] = 0.0
            return
        return super()._forward(observation, action)

import os as _os
from bipe import firmware_control as _fc
_Base = _fc.FirmwareController
class KneeFF(_Base):
    """Classical with the wheel reaction torque added to the knee feedforward."""
    lagged = None
    def leg_torques(self, s, qref, qdref, feedforward=None):
        k = float(_os.environ.get("KNEEFF", "0"))
        tau = float(_os.environ.get("KNEETAU", "0"))
        w = np.array(self.previous_request[4:6], float)
        if self.lagged is None: self.lagged = np.zeros(2)
        self.lagged += (1 - np.exp(-0.001 / tau)) * (w - self.lagged) if tau > 0 else (w - self.lagged)
        if feedforward is not None and k:
            feedforward = np.array(feedforward, float).copy()
            feedforward[1] += k * self.lagged[0]
            feedforward[3] += k * self.lagged[1]
        return super().leg_torques(s, qref, qdref, feedforward)
