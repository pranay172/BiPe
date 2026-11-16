"""Real ground inclination, chassis impulses and dissipative gearbox lost motion.

The rotor coordinates carry the same output-reflected armature as the rigid
model. A gear face engagement projects *velocity* through the full mass matrix,
conserving generalized momentum and dissipating relative kinetic energy.
Only the massless rotor angle is corrected to its face after step overshoot;
no chassis/wheel/leg pose or velocity is prescribed for balance.
"""

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .articulated import generate_xml, reset


def build_disturbed(p, slope_deg=0, backlash_deg=0):
    if not (math.isfinite(slope_deg) and abs(slope_deg) <= 15):
        raise ValueError("Inclination must be finite and within 15 degrees")
    if not (math.isfinite(backlash_deg) and 0 <= backlash_deg <= 2):
        raise ValueError("Backlash must be finite and between zero and two degrees")
    root = ET.fromstring(generate_xml(p))
    # +angle is uphill in +X. R_y(-angle) maps plane +Z to [-sin(a),0,cos(a)].
    a = math.radians(slope_deg)
    root.find("worldbody/geom[@name='ground']").set(
        "quat", f"{math.cos(a / 2)} 0 {-math.sin(a / 2)} 0"
    )
    # Below numerical angular resolution the physical rigid-gear limit is exact.
    active = math.radians(backlash_deg) > 1e-9
    if active:
        for side in ["left", "right"]:
            wheel = root.find(f".//body[@name='{side}_wheel']")
            lower = root.find(f".//body[@name='{side}_lower']")
            wheel.find("joint").set("armature", "0")
            rotor = ET.SubElement(lower, "body", name=side + "_rotor", pos=wheel.get("pos"))
            # MuJoCo requires a positive body inertia. Negligible carrier mass is
            # bookkeeping only; the declared output inertia lives in armature.
            ET.SubElement(
                rotor, "inertial", pos="0 0 0", mass="1e-10", diaginertia="1e-12 1e-12 1e-12"
            )
            ET.SubElement(
                rotor,
                "joint",
                name=side + "_rotor_joint",
                type="hinge",
                axis="0 1 0",
                armature=str(p["motor"]["rotor_inertia_at_output"]),
            )
            root.find(f"actuator/motor[@name='{side}_wheel_joint_drive']").set(
                "joint", side + "_rotor_joint"
            )
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    return m, mujoco.MjData(m), active


def reset_on_plane(m, d, p, slope_deg, angle_deg=40, offset_deg=0):
    reset(m, d, p, angle_deg, offset_deg)
    if slope_deg:
        a = math.radians(slope_deg)
        axle = np.mean([d.site(s + "_axle").xpos for s in ["left", "right"]], axis=0)
        n = np.array([-math.sin(a), 0, math.cos(a)])
        d.qpos[2] += (p["geometry"]["wheel_radius"] - 0.0001 - n @ axle) / n[2]
        mujoco.mj_forward(m, d)


class BacklashGear:
    def __init__(self, m, gap_deg):
        self.half = math.radians(gap_deg) / 2
        self.wq = np.array([m.joint(s + "_wheel_joint").qposadr[0] for s in ["left", "right"]])
        self.rq = np.array([m.joint(s + "_rotor_joint").qposadr[0] for s in ["left", "right"]])
        self.wv = np.array([m.joint(s + "_wheel_joint").dofadr[0] for s in ["left", "right"]])
        self.rv = np.array([m.joint(s + "_rotor_joint").dofadr[0] for s in ["left", "right"]])
        self.J = np.zeros((2, m.nv))
        for i in range(2):
            self.J[i, self.rv[i]], self.J[i, self.wv[i]] = 1, -1
        self.inverse_rows = np.zeros_like(self.J)
        self.engagements = 0
        self.dissipated_J = 0.0
        self.peak_impulse_Nms = 0.0
        self.max_overshoot_rad = 0.0
        self.side = np.zeros(2)
        self.inertia = m.dof_armature[self.rv].copy()
        self.substeps = 0
        self.constant_data = mujoco.MjData(m)

    def refresh_inertia_constants(self, m):
        # Changing armature also changes MuJoCo's reference inverse weights
        # used by friction/contact constraints. Updating qM alone is insufficient.
        mujoco.mj_setConst(m, self.constant_data)

    def step(self, m, d, torque):
        """Resolve the free-flight face crossing within the physics interval.

        Near-zero gaps cannot be charged an artificial whole 1 ms delay on
        every reversal. A semi-implicit acceleration predictor finds crossing
        times; the remaining interval uses the engaged rigid gear dynamics.
        """
        original = float(m.opt.timestep)
        remaining = original
        try:
            for _ in range(32):
                self.prepare(m, d, torque)
                h = remaining
                if not self.side.all():
                    mujoco.mj_forward(m, d)
                    gap = d.qpos[self.rq] - d.qpos[self.wq]
                    speed = self.J @ d.qvel
                    acceleration = self.J @ d.qacc
                    for i in np.flatnonzero(self.side == 0):
                        for face in [-self.half, self.half]:
                            a, b, c = acceleration[i], speed[i], gap[i] - face
                            if abs(a) < 1e-12:
                                roots = [-c / b] if abs(b) > 1e-12 else []
                            else:
                                discriminant = b * b - 4 * a * c
                                roots = (
                                    [
                                        (-b - math.sqrt(discriminant)) / (2 * a),
                                        (-b + math.sqrt(discriminant)) / (2 * a),
                                    ]
                                    if discriminant >= 0
                                    else []
                                )
                            for root in roots:
                                if 1e-10 < root < h:
                                    h = root
                h = max(min(h, remaining), min(1e-7, remaining))
                m.opt.timestep = h
                mujoco.mj_step(m, d)
                self.engage(m, d)
                self.substeps += 1
                remaining -= h
                if remaining < 1e-12:
                    break
            if remaining >= 1e-12:
                m.opt.timestep = remaining
                self.prepare(m, d, torque)
                mujoco.mj_step(m, d)
                self.engage(m, d)
                self.substeps += 1
        finally:
            m.opt.timestep = original

    def prepare(self, m, d, torque):
        """Eliminate an engaged rotor coordinate into wheel armature.

        This is the exact rigid-gear inertia within an engaged face, rather
        than repeatedly impacting the rotor once per physics step. Release
        whenever the required face reaction becomes tensile.
        """
        previous_armature = m.dof_armature.copy()
        for i in range(2):
            locked = self.side[i] != 0
            m.dof_armature[self.wv[i]] = self.inertia[i] if locked else 0
            m.dof_armature[self.rv[i]] = 0 if locked else self.inertia[i]
            d.ctrl[4 + i] = 0 if locked else torque[i]
            # Actuator routing remains rotor-only; engaged input is a physical
            # joint generalized force, not an unconstrained large motor pulse.
            d.qfrc_applied[self.wv[i]] = torque[i] if locked else 0
        if not np.array_equal(previous_armature, m.dof_armature):
            self.refresh_inertia_constants(m)
        if self.side.any():
            mujoco.mj_forward(m, d)
            for i in range(2):
                reaction = torque[i] - self.inertia[i] * d.qacc[self.wv[i]]
                if self.side[i] and self.side[i] * reaction < 0:
                    self.side[i] = 0
                    m.dof_armature[self.wv[i]] = 0
                    m.dof_armature[self.rv[i]] = self.inertia[i]
                    d.ctrl[4 + i] = torque[i]
                    d.qfrc_applied[self.wv[i]] = 0
            if not np.array_equal(previous_armature, m.dof_armature):
                self.refresh_inertia_constants(m)

    def engage(self, m, d):
        for i in np.flatnonzero(self.side):
            d.qpos[self.rq[i]] = d.qpos[self.wq[i]] + self.side[i] * self.half
            d.qvel[self.rv[i]] = d.qvel[self.wv[i]]
        gap = d.qpos[self.rq] - d.qpos[self.wq]
        relative = self.J @ d.qvel
        sides = np.sign(gap)
        active = (abs(gap) >= self.half) & (sides * relative > 0)
        if not active.any():
            return
        # Refresh mass factorization at the new configuration after integration.
        mujoco.mj_fwdPosition(m, d)
        mujoco.mj_solveM(m, d, self.inverse_rows, self.J)
        ids = np.flatnonzero(active)
        W = self.J @ self.inverse_rows.T
        impulses = np.linalg.solve(W[np.ix_(ids, ids)], relative[ids])
        # Coupled unilateral solve: do not retain a tensile face impulse.
        for _ in range(2):
            valid = impulses * sides[ids] >= 0
            if valid.all():
                break
            ids = ids[valid]
            if not len(ids):
                return
            impulses = np.linalg.solve(W[np.ix_(ids, ids)], relative[ids])
        d.qvel[:] -= self.inverse_rows[ids].T @ impulses
        self.dissipated_J += float(0.5 * impulses @ relative[ids])
        self.peak_impulse_Nms = max(self.peak_impulse_Nms, float(abs(impulses).max()))
        self.engagements += len(ids)
        self.max_overshoot_rad = max(
            self.max_overshoot_rad, float(np.maximum(abs(gap) - self.half, 0).max())
        )
        d.qpos[self.rq[ids]] = d.qpos[self.wq[ids]] + sides[ids] * self.half
        self.side[ids] = sides[ids]

    def metrics(self):
        return {
            "face_impulse_steps": self.engagements,
            "engagement_energy_dissipated_J": self.dissipated_J,
            "peak_gear_impulse_Nms": self.peak_impulse_Nms,
            "max_step_gap_overshoot_deg": math.degrees(self.max_overshoot_rad),
            "physics_substeps": self.substeps,
        }
