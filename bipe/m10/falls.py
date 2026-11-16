"""M10 phase D: passive tips on the serial-leg plant and the structural screen.

Experiment module, not hardware certification. The free base, CAD rigid
bodies, finite belts with lash, bearing friction and the M10 material
contact laws are those of bipe.m10.plant. Drives are unpowered (zero
commanded current). No force, pose or speed is clipped; a factor below the
requirement is a failed path.

Structural loads come from the solved joint wrenches (MuJoCo cfrc_int after
the constraint solve) plus the belt radial pulls, which torque-only belt
rows omit. Member data are config m10.structure and the B3 CAD sections and
stations (artifacts/m10/cad/sections.json).

    python -m bipe.m10.falls --output artifacts/m10/falls
"""

import argparse
import json
import math
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from ..m9_impact import (
    allowable,
    fastener_demands,
    hollow_section,
    mechanical_energy,
    paired_reactions,
)
from ..parameters import ROOT
from .plant import SIDES, Contacts, MechanicalDrives, build, parameters, reset

ENERGY_EVENT_J = 1e-3
# Fall step: the forward-rest and sideways-0.3 worst factors at 0.125 ms are within 6 %
# of 0.0625 ms; the 0.25 ms plant step under-resolves the lateral tire strike.
FALL_DT_S = 1.25e-4


class CoulombContacts:
    """Fall contacts: the M10 material normal law on frictionless solver rows plus
    explicit Coulomb friction.

    MuJoCo's convex contact relaxation separates a sliding contact at a normal
    speed of about mu times the slip speed; with the plant's hard friction rows a
    sliding PA12 strike reversed -0.77 m/s into +2.96 m/s (+0.20 J). Here the solver
    rows carry only the material normal law (condim 1) and friction is a body
    wrench: Coulomb |f_t| <= mu f_n, where f_n is the contact's material law
    (elastic plus damping at the current penetration rate), limited to the impulse
    that stops the slip within one step (the stable discrete form of stick).
    Rolling and torsional friction are neglected.
    """

    def __init__(self, m, p):
        m.geom_condim[:] = 1
        self.normal = Contacts(p)
        self.normal.configure(m)
        self.mu = m.geom_friction[:, 0].copy()
        self.friction = []

    def configure(self, m):
        self.normal.configure(m)

    def __call__(self, m, d):
        self.normal(m, d)
        d.xfrc_applied[:] = 0
        self.friction = []
        if not d.ncon:
            return
        dt = m.opt.timestep
        pairs = {}
        for i in range(d.ncon):
            key = tuple(sorted(d.contact[i].geom))
            pairs[key] = pairs.get(key, 0) + 1
        ja, jb = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        for i in range(d.ncon):
            c = d.contact[i]
            ga, gb = (int(g) for g in c.geom)
            ba, bb = int(m.geom_bodyid[ga]), int(m.geom_bodyid[gb])
            frame = c.frame.reshape(3, 3)
            mujoco.mj_jac(m, d, ja, None, c.pos, ba)
            mujoco.mj_jac(m, d, jb, None, c.pos, bb)
            rows = frame @ (jb - ja)
            inverse = np.zeros_like(rows)
            mujoco.mj_solveM(m, d, inverse, rows)
            mobility = rows @ inverse.T
            velocity = rows @ d.qvel
            depth = max(0.0, -float(c.dist))
            stiffness, damping = -c.solref[0], -c.solref[1]
            if stiffness <= 0 or c.efc_address < 0:
                continue
            # The material law as the solver applies it (Contacts.solver_mobility).
            row = d.efc_diagA[c.efc_address]
            normal = max(0.0, (stiffness * depth - damping * velocity[0]) / row)
            slip = velocity[1:]
            stop = np.linalg.solve(mobility[1:, 1:], slip) / dt / pairs[tuple(sorted((ga, gb)))]
            limit = max(self.mu[ga], self.mu[gb]) * normal
            size = float(np.linalg.norm(stop))
            tangential = -stop if size <= limit else -stop * limit / size
            force = frame[1:].T @ tangential
            for body, sign in ((bb, 1.0), (ba, -1.0)):
                if body:
                    d.xfrc_applied[body, :3] += sign * force
                    d.xfrc_applied[body, 3:] += np.cross(c.pos - d.xipos[body], sign * force)
            self.friction.append((ga, gb, c.pos.copy(), force))


def parent_wrench(m, d, body):
    """Parent-to-body wrench (world axes), moment about the body origin."""
    w = d.cfrc_int[body]
    force = w[3:].copy()
    moment = w[:3] - np.cross(d.xpos[body] - d.subtree_com[m.body_rootid[body]], force)
    return force, moment


def geom_contact_wrench(m, d, geoms, point, friction=()):
    """Contact wrench (world axes) on the given geoms, moment about point: solved
    normal rows plus the explicit friction forces."""
    force, moment = np.zeros(3), np.zeros(3)
    for ga, gb, pos, f in friction:
        for g, sign in ((gb, 1.0), (ga, -1.0)):
            if g in geoms:
                force += sign * f
                moment += np.cross(pos - point, sign * f)
    for i in range(d.ncon):
        c = d.contact[i]
        for position, g in enumerate(c.geom):
            if g not in geoms:
                continue
            local = np.zeros(6)
            mujoco.mj_contactForce(m, d, i, local)
            frame = c.frame.reshape(3, 3)
            sign = 1 if position == 1 else -1
            f = sign * frame.T @ local[:3]
            force += f
            moment += np.cross(c.pos - point, f) + sign * frame.T @ local[3:]
    return force, moment


def limit_torque(m, d, joint):
    """Generalised joint-limit (end-stop) torque on a hinge from the solved constraints."""
    dof = m.joint(joint).dofadr[0]
    jid = m.joint(joint).id
    rows = [
        i for i in range(d.nefc)
        if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT and d.efc_id[i] == jid
    ]
    if not rows:
        return 0.0
    if mujoco.mj_isSparse(m):
        jac = np.zeros((d.nefc, m.nv))
        mujoco.mju_sparse2dense(jac, d.efc_J, d.efc_J_rownnz, d.efc_J_rowadr, d.efc_J_colind)
    else:
        jac = d.efc_J.reshape(d.nefc, m.nv)
    return float(sum(d.efc_force[i] * jac[i, dof] for i in rows))


def shift(force, moment, frm, to):
    """Moment of a wrench about point `to`, given its moment about `frm`."""
    return moment + np.cross(frm - to, force)


def von_mises(sigma, tau):
    return math.sqrt(sigma * sigma + 3 * tau * tau)


def with_pad(react, force, y_pad, ya, yb):
    """Bearing pair reactions with an end-stop pad force added at y_pad.

    The plant's joint limit is a pure torque; the physical pad couples it with a
    force |M|/r through the bearings. Its direction is not resolved, so its
    lever shares add to the radial magnitudes (conservative)."""
    span = yb - ya
    share = [abs(force * (yb - y_pad) / span), abs(force * (y_pad - ya) / span)]
    radial = [r + q for r, q in zip(react["radial_N"], share)]
    out = dict(react, radial_N=radial)
    out["equivalent_static_N"] = [max(r, 0.6 * r + 0.5 * react["axial_N"]) for r in radial]
    return out


def merge(base, patch):
    """Deep merge of a config patch (dicts merge, everything else replaces)."""
    out = dict(base)
    for k, v in patch.items():
        out[k] = merge(base[k], v) if isinstance(v, dict) and isinstance(base.get(k), dict) else v
    return out


def candidate_config(shared, name):
    """Shared config with a remedy candidate (m10.structure.candidates) merged into m10."""
    if not name:
        return shared
    patch = shared["m10"]["structure"]["candidates"][name]["patch"]
    return dict(shared, m10=merge(shared["m10"], patch))


class Structure:
    """Per-sample factors of every M10 structural path (config m10.structure).

    Link sections take the loads on the segment distal of each station (the
    knee or hub reaction plus the solved contacts located beyond the station);
    segment inertia is neglected. Shafts are checked at their bearing seats
    from the reactions and belt pulls on the inboard span.
    """

    def __init__(self, shared, p, m):
        self.s = shared["m10"]["structure"]
        self.p = p
        self.materials = json.loads((ROOT / self.s["materials_source"]).read_text())
        self.sec = json.loads((ROOT / "artifacts/m10/cad/sections.json").read_text())
        st = {k: v[0] for k, v in self.sec["stations"].items()}
        self.y = {k: (v["y_m"][0] + v["y_m"][1]) / 2 for k, v in st.items()}
        self.shin_face = st["Shin"]["y_m"][1]
        self.allow = {k: allowable(self.materials, k) for k in ("al7075", "pa12")}
        printed = self.s["printed"]
        self.allow_shin = allowable(self.materials, printed.get("shin_material", "pa12"))
        self.allow_bracket = allowable(self.materials, printed.get("hip_bracket_material", "pa12"))
        pads = p["physics"]["end_stop_pads"]["joints"]
        self.pads = {k: (v["radius_m"], v["y_m"]) for k, v in pads.items()}
        g = p["geometry"]
        self.l1, self.l2 = g["thigh_length"], g["shin_length"]
        self.wheel_y = g["wheel_track"] / 2
        motor = np.array([g["motor_row_x"], 0.0, g["motor_row_z"]])
        self.to_motor = motor / np.linalg.norm(motor)
        b = self.s["belts"]
        self.r80 = b["hip"]["large_pulley_teeth"] * b["hip"]["pitch"] / (2 * math.pi)
        self.r40 = b["knee"]["pulley_teeth"] * b["knee"]["pitch"] / (2 * math.pi)
        self.hip_ratio = b["hip"]["large_pulley_teeth"] / b["hip"]["small_pulley_teeth"]
        self.t0_hip = b["hip"]["pretension_per_span_N"]
        self.t0_knee = b["knee"]["pretension_per_span_N_assumption"]
        mb = self.materials["belt"]
        self.hip_rpm = np.array(mb["rpm"], dtype=float)
        self.hip_rating = np.array(
            [v * mb["pound_inch_Nm"] * b["hip"]["length_factor"] for v in mb["hip_input_rating_lb_in"]]
        )
        kr = b["knee"]["corrected_rating_Nm"]
        self.knee_rpm, self.knee_rating = np.array([10.0, 100.0]), np.array([kr["10"], kr["100"]])
        self.c0 = {n: self.s["bearing_catalog"][k]["C0_N"] for n, k in self.s["bearings"].items()}
        self.seat = self.s["bearing_catalog"][self.s["bearings"]["HipPodBearing"]]
        self.ids = {
            n: m.body(n).id
            for side in SIDES
            for n in (f"{side}_thigh", f"{side}_shin", f"{side}_wheel", f"{side}_kneedrive")
        }
        self.chassis = m.body("chassis").id
        geom = {m.geom(i).name: i for i in range(m.ngeom)}
        self.link_geoms = {
            (side, "thigh"): {geom[f"{side}_{n}"] for n in ("Thigh", "ThighLid", "HipCap")}
            for side in SIDES
        } | {(side, "shin"): {geom[f"{side}_Shin"]} for side in SIDES}
        head = self.s["head"]["geoms"]
        self.head_geoms = {geom[n] for n in head}
        parts = json.loads((ROOT / "artifacts/m10/cad/mass.json").read_text())
        by_part, allowances = parts["mass_by_part_kg"], parts["allowances_kg"]
        head_parts = [
            "HeadFront", "HeadRear", "Ear", "EyeLens", "EyeLEDBoard", "CameraBoard", "CameraLens",
            "CameraLensBlock", "CameraRing", "CameraBezel", "HeadScrew",
        ]
        self.head_mass = sum(by_part.get(k, 0.0) for k in head_parts) + allowances["head_pan_reserve"]
        # Head centre of mass in chassis axes: shell geoms weighted by their part masses.
        weights = {"HeadFront": by_part["HeadFront"], "HeadRear": by_part["HeadRear"]}
        self.head_com = sum(w * m.geom_pos[geom[n]] for n, w in weights.items()) / sum(
            weights.values()
        )
        self.neck = min(self.sec["neck"], key=lambda r: r["area_m2"])
        self.neck_point = np.array(self.neck["centroid_hip_m"])
        pose = self.sec["pose"]
        knee = np.array([pose["knee_m"][0], 0.0, pose["knee_m"][1]])
        # thigh_closed: the lid fastened along the rim closes the channel (Bredt torsion).
        self.thigh_closed = printed.get("thigh_sections", "thigh") == "thigh_closed"
        thigh = self.sec["thigh_closed" if self.thigh_closed else "thigh"]
        if self.thigh_closed and printed.get("thigh_hip_flange") == "cad":
            # Phase D4 CAD: the hip flange is modelled; Bredt cells come from the slices of
            # the thigh, its lid and the flange (sections thigh_torsion), bending and area
            # from the PA12 thigh_closed rows.
            cells = {r["fraction"]: r for r in self.sec["thigh_torsion"]}
            thigh = [
                {**r, "closed_ring": cells[r["fraction"]]["closed_ring"],
                 "enclosed_mid_area_m2": cells[r["fraction"]]["enclosed_mid_area_m2"]}
                for r in thigh
            ]
        elif self.thigh_closed and printed.get("thigh_hip_flange"):
            # Screen assumption ahead of the CAD (phase D4): a 7075 flange on the outer
            # shaft, screwed to the inner wall, bridges the bore that opens the hip-end
            # slices, so those rows take the nearest closed row's cell area for Bredt
            # torsion. Area and bending properties stay those of the PA12 slice.
            cell = next(r for r in thigh if r.get("closed_ring"))
            thigh = [
                r if r.get("closed_ring") or r["station_m"] > cell["station_m"]
                else {**r, "closed_ring": True,
                      "enclosed_mid_area_m2": cell["enclosed_mid_area_m2"]}
                for r in thigh
            ]
        self.stations = {
            "thigh": self._local(thigh, np.zeros(3), math.radians(pose["a1_deg"])),
            "shin": self._local(self.sec["shin"], knee, math.radians(pose["a2_deg"])),
        }
        self.bracket = [r for r in self.sec["hip_bracket"] if r["x_m"] > 0]
        # Flexible TPU ears (phase D candidate): retention screws at the ear root, which is
        # the ear hull's inboard edge where it meets the head pod.
        bumpers = p["physics"].get("bumpers", {})
        self.ear_roots = {}
        if bumpers.get("enabled") and bumpers.get("ear", {}).get("retention"):
            self.ear_group = bumpers["ear"]["retention"]
            tpu = self.materials["materials"]["tpu"]
            washer = self.materials["fasteners"]["thread_profiles"][self.ear_group["thread"]]
            self.ear_pullthrough = (
                math.pi * washer["washer_od_m"] * self.ear_group["tab_thickness_m"]
                * allowable(self.materials, "tpu") / math.sqrt(3)
            )
            self.ear_tpu = tpu
            for name in ("Ear", "Ear_1"):
                g = m.geom(name).id
                mesh = m.geom_dataid[g]
                start, count = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
                rot = np.zeros(9)
                mujoco.mju_quat2Mat(rot, m.geom_quat[g])
                v = m.mesh_vert[start : start + count] @ rot.reshape(3, 3).T + m.geom_pos[g]
                inboard = np.abs(v[:, 1]) <= np.abs(v[:, 1]).min() + 0.0015
                self.ear_roots[g] = (name, v[inboard].mean(axis=0))

    @staticmethod
    def _local(rows, origin, angle):
        """Station centroids in link axes (x hip->knee or knee->axle, z in-plane normal)."""
        e1 = np.array([math.cos(angle), 0.0, math.sin(angle)])
        en = np.array([-math.sin(angle), 0.0, math.cos(angle)])
        out = []
        for r in rows:
            c = np.array(r["centroid_hip_m"]) - origin
            out.append((np.array([c @ e1, c[1], c @ en]), r))
        return out

    @staticmethod
    def tube(od, bore, bending, torsion, axial, shear):
        area, _, z, j = hollow_section(od, bore)
        sigma = bending / z + abs(axial) / area
        tau = torsion * od / (2 * j) + 2 * shear / area
        return von_mises(sigma, tau)

    @staticmethod
    def printed(row, force, moment, wall, closed=False):
        """Printed section stress; force and moment in link axes (x normal, y joint axis).

        Torsion: open section 3T/(A t); a closed single cell (closed=True and a
        two-loop slice) uses Bredt T/(2 A_m t) with the wall mid-line area A_m."""
        a = row["area_m2"]
        sigma = (
            abs(force[0]) / a
            + abs(moment[1]) / row["Z_sagittal_m3"]
            + abs(moment[2]) / row["Z_lateral_m3"]
        )
        cell = row.get("enclosed_mid_area_m2", 0.0) if closed and row.get("closed_ring") else 0.0
        twist = abs(moment[0]) / (2 * cell * wall) if cell > 0 else 3 * abs(moment[0]) / (a * wall)
        tau = twist + 1.5 * math.hypot(force[1], force[2]) / a
        return von_mises(sigma, tau)

    def contacts_on(self, m, d, geoms, body):
        """Contact forces on geoms (solved normal rows and explicit friction), as
        (point, force, torque) in body axes."""
        out = []
        r, o = d.xmat[body].reshape(3, 3), d.xpos[body]
        for ga, gb, pos, f in self.friction:
            for g, sign in ((gb, 1.0), (ga, -1.0)):
                if g in geoms:
                    out.append((r.T @ (pos - o), r.T @ (sign * f), np.zeros(3)))
        for i in range(d.ncon):
            c = d.contact[i]
            for position, g in enumerate(c.geom):
                if g not in geoms:
                    continue
                local = np.zeros(6)
                mujoco.mj_contactForce(m, d, i, local)
                frame = c.frame.reshape(3, 3)
                sign = 1 if position == 1 else -1
                out.append(
                    (r.T @ (c.pos - o), r.T @ (sign * frame.T @ local[:3]),
                     r.T @ (sign * frame.T @ local[3:]))
                )
        return out

    def link(self, name, stations, tip_force, tip_moment, tip, contacts, sgn, wall, closed=False):
        """Worst section stress of a link from the loads distal of each station."""
        worst = 0.0
        for local, row in stations:
            pnt = local * np.array([1, sgn, 1])
            force, moment = tip_force.copy(), shift(tip_force, tip_moment, tip, pnt)
            for at, f, t in contacts:
                if at[0] > pnt[0]:
                    force += f
                    moment += np.cross(at - pnt, f) + t
            worst = max(worst, self.printed(row, force, moment, wall, closed))
        return worst

    def belt_rating(self, torque_small, rpm, rpms, ratings):
        """Rating at the small-pulley speed; above the table the top row is used (flagged)."""
        rating = float(np.interp(math.log(max(rpm, rpms[0])), np.log(rpms), ratings))
        return rating, rpm > rpms[-1]

    def sample(self, m, d, drives, friction=()):
        mujoco.mj_rnePostConstraint(m, d)
        self.friction = friction
        f, dem, flags = {}, {}, {}

        def record(name, demand, capacity):
            dem[name] = float(demand)
            f[name] = float(capacity / max(abs(demand), 1e-12))

        belt = drives.last_forces
        al, pa = self.allow["al7075"], self.allow["pa12"]
        rc = d.xmat[self.chassis].reshape(3, 3)
        for i, side in enumerate(SIDES):
            sgn = 1.0 if side == "left" else -1.0
            yv = {k: sgn * v for k, v in self.y.items()}
            t, sh, wh, kd = (self.ids[f"{side}_{n}"] for n in ("thigh", "shin", "wheel", "kneedrive"))
            rt, rs = d.xmat[t].reshape(3, 3), d.xmat[sh].reshape(3, 3)
            # Axle tube: shin -> wheel wrench (shin axes, about the wheel origin), cantilevered
            # from the shin outer face.
            fw, mw = parent_wrench(m, d, wh)
            fw, mw = rs.T @ fw, rs.T @ mw
            ow = np.array([self.l2, 0.0, 0.0])
            mr = shift(fw, mw, ow, np.array([self.l2, sgn * self.shin_face, 0.0]))
            ax = self.s["shafts"]["axle"]
            record(
                f"{side}_axle_tube",
                self.tube(ax["od"], ax["bore"], math.hypot(mr[0], mr[2]), abs(mr[1]), fw[1],
                          math.hypot(fw[0], fw[2])),
                al,
            )
            span = self.s["hub_bearings"]["span_m_assumption"]
            hub = paired_reactions(
                fw, mw, [sgn * (self.wheel_y - span / 2), sgn * (self.wheel_y + span / 2)]
            )
            dem[f"{side}_hub_bearing_required_C0_N"] = (
                self.s["required_factor"] * max(hub["equivalent_static_N"])
            )
            # Shin: hub reaction on the shin plus shin contacts beyond each station.
            record(
                f"{side}_shin_section",
                self.link("shin", self.stations["shin"], -fw, -mw, ow,
                          self.contacts_on(m, d, self.link_geoms[(side, "shin")], sh), sgn,
                          self.s["printed"]["shin_thickness_m"]),
                self.allow_shin,
            )
            # Knee: the 6700s carry the thigh -> shin wrench less the knee-belt pull on the knee
            # pulley (towards the hip pulley along the thigh).
            fs, ms = parent_wrench(m, d, sh)
            fs, ms = rs.T @ fs, rs.T @ ms
            tk = belt[4 + i]
            pull_k = max(2 * self.t0_knee, abs(tk) / self.r40)
            belt_k = rs.T @ (-rt[:, 0]) * pull_k
            yp = yv["KneePulley"]
            bf = fs - belt_k
            bm = ms - np.cross([0, yp, 0], belt_k)
            # The bearings pass no moment about the knee axis: the belt carries the knee
            # torque. Only the end-stop torque (if engaged) goes through the thigh (hinge -y).
            stop = limit_torque(m, d, f"{side}_shin_joint")
            bm[1] = -stop
            dem[f"{side}_knee_stop_Nm"] = abs(stop)
            dem[f"{side}_hip_stop_Nm"] = abs(limit_torque(m, d, f"{side}_hip"))
            dem[f"{side}_kneedrive_stop_Nm"] = abs(limit_torque(m, d, f"{side}_knee"))
            yi, yo = yv["KneeInnerBearing"], yv["KneeOuterBearing"]
            radius, y_pad = self.pads["knee_angle"]
            pad_k, y_pad = abs(stop) / radius, sgn * y_pad
            react = with_pad(paired_reactions(bf, bm, [yi, yo]), pad_k, y_pad, yi, yo)
            for name, p0 in zip(("KneeInnerBearing", "KneeOuterBearing"), react["equivalent_static_N"]):
                record(f"{side}_{name}", p0, self.c0[name])
            ri = np.array(react["signed_reactions_N"][0])
            mo = np.cross([0, yi - yo, 0], ri) + np.cross([0, yp - yo, 0], belt_k)
            ks = self.s["shafts"]["knee"]
            record(
                f"{side}_knee_shaft",
                self.tube(ks["od"], ks["bore"], math.hypot(mo[0], mo[2]) + pad_k * abs(y_pad - yo),
                          abs(tk), react["axial_N"], max(react["radial_N"])),
                al,
            )
            # Thigh: knee-bearing reaction on the thigh plus thigh contacts beyond each station.
            to_t = rt.T @ rs
            record(
                f"{side}_thigh_section",
                self.link("thigh", self.stations["thigh"], -(to_t @ bf), -(to_t @ bm),
                          np.array([self.l1, 0.0, 0.0]),
                          self.contacts_on(m, d, self.link_geoms[(side, "thigh")], t), sgn,
                          self.s["printed"]["thigh_wall_m"], self.thigh_closed),
                pa,
            )
            # Hip outer shaft on the 6704s: chassis -> thigh wrench less the thigh hip-belt pull.
            fh, mh = parent_wrench(m, d, t)
            fh, mh = rc.T @ fh, rc.T @ mh
            th = belt[2 * i]
            pull = self.to_motor * max(2 * self.t0_hip, abs(th) / self.r80)
            yt = yv["ThighHipPulley"]
            yhi, ypod = yv["HipInnerBearing"], yv["HipPodBearing"]
            radius, y_pad = self.pads["thigh_a1"]
            pad_h, y_pad = abs(limit_torque(m, d, f"{side}_hip")) / radius, sgn * y_pad
            react = with_pad(
                paired_reactions(fh - pull, mh - np.cross([0, yt, 0], pull), [yhi, ypod]),
                pad_h, y_pad, yhi, ypod,
            )
            for name, p0 in zip(("HipInnerBearing", "HipPodBearing"), react["equivalent_static_N"]):
                record(f"{side}_{name}", p0, self.c0[name])
            ri = np.array(react["signed_reactions_N"][0])
            mo = np.cross([0, yhi - ypod, 0], ri) + np.cross([0, yt - ypod, 0], pull)
            hs = self.s["shafts"]["hip_outer"]
            record(
                f"{side}_hip_outer_shaft",
                self.tube(hs["od"], hs["bore"], math.hypot(mo[0], mo[2]) + pad_h * abs(y_pad - ypod),
                          abs(th), react["axial_N"], max(react["radial_N"])),
                al,
            )
            # Printed seats: projected pressure of the pod 6704 in the PA12 hip housing; the
            # inner 6704 reaction carried by the PA12 hip-bracket arm to the side wall.
            record(
                f"{side}_hip_housing_seat",
                react["radial_N"][1] / (self.seat["D"] * self.seat["B"]),
                pa,
            )
            # The pad share (direction unresolved) is added to the vertical component.
            vertical = abs(ri[2]) + react["radial_N"][0] - math.hypot(ri[0], ri[2])
            worst = 0.0
            for row in self.bracket:
                x = row["x_m"]
                z_about_y = row["I_about_a_m4"] / row["extreme_along_b_m"]
                z_about_z = row["I_about_b_m4"] / row["extreme_along_a_m"]
                sigma = (
                    abs(ri[0]) / row["area_m2"]
                    + vertical * x / z_about_y
                    + react["axial_N"] * x / z_about_z
                )
                worst = max(worst, sigma)
            record(f"{side}_hip_bracket_arm", worst, self.allow_bracket)
            # Inner shaft on the MR128s: knee-drive wrench less the knee-drive hip-belt pull
            # (inboard pulley) and the knee-belt pull (outboard pulley, towards the knee).
            fk, mk = parent_wrench(m, d, kd)
            fk, mk = rc.T @ fk, rc.T @ mk
            tkd = belt[2 * i + 1]
            p1 = self.to_motor * max(2 * self.t0_hip, abs(tkd) / self.r80)
            p2 = rc.T @ rt[:, 0] * pull_k
            y1, y2 = yv["KneeHipPulley"], yv["KneeBeltHipPulley"]
            ya, yb = yv["BulkheadBearing"], yv["InnerShaftEndBearing"]
            radius, y_pad = self.pads["knee_drive_a2"]
            pad_d, y_pad = abs(limit_torque(m, d, f"{side}_knee")) / radius, sgn * y_pad
            react = with_pad(
                paired_reactions(
                    fk - p1 - p2, mk - np.cross([0, y1, 0], p1) - np.cross([0, y2, 0], p2), [ya, yb]
                ),
                pad_d, y_pad, ya, yb,
            )
            for name, p0 in zip(("BulkheadBearing", "InnerShaftEndBearing"), react["equivalent_static_N"]):
                record(f"{side}_{name}", p0, self.c0[name])
            ra = np.array(react["signed_reactions_N"][0])
            bending = max(
                math.hypot(*(np.cross([0, ya - ys, 0], ra) + np.cross([0, y1 - ys, 0], p1))[[0, 2]])
                + pad_d * abs(y_pad - ya)
                for ys in (y1, y2)
            )
            ins = self.s["shafts"]["hip_inner"]
            record(
                f"{side}_hip_inner_shaft",
                self.tube(ins["od"], ins["bore"], bending, abs(tkd), react["axial_N"],
                          max(react["radial_N"])),
                al,
            )
            # Belts: slack-side tension and the Gates rating at the small-pulley speed.
            for row, role in ((2 * i, "thigh"), (2 * i + 1, "kneedrive")):
                tq = abs(belt[row])
                name = f"{side}_{role}_hip_belt"
                record(name + "_slack", tq, 2 * self.t0_hip * self.r80)
                rpm = abs(d.qvel[drives.rv[row]]) * 60 / (2 * math.pi)
                rating, over = self.belt_rating(tq, rpm, self.hip_rpm, self.hip_rating)
                record(name + "_rating", tq / self.hip_ratio, rating)
                flags[name + "_above_rated_rpm"] = over
            record(f"{side}_knee_belt_slack", abs(tk), 2 * self.t0_knee * self.r40)
            rel = abs(d.qvel[drives.ov[2 * i + 1]] - d.qvel[drives.ov[2 * i]]) * 60 / (2 * math.pi)
            rating, over = self.belt_rating(abs(tk), rel, self.knee_rpm, self.knee_rating)
            record(f"{side}_knee_belt_rating", abs(tk), rating)
            flags[f"{side}_knee_belt_above_rated_rpm"] = over
        # Neck: head contacts plus head weight and inertia (proper acceleration of the head
        # centre), about the weakest neck section, chassis axes.
        acc = np.zeros(6)
        mujoco.mj_objectAcceleration(m, d, mujoco.mjtObj.mjOBJ_BODY, self.chassis, acc, 0)
        omega = d.cvel[self.chassis][:3]
        head = d.xpos[self.chassis] + rc @ self.head_com
        point = d.xpos[self.chassis] + rc @ self.neck_point
        lever = head - d.xipos[self.chassis]
        proper = acc[3:] + np.cross(acc[:3], lever) + np.cross(omega, np.cross(omega, lever))
        cf, cm = geom_contact_wrench(m, d, self.head_geoms, point, self.friction)
        inertial = -self.head_mass * proper
        force = rc.T @ (cf + inertial)
        moment = rc.T @ (cm + np.cross(head - point, inertial))
        n = self.neck
        sigma = (
            abs(force[2]) / n["area_m2"]
            + abs(moment[0]) * n["extreme_along_b_m"] / n["I_about_a_m4"]
            + abs(moment[1]) * n["extreme_along_a_m"] / n["I_about_b_m4"]
        )
        tau = abs(moment[2]) * n["radius_max_m"] / n["polar_m4"] + 2 * math.hypot(
            force[0], force[1]
        ) / n["area_m2"]
        record("neck_section", von_mises(sigma, tau), pa)
        # Ear retention: the solved ear contact wrench about the root, in chassis axes, on
        # the two root screws (axis y, into the head side wall); TPU tab washer pull-through.
        for g, (name, root) in self.ear_roots.items():
            point = d.xpos[self.chassis] + rc @ root
            ef, em = geom_contact_wrench(m, d, {g}, point, self.friction)
            ef, em = rc.T @ ef, rc.T @ em
            checks = fastener_demands(self.ear_group, ef, em, self.materials, 1)
            normal, capacity = checks["thread_and_head"]
            record(f"{name}_retention_thread", normal, min(capacity, self.ear_pullthrough))
            record(f"{name}_retention_shear", *checks["shear_and_tearout"])
        return f, dem, flags


def contact_elastic_energy(m, d, contacts):
    """Elastic energy stored in the ground material contacts: each contact carries the
    law F(depth) / count of its geom (MaterialContacts), so it stores the integral of that
    law over its own depth."""
    ground = m.geom("ground").id
    active = [i for i in range(d.ncon) if ground in d.contact[i].geom]
    count = {}
    for i in active:
        pair = d.contact[i].geom
        geom = int(pair[1] if pair[0] == ground else pair[0])
        count[geom] = count.get(geom, 0) + 1
    face = getattr(contacts, "face", None)
    total = sum(face.energy.values()) if face is not None else 0.0
    for i in active:
        c = d.contact[i]
        geom = int(c.geom[1] if c.geom[0] == ground else c.geom[0])
        depth = max(0.0, -float(c.dist))
        if depth == 0.0:
            continue
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, geom)
        if face is not None and name in face.rows:
            continue
        axial = abs(float(c.frame[:3] @ d.xmat[m.geom_bodyid[geom]].reshape(3, 3)[:, 1]))
        s = np.linspace(0.0, depth, 17)
        law = [contacts.force(name, x, axial) if x > 0 else 0.0 for x in s]
        total += float(np.trapezoid(law, s)) / count[geom]
    return total


def energy(m, d, drives=None, contacts=None):
    """Mechanical energy: MuJoCo gravity and kinetic terms, plus the belt springs, the
    end-stop pads and the ground material contacts' elastic energy when the drives and
    the material contacts are given."""
    if drives is None:
        mujoco.mj_energyPos(m, d)
        mujoco.mj_energyVel(m, d)
        out = {"potential_J": float(d.energy[0]), "kinetic_J": float(d.energy[1])}
        out["total_J"] = out["potential_J"] + out["kinetic_J"]
        return out
    e = mechanical_energy(m, d, drives)
    pads = 0.0
    for j, s in (contacts.stops if contacts is not None else {}).items():
        q = d.qpos[m.jnt_qposadr[j]]
        lo, hi = m.jnt_range[j]
        over = max(0.0, lo - q, q - hi)
        pads += 0.5 * s["stiffness"] * over**2 + 0.5 * s["backing"] * max(0.0, over - s["travel"]) ** 2
    out = {k: e[k] for k in ("potential_J", "kinetic_J")}
    out["belt_elastic_J"], out["stop_pad_elastic_J"] = e["elastic_J"], pads
    ground = contact_elastic_energy(m, d, contacts) if contacts is not None else 0.0
    out["contact_elastic_J"] = ground
    out["total_J"] = e["total_J"] + pads + ground
    return out


def release(m, d, p, direction):
    """Tilt the nominal standing robot about its base and set the lowest tire on the ground."""
    if direction in ("forward", "backward"):
        angle, axis = math.radians(10 if direction == "forward" else -10), np.array([0, 1.0, 0])
    else:
        mujoco.mj_forward(m, d)
        com = d.subtree_com[m.body("chassis").id]
        g = p["geometry"]
        edge = g["wheel_track"] / 2 + g["wheel_width"] / 2
        angle = math.atan2(edge, com[2]) + math.radians(5)
        axis = np.array([1.0, 0, 0])
    tilt = np.r_[math.cos(angle / 2), axis * math.sin(angle / 2)]
    quat = np.zeros(4)
    mujoco.mju_mulQuat(quat, tilt, d.qpos[3:7].copy())
    d.qpos[3:7] = quat
    mujoco.mj_forward(m, d)
    ground, distance = m.geom("ground").id, np.zeros(6)
    supports = [f"{s}_tire" for s in SIDES] + [
        m.geom(i).name for i in range(m.ngeom) if m.geom(i).name.endswith(("WheelRing", "SkidPad"))
    ]
    depth = min(
        mujoco.mj_geomDistance(m, d, ground, m.geom(n).id, 1.0, distance) for n in supports
    )
    d.qpos[2] -= depth
    mujoco.mj_forward(m, d)
    return math.degrees(angle)


def run_case(shared, direction, speed, duration=3.0, dt=None, output=None, friction="explicit"):
    """One unpowered tip at FALL_DT_S unless dt is given. friction: "explicit"
    (CoulombContacts) or "native" (the plant's solver friction rows, kept for the
    comparison that motivated the explicit form)."""
    p = parameters(shared)
    p["simulation"]["physics_dt"] = FALL_DT_S if dt is None else dt
    m, d = build(p)
    m.opt.timestep = p["simulation"]["physics_dt"]
    callback = CoulombContacts(m, p) if friction == "explicit" else None
    reset(m, d, p)
    angle = release(m, d, p, direction)
    d.qvel[0] = speed
    for side in SIDES:
        d.qvel[m.joint(f"{side}_wheel_joint").dofadr[0]] = speed / p["geometry"]["wheel_radius"]
    drives = MechanicalDrives(m, p)
    drives.forward(m, d, callback)
    structure = Structure(shared, p, m)
    ground = m.geom("ground").id
    names = [m.geom(i).name for i in range(m.ngeom)]
    shell_backing = p["physics"]["contact"].get("shell_backing", {})
    backing = {k: np.array(v["spots_m"]) for k, v in shell_backing.get("parts", {}).items()}
    backing_radius = shell_backing.get("radius_m", 0.0)
    minimum, when, peak, contacts, flagged, first = {}, {}, {}, {}, {}, None
    stops = callback.normal if callback is not None else drives.contacts
    initial = energy(m, d, drives, stops)
    previous = initial
    # Unpowered: total mechanical energy (gravity, kinetic, belt springs, stop pads and
    # ground contacts) can only fall. A step increase above the threshold is an injection
    # when the motion energy (total less the contact elastic term) rises with it; when only
    # the stored contact energy steps up, the mesh-plane collision has swapped the hull
    # vertices it reports (contact-patch resampling), recorded separately.
    injections, resampling = [], []
    trace = []
    dt = m.opt.timestep
    for step in range(round(duration / dt)):
        drives.step(m, d, np.zeros(6), contact_callback=callback)
        factors, demands, flags = structure.sample(
            m, d, drives, callback.friction if callback else ()
        )
        for k, v in flags.items():
            flagged[k] = flagged.get(k, False) or v
        for k, v in factors.items():
            if v < minimum.get(k, math.inf):
                minimum[k], when[k] = v, float(d.time)
        for k, v in demands.items():
            peak[k] = max(peak.get(k, 0.0), abs(v))
        for i in range(d.ncon):
            c = d.contact[i]
            if ground not in c.geom:
                continue
            gid = c.geom[1] if c.geom[0] == ground else c.geom[0]
            force = np.zeros(6)
            mujoco.mj_contactForce(m, d, i, force)
            name = names[gid]
            row = contacts.setdefault(name, {"peak_normal_N": 0.0, "peak_depth_m": 0.0})
            row["peak_normal_N"] = max(row["peak_normal_N"], abs(float(force[0])))
            row["peak_depth_m"] = max(row["peak_depth_m"], max(0.0, -float(c.dist)))
            if name in backing:
                # Measured backing gaps hold only near the probed spots (body frame).
                b = m.geom_bodyid[gid]
                local = d.xmat[b].reshape(3, 3).T @ (c.pos - d.xpos[b])
                near = any(np.linalg.norm(local - s) <= backing_radius for s in backing[name])
                key = "peak_depth_at_spot_m" if near else "peak_depth_off_spot_m"
                row[key] = max(row.get(key, 0.0), max(0.0, -float(c.dist)))
            if "tire" not in name and abs(force[0]) > 1e-6 and first is None:
                first = {"time_s": float(d.time), "part": name, **energy(m, d, drives, stops)}
        now = energy(m, d, drives, stops)
        rise = now["total_J"] - previous["total_J"]
        if rise > ENERGY_EVENT_J:
            motion = rise - (now["contact_elastic_J"] - previous["contact_elastic_J"])
            event = {"time_s": float(d.time), "increase_J": float(rise), "motion_J": float(motion)}
            (injections if motion > ENERGY_EVENT_J else resampling).append(event)
        previous = now
        if step % round(0.002 / dt) == 0:
            trace.append(
                [d.time, *d.qpos[:7], *[factors[k] for k in sorted(factors)]]
            )
    worst = min(minimum, key=minimum.get)
    result = {
        "direction": direction,
        "speed_m_s": speed,
        "release_angle_deg": angle,
        "dt_s": dt,
        "friction_model": friction,
        "initial_energy": initial,
        "energy_injection_events": injections,
        "energy_injection_total_J": float(sum(i["increase_J"] for i in injections)),
        "contact_resampling_events": resampling,
        "contact_resampling_total_J": float(sum(i["increase_J"] for i in resampling)),
        "first_non_tire_contact": first,
        "contacts": contacts,
        "minimum_factors": minimum,
        "minimum_factor_time_s": when,
        "peak_demands": peak,
        "belt_speed_above_rating_table": sorted(k for k, v in flagged.items() if v),
        "worst_path": worst,
        "worst_factor": minimum[worst],
        "failures": sorted(k for k, v in minimum.items() if v < structure.s["required_factor"]),
        "drive_metrics": drives.metrics(),
    }
    # Printed parts beyond the shell law's modelled travel bottom onto the backing: a
    # failure (M9 constitutive scope), never a clip.
    law = callback.normal if callback is not None else drives.contacts
    face, ears = getattr(law, "face", None), getattr(law, "ears", None)
    travel = law.shell["maximum_travel_m"]
    result["shell_travel_m"] = travel
    parts = shell_backing.get("parts", {})
    result["shell_backing_travel_m"] = {k: v["travel_m"] for k, v in parts.items()}

    def exceeded(k, v):
        if k not in parts:
            return v["peak_depth_m"] > travel
        # At a probed spot the measured gap applies; anywhere else the default travel.
        return (
            v.get("peak_depth_at_spot_m", 0.0) > parts[k]["travel_m"]
            or v.get("peak_depth_off_spot_m", 0.0) > travel
        )

    result["failures"] += sorted(
        f"{k}_shell_travel_exceeded"
        for k, v in contacts.items()
        if "tire" not in k and exceeded(k, v)
        and not (face is not None and k in face.rows)
        and not (ears and k in ears["names"])
    )
    if face is not None:
        bumpers = p["physics"]["bumpers"]
        result["bumper_peak_strain"] = face.peak_strain
        result["failures"] += sorted(
            f"{k}_bumper_bottomed" for k, v in face.peak_strain.items() if v > bumpers["max_strain"]
        )
    if ears:
        strain = {k: contacts.get(k, {}).get("peak_depth_m", 0.0) * ears["strain_per_m"] for k in ears["names"]}
        result["ear_peak_root_strain"] = strain
        result["failures"] += sorted(
            f"{k}_root_strain" for k, v in strain.items() if v > ears["allowable_strain"]
        )
    if callback is not None:
        pads = p["physics"]["end_stop_pads"]
        result["stop_pad_peak_strain"] = callback.normal.peak_stop_strain
        over = sorted(k for k, v in callback.normal.peak_stop_strain.items() if v > pads["max_strain"])
        result["failures"] += [f"{k}_stop_pad_bottomed" for k in over]
    result["passed"] = not result["failures"]
    if output is not None:
        output.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            output / "trace.npz",
            trace=np.array(trace),
            columns=np.array(["time", "x", "y", "z", "qw", "qx", "qy", "qz", *sorted(minimum)]),
        )
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


# Candidate patches that change only the screen's ratings; their factors scale exactly from
# the baseline demands (the dynamics are unchanged).
SCALED_KEYS = {"bearings", "printed.hip_bracket_material", "printed.shin_material"}


def _patch_keys(patch, prefix=""):
    for k, v in patch.items():
        if isinstance(v, dict) and k != "bearings":
            yield from _patch_keys(v, f"{prefix}{k}.")
        else:
            yield f"{prefix}{k}"


def scaled_factors(shared, name, minimum):
    """Minimum factors of a ratings-only candidate from the baseline minima, or None if
    the candidate changes the dynamics or the load model (it needs its own run)."""
    patch = shared["m10"]["structure"]["candidates"][name]["patch"]
    keys = {k.removeprefix("structure.") for k in _patch_keys(patch)}
    if not all(k.startswith("structure.") for k in _patch_keys(patch)) or not keys <= SCALED_KEYS:
        return None
    materials = json.loads((ROOT / shared["m10"]["structure"]["materials_source"]).read_text())

    def ratings(structure):
        cat, brg, pr = structure["bearing_catalog"], structure["bearings"], structure["printed"]
        seat = cat[brg["HipPodBearing"]]
        out = {n: cat[k]["C0_N"] for n, k in brg.items()}
        out["hip_housing_seat"] = seat["D"] * seat["B"]
        out["hip_bracket_arm"] = allowable(materials, pr.get("hip_bracket_material", "pa12"))
        out["shin_section"] = allowable(materials, pr.get("shin_material", "pa12"))
        return out

    old = ratings(shared["m10"]["structure"])
    new = ratings(candidate_config(shared, name)["m10"]["structure"])
    out = {}
    for path, factor in minimum.items():
        key = path.split("_", 1)[1] if path.startswith(tuple(SIDES)) else path
        out[path] = factor * new[key] / old[key] if key in old else factor
    return out


def report(output, shared):
    """Candidate table: the minimum factor per path over the six tips, the failing paths and
    the mass estimate. Ratings-only candidates are scaled from the baseline; the others are
    read from their own runs (output/candidates/<name>)."""
    base = json.loads((output / "summary.json").read_text())
    cases = sorted(base["cases"])
    per_case = {c: json.loads((output / c / "result.json").read_text()) for c in cases}
    required = shared["m10"]["structure"]["required_factor"]
    rows = {}
    for name, cand in shared["m10"]["structure"]["candidates"].items():
        if name == "basis":
            continue
        scaled = [scaled_factors(shared, name, per_case[c]["minimum_factors"]) for c in cases]
        if scaled[0] is not None:
            source, minima = "scaled from the baseline demands", scaled
        else:
            run = output / "candidates" / name
            if not (run / "summary.json").exists():
                continue
            source = str(run.relative_to(ROOT))
            minima = [json.loads((run / c / "result.json").read_text())["minimum_factors"] for c in cases]
        by_path = {p: min(m[p] for m in minima) for p in minima[0]}
        rows[name] = {
            "source": source,
            "visible": cand["visible"],
            "mass_delta_kg_estimate": cand["mass_delta_kg_estimate"],
            "minimum_factor_by_path": by_path,
            "failing_paths": sorted(p for p, v in by_path.items() if v < required),
        }
    out = {
        "baseline_failing_paths": sorted(
            p for p, v in base["minimum_factor_by_path"].items() if v < required
        ),
        "candidates": rows,
    }
    (output / "candidates" / "summary.json").write_text(json.dumps(out, indent=2) + "\n")
    return out


def _job(args):
    name, direction, speed, duration, dt, output, candidate = args
    shared = candidate_config(json.loads((ROOT / "config/robot.json").read_text()), candidate)
    return name, run_case(shared, direction, speed, duration, dt, output / name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="artifacts/m10/falls")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--dt", type=float)
    parser.add_argument("--case", action="append", help="case name, e.g. forward_rest (repeatable)")
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument(
        "--speeds", type=float, nargs="+", default=[0.0, 0.3], help="initial speeds, m/s"
    )
    parser.add_argument("--candidate", help="remedy candidate in m10.structure.candidates")
    parser.add_argument("--report", action="store_true", help="candidate table only, no runs")
    args = parser.parse_args()
    output = ROOT / args.output
    if args.report:
        report(output, json.loads((ROOT / "config/robot.json").read_text()))
        return
    shared = candidate_config(json.loads((ROOT / "config/robot.json").read_text()), args.candidate)
    cases = [
        (f"{direction}_{'rest' if speed == 0 else f'{speed:g}'.replace('.', 'p')}", direction, speed)
        for direction in ("forward", "backward", "sideways")
        for speed in args.speeds
    ]
    if args.case:
        cases = [c for c in cases if c[0] in args.case]
    output.mkdir(parents=True, exist_ok=True)
    (output / "configuration.json").write_text(
        json.dumps(
            {
                "structure": shared["m10"]["structure"],
                "end_stop_pads": shared["m10"]["physics"]["end_stop_pads"],
                "bumpers": shared["m10"]["physics"].get("bumpers"),
                "shell_backing": shared["m10"]["physics"]["contact"].get("shell_backing"),
                "fall_dt_s": args.dt or FALL_DT_S,
                "experiment": vars(args),
            },
            indent=2,
        )
        + "\n"
    )
    jobs = [
        (n, direction, speed, args.duration, args.dt, output, args.candidate)
        for n, direction, speed in cases
    ]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = dict(pool.map(_job, jobs))
    for name, r in results.items():
        print(json.dumps({"case": name, "worst": r["worst_path"], "factor": r["worst_factor"],
                          "failures": len(r["failures"])}), flush=True)
    paths = next(iter(results.values()))["minimum_factors"]
    summary = {
        "cases": {
            k: {kk: v[kk] for kk in ("worst_path", "worst_factor", "failures", "passed",
                                     "first_non_tire_contact", "energy_injection_total_J",
                                     "contact_resampling_total_J")}
            for k, v in results.items()
        },
        "minimum_factor_by_path": {p: min(r["minimum_factors"][p] for r in results.values()) for p in paths},
        "peak_demand_by_quantity": {
            q: max(r["peak_demands"][q] for r in results.values())
            for q in next(iter(results.values()))["peak_demands"]
        },
        "passed": all(r["passed"] for r in results.values()),
        "requirement": requirement(results),
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")


def requirement(results):
    """Phase D requirement (user decision 13 October 2026): forward and backward tips keep
    factor >= 2 on every path; sideways tips are containment cases, so cosmetic parts must
    stay attached (retention factor >= 1, ear root strain within the allowable) and within
    the shell law's travel (the measured gap at the probed strike spots, user decision
    17 October 2026). Sideways structural shortfalls are reported as recorded risk."""
    cases = {}
    for name, r in results.items():
        if r["direction"] != "sideways":
            cases[name] = {"kind": "factor_2", "passed": r["passed"], "open": r["failures"]}
            continue
        open_ = sorted(
            [f for f in r["failures"] if f.endswith(("_shell_travel_exceeded", "_root_strain"))]
            + [k for k, v in r["minimum_factors"].items() if "_retention_" in k and v < 1.0]
        )
        cases[name] = {
            "kind": "containment",
            "passed": not open_,
            "open": open_,
            "recorded_risk": [f for f in r["failures"] if f not in open_],
        }
    return {
        "basis": "User decision 13 October 2026: forward/backward factor >= 2 on every path; "
        "sideways tips are containment cases (cosmetic parts attached and within the shell "
        "travel), their structural shortfalls a recorded risk",
        "cases": cases,
        "passed": all(c["passed"] for c in cases.values()),
    }


if __name__ == "__main__":
    main()
