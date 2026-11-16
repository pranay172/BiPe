"""Passive M9 contact falls and material/load-path calculations.

This is an experiment module, not a hardware certification. It uses actual
CAD mass/tensors, unilateral contacts, finite belts/slip clutches and stated
material knockdowns. No force, pose, speed, or measured robot state is clipped
to meet an acceptance threshold. Exceeded travel/strength is a failed gate.
"""

import argparse
import copy
import csv
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from .parameters import ROOT

MATERIALS = ROOT / "config/m9-impact-materials.json"


def load_materials():
    return json.loads(MATERIALS.read_text())


def allowable(materials, name):
    row = materials["materials"][name]
    return row["yield_supplier_Pa"] * row["strength_knockdown"]


def hollow_section(od, bore):
    """Area, second moment, bending modulus, polar moment for a hollow shaft."""
    area = math.pi * (od**2 - bore**2) / 4
    inertia = math.pi * (od**4 - bore**4) / 64
    return area, inertia, 2 * inertia / od, 2 * inertia


def paired_reactions(force, moment, stations):
    """Signed X/Z reactions from wrench at Y=0, plus worst axial assignment.

    Positive/negative reactions are retained. A deep-groove bearing can carry
    reversed radial loads; its housing/shaft retention must also carry them.
    One bearing takes all axial load, conservatively, rather than assuming an
    unspecified preload shares it between races.
    """
    force, moment = np.asarray(force), np.asarray(moment)
    a, b = stations
    span = b - a
    front_moment = moment - np.cross([0, a, 0], force)
    rear = np.array([-front_moment[2] / span, 0, front_moment[0] / span])
    front = force - rear
    front[1] = 0
    radial = np.array([np.linalg.norm(front[[0, 2]]), np.linalg.norm(rear[[0, 2]])])
    axial = abs(force[1])
    # Deep-groove static equivalent load P0=max(Fr,0.6Fr+0.5Fa).
    # Check either possible locating bearing, without relying on load sharing.
    equivalent = np.maximum(radial, 0.6 * radial + 0.5 * axial)
    return {
        "signed_reactions_N": [front.tolist(), rear.tolist()],
        "radial_N": radial.tolist(),
        "axial_N": axial,
        "equivalent_static_N": equivalent.tolist(),
    }


def hardware_ledger(materials=None):
    materials = materials or load_materials()
    hardware = materials["fasteners"]
    density = hardware["density_kg_m3"]
    rows = []
    for group in hardware["schedule"]:
        size = hardware["thread_profiles"][group["thread"]]
        screw = (
            density
            * math.pi
            / 4
            * (
                size["diameter_m"] ** 2 * group["length_m"]
                + size["head_diameter_m"] ** 2 * size["head_height_m"]
            )
        )
        washer = (
            density
            * math.pi
            / 4
            * (size["washer_od_m"] ** 2 - size["washer_id_m"] ** 2)
            * size["washer_thickness_m"]
        )
        mass = group["count"] * (screw + group["washers_per_screw"] * washer)
        rows.append(
            {
                "name": group["name"],
                "count": group["count"],
                "mass_kg": mass,
                "basis": "Full solid head/shank/washer envelopes; no recess/thread subtraction",
            }
        )
    insert = hardware["insert"]
    insert_mass = (
        insert["count"]
        * insert["density_kg_m3"]
        * math.pi
        / 4
        * (insert["od_m"] ** 2 - insert["id_m"] ** 2)
        * insert["length_m"]
    )
    rows.append(
        {"name": "SPIROL19_long_M2_inserts", "count": insert["count"], "mass_kg": insert_mass}
    )
    rows.append(
        {
            "name": "rings_shims_nuts_clips",
            "mass_kg": hardware["retaining_rings_shims_nuts_and_clips_mass_kg_assumption"],
        }
    )
    mass = sum(row["mass_kg"] for row in rows)
    return {
        "rows": rows,
        "total_mass_kg": mass,
        "existing_allocation_kg": hardware["existing_allocation_kg"],
        "allocation_margin_kg": hardware["existing_allocation_kg"] - mass,
        "fit_status": hardware["retention_schedule_status"],
    }


def fastener_capacities(group, materials):
    """Bolt proof/shear, parent tearout/thread stripping and insert pullout."""
    fast = materials["fasteners"]
    size = fast["thread_profiles"][group["thread"]]
    diameter = size["diameter_m"]
    parent = allowable(materials, group["parent_material"])
    shear = parent / math.sqrt(3)
    edge_ligament = max(0, group["edge_center_distance_m"] - diameter / 2)
    capacities = {
        "bolt_proof_N": fast["proof_stress_Pa"] * size["stress_area_m2"],
        "bolt_shear_N": fast["yield_stress_Pa"]
        / math.sqrt(3)
        * math.pi
        / 4
        * size["root_diameter_m"] ** 2,
        "parent_bearing_N": parent * diameter * group["member_thickness_m"],
        "parent_tearout_N": 2 * shear * edge_ligament * group["member_thickness_m"],
        # Conservative half cylinder of engaged parent thread receives shear.
        "parent_thread_strip_N": 0.5
        * math.pi
        * size["root_diameter_m"]
        * group["thread_engagement_m"]
        * shear,
    }
    if group["parent_material"] == "pa12":
        ins = fast["insert"]
        capacities["insert_pullout_N"] = min(
            ins["comparison_knockdown"] * ins["nylon66_test_pullout_N"],
            0.5 * math.pi * ins["od_m"] * ins["length_m"] * shear,
        )
        # Washer pulls through printed member by punching shear.
        capacities["washer_pullthrough_N"] = (
            math.pi * size["washer_od_m"] * group["member_thickness_m"] * shear
        )
    return capacities


def fastener_demands(group, force, moment, materials, axis):
    """Separate axial thread/head loads from shear tearout and bolt VM."""
    force, moment = np.asarray(force), np.asarray(moment)
    transverse = [i for i in range(3) if i != axis]
    normal = (
        abs(force[axis]) / group["per_interface"]
        + np.linalg.norm(moment[transverse]) / group["bolt_spacing_m"]
    )
    shear = (
        np.linalg.norm(force[transverse]) / group["per_interface"]
        + abs(moment[axis]) / group["bolt_spacing_m"]
    )
    size = materials["fasteners"]["thread_profiles"][group["thread"]]
    root_area = math.pi * size["root_diameter_m"] ** 2 / 4
    capacities = fastener_capacities(group, materials)
    tension_capacity = min(
        v
        for k, v in capacities.items()
        if k
        in ["bolt_proof_N", "parent_thread_strip_N", "insert_pullout_N", "washer_pullthrough_N"]
    )
    shear_capacity = min(
        v
        for k, v in capacities.items()
        if k in ["bolt_shear_N", "parent_bearing_N", "parent_tearout_N"]
    )
    combined_stress = math.sqrt(
        (normal / size["stress_area_m2"]) ** 2 + 3 * (shear / root_area) ** 2
    )
    return {
        "thread_and_head": (normal, tension_capacity),
        "shear_and_tearout": (shear, shear_capacity),
        "bolt_combined": (combined_stress, materials["fasteners"]["proof_stress_Pa"]),
    }


def foam_stress(strain, materials, stiffness_scale=1.0):
    """Continuous compressive law; beyond catalog range, stated densification.

    No force clipping occurs at the permitted strain. This permits a physical
    overload trajectory while acceptance separately rejects exceeded travel.
    """
    row = materials["materials"]["ld70_upper"]
    strain = np.maximum(np.asarray(strain), 0)
    stress = np.interp(strain, row["compression_strain"], row["compression_stress_Pa"])
    excess = np.maximum(strain - row["densification_start_assumption"], 0)
    stress += row["densification_modulus_Pa_assumption"] * excess**2
    return stress * stiffness_scale


def tire_force(p, depth, materials, axial_fraction=0, stiffness_scale=1.0):
    """Cylinder-on-plane foam foundation including 1mm TPU in series.

    Radial patch width follows cylindrical geometry and is integrated rather
    than assigning the full tire ring as an impact contact patch. Lateral
    contact uses annular face area and finite foam width. Oblique contact
    interpolates these two declared compliance surrogates.
    """
    if depth <= 0:
        return 0.0
    g, s = p["geometry"], p["structure"]["tire_construction"]
    radius, width = g["wheel_radius"], g["wheel_width"]
    outer = radius - s["outer_tpu_skin_m"] - s["bond_film_m"]
    inner = g["wheel_tire_inner"] + s["bond_film_m"]
    foam_thickness = outer - inner
    # Efoam secant at50percent; TPU series correction is small but nonzero.
    efoam = 251000 / 0.5 * stiffness_scale
    etpu = materials["materials"]["tpu"]["modulus_Pa"]
    foam_fraction = (foam_thickness / efoam) / (
        foam_thickness / efoam + s["outer_tpu_skin_m"] / etpu
    )
    foam_depth = depth * foam_fraction
    half_patch = math.sqrt(max(0, 2 * radius * min(depth, radius) - min(depth, radius) ** 2))
    x = np.linspace(-half_patch, half_patch, 33)
    local = np.maximum(foam_depth - (radius - np.sqrt(np.maximum(radius**2 - x * x, 0))), 0)
    radial = width * float(
        np.trapezoid(foam_stress(local / foam_thickness, materials, stiffness_scale), x)
    )
    annular_area = math.pi * (outer**2 - inner**2)
    lateral = annular_area * float(foam_stress(depth / width, materials, stiffness_scale))
    return (1 - axial_fraction) * radial + axial_fraction * lateral


def shell_strip(p, materials):
    c = materials["contacts"]
    t = p["geometry"]["shell_wall"]
    b, span = c["shell_effective_strip_width_m_assumption"], c["shell_beam_span_m_assumption"]
    inertia = b * t**3 / 12
    stiffness = 48 * materials["materials"]["pa12"]["modulus_Pa"] * inertia / span**3
    stress_per_force = 1.5 * span / (b * t * t) * c["shell_notch_factor_assumption"]
    return {
        "stiffness_N_m": stiffness,
        "stress_per_force_Pa_N": stress_per_force,
        "maximum_travel_m": c["shell_maximum_travel_m_assumption"],
        "capacity_N": allowable(materials, "pa12") / stress_per_force,
    }


def restitution_damping_ratio(restitution):
    log = math.log(restitution)
    return -log / math.sqrt(math.pi**2 + log**2)


class MaterialContacts:
    """Convert actual material force/depth to MuJoCo reference acceleration.

    The conversion uses the per-contact normal effective mass J M^-1 J.T.
    A very small constant impedance avoids MuJoCo's gravity compensation acting
    as a hidden contact spring. A one-DoF force/depth test checks this mapping.
    Contacts remain unilateral, frictional, and finite; no pose projections.

    MuJoCo regularises each row with its approximate diagonal (efc_diagA, from
    the qpos0 body weights), so with the small impedance the solved force is the
    target law times J M^-1 J.T / diagA. That ratio is 1 for a free coupon but
    about 1.9 for the M10 tires at rest and 56 for a folded-leg shin strike.
    solver_mobility=True maps the law through diagA so the solved force equals
    it in any configuration (the damping ratio still uses the true effective
    mass). The default keeps the M9 archives reproducible.
    """

    solver_mobility = False

    def __init__(
        self,
        p,
        materials=None,
        stiffness_scale=1.0,
        restitution=0.3,
        cache_enabled=True,
        backing_stiffness_scale=1.0,
    ):
        self.p = p
        self.materials = (
            materials
            if materials is not None
            else p["contact_materials"]
            if "contact_materials" in p
            else load_materials()
        )
        self.scale, self.zeta = stiffness_scale, restitution_damping_ratio(restitution)
        self.shell = shell_strip(p, self.materials)
        self.peak_depth = {}
        self.effective_mass_range = [math.inf, 0]
        self.cache_enabled = cache_enabled
        self.backing_scale = backing_stiffness_scale
        self.cache_hits, self.cache_calls = 0, 0
        self._cache_key = None
        self._cache_refs = None
        self._configured_model = None
        self._mass_signature = None
        self.friction_damping = None

    def configure(self, m):
        # Fix impedance beforestep1 so MuJoCo's cached constraint projection
        # and reference use the same regularization. Modifying only the
        # references afterstep1 would leave its cached inverse at the oldd.
        # This fall variant shares impedance across exterior contacts; physical
        # equality joints retain their independently specified parameters.
        impedance = self.materials["contacts"]["impedance"]
        m.geom_solimp[:] = [impedance, impedance, 0.001, 0.5, 2]
        # Coulomb friction rows. The small shared impedance otherwise turns
        # contact friction into a viscous damper that creeps under any steady
        # tangential load. A large impratio removes that friction
        # regularization; finite rate damping on each friction row keeps the
        # stick explicit and stable. Absent keys reproduce the earlier
        # viscous-friction archives.
        self.friction_damping = self.materials["contacts"].get("friction_row_damping_s_inv")
        if self.friction_damping is not None:
            m.opt.impratio = self.materials["contacts"]["friction_impratio"]
        self._configured_model = id(m)
        # Public experiments keep these model constants fixed after build.
        # A new/mutated model must be configured again, or disable the cache.
        self._mass_signature = tuple(
            a.tobytes()
            for a in [m.body_mass, m.body_inertia, m.body_ipos, m.body_iquat, m.dof_armature]
        )
        self._cache_key, self._cache_refs = None, None

    def _friction_rows(self, d):
        """Direct friction-row damping for every contact, ground and self."""
        if self.friction_damping is None or not d.ncon:
            return
        # Direct MuJoCo references divide damping by dmax; scale it back.
        d.contact.solreffriction[: d.ncon, 0] = -1.0
        d.contact.solreffriction[: d.ncon, 1] = (
            -self.friction_damping * d.contact.solimp[: d.ncon, 1]
        )

    @staticmethod
    def _rebuild(m, d):
        mujoco.mj_makeConstraint(m, d)
        # Fresh constraints need fresh island maps before the native NoSlip
        # postsolver uses their friction rows. No robot state is projected.
        mujoco.mj_island(m, d)
        mujoco.mj_projectConstraint(m, d)
        mujoco.mj_referenceConstraint(m, d)

    def category(self, name):
        if name.startswith("ClosureBumper"):
            return "bumper"
        if "tire" in name:
            return "tire"
        if "Shell" in name or "Eye" in name or name == "Smile":
            return "shell"
        if "Cover" in name or "Readhead" in name:
            return "cover"
        if "ImpactPad" in name:
            return "pad"
        return "metal"

    def force(self, name, depth, axial_fraction=0):
        kind, c = self.category(name), self.materials["contacts"]
        if kind == "bumper":
            pad = next(b for b in self.p["candidate_bumpers"] if b["name"] == name)
            size, axis = np.array(pad["size_m"]), pad["axis"]
            strain = max(0, depth / size[axis])
            foam = self.p.get(
                "candidate_bumper_material", self.materials["materials"]["ld45_upper"]
            )
            stress = np.interp(strain, foam["compression_strain"], foam["compression_stress_Pa"])
            stress += foam["densification_modulus_Pa_assumption"] * max(0, strain - 0.5) ** 2
            return self.scale * stress * np.prod(np.delete(size, axis))
        if kind == "tire":
            return tire_force(self.p, depth, self.materials, axial_fraction, self.scale)
        if kind == "shell":
            force = self.shell["stiffness_N_m"] * self.scale * depth
            # Interior backing is finite stiffness, never a force/stroke clamp.
            force += (
                max(0, depth - self.shell["maximum_travel_m"])
                * c["metal_contact_stiffness_N_m_assumption"]
                * self.backing_scale
            )
            return force
        if kind == "cover":
            t = self.p["structure"]["cover_wall"]
            k = 4 * self.materials["materials"]["pa12"]["modulus_Pa"]
            k *= c["cover_effective_width_m"] * t**3 / c["cover_beam_span_m"] ** 3
            return k * self.scale * depth
        if kind == "pad":
            # Existing native pad has15mm rounded width; bending/compression
            # of5mm effective TPU thickness on15x20mm projected area.
            e = self.materials["materials"]["tpu"]["flexural_modulus_Pa"]
            return self.scale * e * 0.015 * 0.020 / 0.005 * depth
        return c["metal_contact_stiffness_N_m_assumption"] * depth

    def __call__(self, m, d):
        ground = m.geom("ground").id
        if self._configured_model != id(m):
            raise ValueError("Configure the material contact model before stepping")
        self.cache_calls += 1
        key = None
        if self.cache_enabled:
            key = (
                self._mass_signature,
                d.qpos.tobytes(),
                d.contact.geom.tobytes(),
                d.contact.pos.tobytes(),
                d.contact.frame.tobytes(),
                d.contact.dist.tobytes(),
                m.geom_quat[ground].tobytes(),
                m.geom_pos[ground].tobytes(),
            )
            if key == self._cache_key:
                refs, impedance = self._cache_refs
                d.contact.solref[:] = refs
                d.contact.solimp[:] = impedance
                self._friction_rows(d)
                self.cache_hits += 1
                self._rebuild(m, d)
                return
        active = [i for i in range(d.ncon) if ground in d.contact[i].geom]
        count = {}
        for i in active:
            pair = d.contact[i].geom
            geom = pair[1] if pair[0] == ground else pair[0]
            count[geom] = count.get(geom, 0) + 1
        jac_a, jac_b = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        for i in active:
            contact = d.contact[i]
            ga, gb = contact.geom
            geom = gb if ga == ground else ga
            body = m.geom_bodyid[geom]
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, geom)
            normal = contact.frame[:3]
            depth = max(0, -float(contact.dist))
            self.peak_depth[name] = max(self.peak_depth.get(name, 0), depth)
            mujoco.mj_jac(m, d, jac_a, None, contact.pos, int(m.geom_bodyid[ga]))
            mujoco.mj_jac(m, d, jac_b, None, contact.pos, int(m.geom_bodyid[gb]))
            row = np.ascontiguousarray((normal @ (jac_b - jac_a))[None, :])
            inverse = np.zeros_like(row)
            mujoco.mj_solveM(m, d, inverse, row)
            mobility = (row @ inverse.T).item()
            if mobility <= 0:
                raise ValueError("Physical contact has no normal mobility")
            effective_mass = 1 / mobility
            self.effective_mass_range[0] = min(self.effective_mass_range[0], effective_mass)
            self.effective_mass_range[1] = max(self.effective_mass_range[1], effective_mass)
            axial = abs(float(normal @ d.xmat[body].reshape(3, 3)[:, 1]))
            # Secant at positive contact penetration. Small-depth tangent
            # keeps initial contact finite and avoids zero-stiffness onset.
            sample_depth = max(depth, 1e-6)
            stiffness = self.force(name, sample_depth, axial) / sample_depth / count[geom]
            damping = 2 * self.zeta * math.sqrt(stiffness * effective_mass)
            impedance = self.materials["contacts"]["impedance"]
            contact.solimp[:] = [impedance] * 2 + [0.001, 0.5, 2]
            if self.solver_mobility and contact.efc_address >= 0:
                scale = float(d.efc_diagA[contact.efc_address])
            else:
                scale = 1 / effective_mass
            contact.solref[:] = [-stiffness * scale, -damping * scale]
        self._friction_rows(d)
        # step1 made constraints using old references. Rebuild the physical
        # contact rows after changing their constitutive parameters.
        if self.cache_enabled:
            self._cache_key = key
            self._cache_refs = (d.contact.solref.copy(), d.contact.solimp.copy())
        self._rebuild(m, d)


def wrench_in_body(m, d, name):
    """Parent-to-body RNE wrench at the native body origin, local axes."""
    body = m.body(name).id
    wrench = d.cfrc_int[body]
    root_com = d.subtree_com[m.body_rootid[body]]
    force = wrench[3:]
    moment = wrench[:3] - np.cross(d.xpos[body] - root_com, force)
    rotation = d.xmat[body].reshape(3, 3)
    return rotation.T @ force, rotation.T @ moment


def contact_wrench(m, d, names, target_body, point_local=None):
    """Solved contact wrench on named native components, in target axes."""
    body = m.body(target_body).id
    rotation = d.xmat[body].reshape(3, 3)
    point_local = np.zeros(3) if point_local is None else np.asarray(point_local)
    origin = d.xpos[body] + rotation @ point_local
    force, moment = np.zeros(3), np.zeros(3)
    for index in range(d.ncon):
        contact = d.contact[index]
        for position, geom in enumerate(contact.geom):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, geom)
            if name not in names:
                continue
            local = np.zeros(6)
            mujoco.mj_contactForce(m, d, index, local)
            frame = contact.frame.reshape(3, 3)
            sign = 1 if position == 1 else -1
            f = sign * frame.T @ local[:3]
            force += f
            moment += np.cross(contact.pos - origin, f) + sign * frame.T @ local[3:]
    return rotation.T @ force, rotation.T @ moment


def structure_sample(m, d, p, drives, materials):
    """Instantaneous CAD-section stresses from solved internal/contact wrenches."""
    mujoco.mj_rnePostConstraint(m, d)
    factors, demands = {}, {}

    def record(name, demand, capacity):
        demands[name] = float(demand)
        factors[name] = float(capacity / max(abs(demand), 1e-12))

    h, s = p["joint_drive"], p["structure"]
    belt_radius = h["large_teeth"] * h["pitch"] / (2 * math.pi)
    pull = (
        2
        * h["belt_pretension_per_span_N"]
        * math.sqrt(1 - ((belt_radius - belt_radius / h["ratio"]) / h["center_distance"]) ** 2)
    )
    neck_area, _, neck_z, neck_j = hollow_section(h["shaft_min_aluminium_od_m"], 0.006)
    for side, sign in [("left", 1), ("right", -1)]:
        for index in range(2):
            name = f"{side}_crank{index}"
            force, moment = wrench_in_body(m, d, name)
            # Add belt radial pull that torque-only multibody couplings omit.
            parent_rotation = d.xmat[m.body("chassis").id].reshape(3, 3)
            crank_rotation = d.xmat[m.body(name).id].reshape(3, 3)
            belt_pull = crank_rotation.T @ parent_rotation @ [0, 0, pull]
            force = force + belt_pull
            moment = moment + np.cross([0, sign * h["belt_pull_y_m"], 0], belt_pull)
            stations = sign * np.array(
                [h["front_journal_center_y_m"], h["front_journal_center_y_m"] + h["bearing_span_m"]]
            )
            reactions = paired_reactions(force, moment, stations)
            pressure = max(
                reactions["radial_N"][0] / (0.012 * h["journal_width"]),
                reactions["radial_N"][1] / (0.012 * h["rear_journal_width"]),
            )
            record(
                name + "_journals",
                pressure,
                materials["materials"]["iglidur_g"]["pressure_allowable_Pa"],
            )
            thrust = (abs(force[1]) + p["catalog"]["schnorr_008900"]["catalog_force_N"]) / (
                math.pi * (0.012**2 - 0.006**2)
            )
            record(
                name + "_clutch_thrust_face",
                thrust,
                materials["materials"]["iglidur_g"]["pressure_allowable_Pa"],
            )
            rear_moment = moment - np.cross([0, stations[1], 0], force)
            bending = np.linalg.norm(rear_moment[[0, 2]]) / neck_z
            # Preload is bounded by the selected spring catalogforce1359N.
            normal = (
                bending
                + (abs(force[1]) + p["catalog"]["schnorr_008900"]["catalog_force_N"]) / neck_area
            )
            torsion = abs(moment[1]) * h["shaft_min_aluminium_od_m"] / (2 * neck_j)
            record(
                name + "_shaft",
                math.sqrt(normal**2 + 3 * torsion**2),
                allowable(materials, "al7075"),
            )
            web_stress = (
                np.linalg.norm(force)
                * s["fork_root_to_hip_m"]
                / (s["fork_web_thickness"] * s["fork_web_width"] ** 2 / 6)
            )
            record(name + "_cassette_web", 1.5 * web_stress, allowable(materials, "al7075"))
            _, _, cassette_z, _ = hollow_section(0.018, 0.014)
            tube_stress = max(reactions["radial_N"]) * 0.020 / cassette_z
            record(name + "_cassette_tube", 1.5 * tube_stress, allowable(materials, "al7075"))
            # Lightened crank net width14-6mm,3mm thick; rod net12-6mm,4mm.
            # Factor2 local stress concentration accounts for hole/slot ends.
            crank_stress = (
                2 * np.linalg.norm(force) / ((s["crank_width"] - 0.006) * s["crank_thickness"])
            )
            record(name + "_link_net", crank_stress, allowable(materials, "aluminium"))
            rf, _ = wrench_in_body(m, d, f"{side}_rod{index}")
            rod_stress = 2 * np.linalg.norm(rf) / ((s["rod_width"] - 0.006) * s["rod_thickness"])
            record(f"{side}_rod{index}_link_net", rod_stress, allowable(materials, "aluminium"))
            rf, rm = wrench_in_body(m, d, f"{side}_hiprotor{index}")
            rotor_rotation = d.xmat[m.body(f"{side}_hiprotor{index}").id].reshape(3, 3)
            rotor_pull = rotor_rotation.T @ parent_rotation @ [0, 0, -pull]
            rf += rotor_pull
            rm += np.cross([0, sign * h["belt_pull_y_m"], 0], rotor_pull)
            rr = paired_reactions(rf, rm, sign * np.array([0.0175, 0.0235]))
            record(
                f"{side}_hiprotor{index}_6802",
                max(rr["equivalent_static_N"]),
                p["catalog"]["6802"]["C0_N"],
            )
            housing_force, housing_moment = contact_wrench(
                m,
                d,
                {f"{side}_hip{index}_ROHousing"},
                "chassis",
                [(-1 if index == 0 else 1) * p["geometry"]["base"] / 2, sign * 0.028, 0.030],
            )
            group = next(
                r
                for r in materials["fasteners"]["schedule"]
                if r["name"] == "hip_motor_housings_and_encoders"
            )
            for mode, (demand, capacity) in fastener_demands(
                group, housing_force, housing_moment, materials, 1
            ).items():
                record(f"{side}_hip{index}_housing_contact_" + mode, demand, capacity)
        wheel_name = f"{side}_wheel"
        wf, wm = wrench_in_body(m, d, wheel_name)
        drive = p["drive"]
        big_radius = drive["large_teeth"] * drive["pitch"] / (2 * math.pi)
        wheel_pull = (
            2
            * drive["pretension_per_span_N"]
            * math.sqrt(
                1 - ((big_radius - big_radius / h["ratio"]) / drive["center_distance"]) ** 2
            )
        )
        # The belt radial load is a real bearing/fork load even when the
        # multibody belt element represents only its finite angular coupling.
        carrier_rotation = d.xmat[m.body(f"{side}_rod0").id].reshape(3, 3)
        wheel_rotation = d.xmat[m.body(wheel_name).id].reshape(3, 3)
        pull_carrier = (
            wheel_pull
            * np.array([drive["motor_along_rod_from_foot"], 0, drive["motor_perpendicular"]])
            / drive["center_distance"]
        )
        pull_wheel = wheel_rotation.T @ carrier_rotation @ pull_carrier
        wf = wf + pull_wheel
        wm = wm + np.cross(
            [0, sign * (drive["belt_y"] + drive["pulley_face_width"] / 2), 0], pull_wheel
        )
        stations = sign * np.array(
            [v + p["catalog"]["608"]["width"] / 2 for v in p["drive"]["wheel_bearing_y"]]
        )
        reactions = paired_reactions(wf, wm, stations)
        record(
            side + "_output_608", max(reactions["equivalent_static_N"]), p["catalog"]["608"]["C0_N"]
        )
        # Moment at either fork support; maximal magnitude includes lateral
        # tire-edge moment without substituting the tire center for contact.
        support = sign * np.array([sum(s["fork_inner_y"]) / 2, sum(s["fork_outer_y"]) / 2])
        moments = [wm - np.cross([0, station, 0], wf) for station in support]
        area, _, section, polar = hollow_section(s["wheel_axle_od_m"], s["wheel_axle_bore_m"])
        bending = max(np.linalg.norm(v[[0, 2]]) for v in moments) / section
        normal = bending + abs(wf[1]) / area
        torsion = abs(wm[1]) * s["wheel_axle_od_m"] / (2 * polar)
        record(
            side + "_wheel_axle",
            1.5 * math.sqrt(normal**2 + 3 * torsion**2),
            allowable(materials, "steel174_h900"),
        )
        # The lightening slot begins45mm from the axle and halves each12mm
        # arm's net width. The two arms receive signed bearing reactions.
        # Evaluate each separately; axle rigidity does not make the fork solid.
        carrier_force = carrier_rotation.T @ wheel_rotation
        for index, reaction in enumerate(reactions["signed_reactions_N"]):
            arm_force = carrier_force @ reaction
            # The inboard support is the separately retained native cap
            # (CAD localX66..91mm), fixed at proposedX76mm. Its9mm lever
            # does not extend through the main fork's lightening slot.
            net_width = s["fork_arm_width"] if index == 0 else s["fork_arm_width"] - 0.006
            thickness, lever = 0.003, 0.009 if index == 0 else 0.045
            z_in = thickness * net_width**2 / 6
            z_out = net_width * thickness**2 / 6
            stress = 1.5 * (
                abs(arm_force[0]) / (net_width * thickness)
                + abs(arm_force[2]) * lever / z_in
                + reactions["axial_N"] * lever / z_out
            )
            record(f"{side}_wheel_fork_arm_{index}", stress, allowable(materials, "aluminium"))
        # Six9x5mm PA12 hub spokes. Credit only two neighboring spokes for
        # radial load, but one spoke for tangential/lateral bending.
        span = p["geometry"]["wheel_tire_inner"] - s["wheel_hub_radius_m"]
        w, depth = 0.009, 0.005
        radial = np.linalg.norm(wf[[0, 2]]) / 2 / (w * depth)
        tangential = abs(wm[1]) / p["geometry"]["wheel_tire_inner"] * span / (depth * w**2 / 6)
        lateral = abs(wf[1]) * span / (w * depth**2 / 6)
        record(
            side + "_wheel_hub_spokes",
            2 * (radial + tangential + lateral),
            allowable(materials, "pa12"),
        )
        # Positive bearing shoulders/retention and motor saddle fasteners.
        rod_id, wheel_id = m.body(f"{side}_rod0").id, m.body(wheel_name).id
        attachment_world = d.xpos[rod_id] + d.xmat[rod_id].reshape(3, 3) @ [0.076, sign * 0.0475, 0]
        attachment_wheel = d.xmat[wheel_id].reshape(3, 3).T @ (attachment_world - d.xpos[wheel_id])
        cap_moment = wm - np.cross(attachment_wheel, wf)
        for label, force, moment in [
            ("wheel_fork_and_bearing_cap_to_rod", wf, cap_moment),
            ("wheel_motor_saddle_retention", *wrench_in_body(m, d, f"{side}_rotor")),
        ]:
            group = next(r for r in materials["fasteners"]["schedule"] if r["name"] == label)
            axis = 1 if label == "wheel_fork_and_bearing_cap_to_rod" else 2
            for mode, (demand, capacity) in fastener_demands(
                group, force, moment, materials, axis
            ).items():
                record(side + "_" + label + "_" + mode, demand, capacity)
        # The stationary housing can strike the ground directly. Rotor RNE
        # alone would entirely omit that saddle load, since housing is welded
        # into the CAD rod0 body in the multibody model.
        f, moment = contact_wrench(
            m,
            d,
            {side + "_RO40Housing", side + "_RO40HousingMountFoot0"},
            side + "_rod0",
            [0.025, sign * 0.090, -0.004],
        )
        # Belt preload pulls the small motor toward the wheel; its reaction
        # passes through this same saddle even with zero motor current.
        f -= pull_carrier
        motor_position = np.array(
            [
                p["geometry"]["rod"] + drive["motor_along_rod_from_foot"],
                sign * sum(drive["housing_y"]) / 2,
                drive["motor_perpendicular"],
            ]
        )
        moment += np.cross(motor_position - [0.025, sign * 0.090, -0.004], -pull_carrier)
        group = next(
            r
            for r in materials["fasteners"]["schedule"]
            if r["name"] == "wheel_motor_saddle_retention"
        )
        for mode, (demand, capacity) in fastener_demands(group, f, moment, materials, 2).items():
            record(side + "_wheel_motor_saddle_contact_and_preload_" + mode, demand, capacity)
        rf, rm = wrench_in_body(m, d, f"{side}_rotor")
        rotor_rotation = d.xmat[m.body(f"{side}_rotor").id].reshape(3, 3)
        rotor_pull = rotor_rotation.T @ carrier_rotation @ -pull_carrier
        rf += rotor_pull
        rm += np.cross(
            [0, sign * (drive["belt_y"] + drive["pulley_face_width"] / 2), 0], rotor_pull
        )
        rr = paired_reactions(
            rf,
            rm,
            sign
            * np.array([y + p["catalog"]["6802"]["width"] / 2 for y in drive["motor_bearing_y"]]),
        )
        record(side + "_motor_6802", max(rr["equivalent_static_N"]), p["catalog"]["6802"]["C0_N"])
    shell_names = {
        mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
        for g in range(m.ngeom)
        if "Shell" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")
    }
    shell_force, shell_moment = contact_wrench(m, d, shell_names, "chassis", [0.064, 0, 0.020])
    group = next(
        r for r in materials["fasteners"]["schedule"] if r["name"] == "shell_face_and_tray"
    )
    for mode, (demand, capacity) in fastener_demands(
        group, shell_force, shell_moment, materials, 0
    ).items():
        record("shell_attachment_" + mode, demand, capacity)
    # Belt capacity is compared at the actual smaller-pulley speed, including
    # the mandatory length factor omitted by the old gross sizing screen.
    table = materials["belt"]
    speed = abs(d.qvel[drives.rv]) * 60 / (2 * math.pi)
    for i in range(6):
        kind = "hip" if i < 4 else "wheel"
        rating = float(np.interp(speed[i], table["rpm"], table[kind + "_input_rating_lb_in"]))
        rating *= table["pound_inch_Nm"] * table[kind + "_length_factor"]
        input_torque = abs(drives.last_forces[i]) / drives.ratio
        record(f"belt_{i}_torque", input_torque, rating)
        if kind == "hip":
            # Protection must release before the protected stage's factor2
            # rated torque, using the upper calibrated release setting.
            release = max(p["physics"]["clutch"]["selected_range_Nm"])
            record(f"belt_{i}_clutch_release", release / drives.ratio, rating)
        design = h if kind == "hip" else p["drive"]
        rsmall = design["small_teeth"] * design["pitch"] / (2 * math.pi)
        tension = design["pretension_per_span_N"]
        required = 1.21 * input_torque / (2 * rsmall)
        demands[f"belt_{i}_required_installation_tension_N"] = float(required)
        demands[f"belt_{i}_installed_tension_N"] = float(tension)
        demands[f"belt_{i}_rotor_speed_rpm"] = float(speed[i])
        slack = tension - input_torque / (2 * rsmall)
        demands[f"belt_{i}_slack_tension_N"] = float(slack)
        if speed[i] > table["rpm"][-1]:
            factors[f"belt_{i}_speed_domain"] = 0.0
    return factors, demands


def mechanical_energy(m, d, drives):
    # step2 integrates qvel; refresh COM velocities so the per-body rotation
    # and translation decomposition uses that same qvel as kinetic energy.
    mujoco.mj_comVel(m, d)
    mujoco.mj_energyPos(m, d)
    mujoco.mj_energyVel(m, d)
    coordinates = drives._coordinates(d)
    deflection = np.maximum(abs(coordinates) - drives.half, 0)
    stored = float(0.5 * (drives.stiffness * deflection**2).sum())
    rotation, translation = 0.0, 0.0
    velocity = np.zeros(6)
    for body in range(1, m.nbody):
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
        # BODY object velocity is at body COM; inertia frame rotates separately.
        omega = d.ximat[body].reshape(3, 3).T @ velocity[:3]
        rotation += 0.5 * float(m.body_inertia[body] @ (omega**2))
        translation += 0.5 * m.body_mass[body] * float(velocity[3:] @ velocity[3:])
    return {
        "potential_J": float(d.energy[0]),
        "kinetic_J": float(d.energy[1]),
        "body_rotation_J": rotation,
        "body_translation_J": translation,
        "elastic_J": stored,
        "total_J": float(d.energy.sum()) + stored,
    }


def remedy_estimates(p, results, materials, cad_limits):
    """Mass of explicit candidate remedies, not a global redesign optimum."""
    rho = p["materials"]["ld70_upper"]
    al, alloy = (materials["materials"][n]["density_kg_m3"] for n in ["aluminium", "al7075"])
    # Native CAD lightening windows are6mm-wide capsules with26mm endspacing.
    slot_volume = (0.006 * 0.026 + math.pi * 0.003**2) * 0.003
    components = json.loads((ROOT / "artifacts/milestone-9/cad/components.json").read_text())
    fork_mass = sum(r["mass_kg"] for r in components if "WheelFork" in r["name"])
    density_correction = fork_mass * (alloy / al - 1)
    foam = materials["materials"]["ld70_upper"]
    energy_density = float(np.trapezoid(foam["compression_stress_Pa"], foam["compression_strain"]))
    # The front/back pair is a candidate; each direction loads a separate pad.
    pair_volume = 2 * 0.020 * 0.060 * 0.020
    pair_mass = pair_volume * rho
    bonds = (
        2
        * 0.020
        * 0.060
        * p["structure"]["tire_construction"]["bond_film_m"]
        * p["materials"]["acrylic_adhesive_estimate"]
    )
    fork_remedy = density_correction + 2 * slot_volume * alloy
    ceiling_margin = cad_limits["allocated_mass_ceiling_kg"] - cad_limits["allocated_mass_kg"]
    hub_factor = min(
        v
        for r in results.values()
        for k, v in r["minimum_factors"].items()
        if "wheel_hub_spokes" in k
    )
    new_depth = 0.005 * math.sqrt(materials["required_factor"] / hub_factor)
    spoke_extra = (
        2
        * 6
        * 0.009
        * (p["geometry"]["wheel_tire_inner"] - p["structure"]["wheel_hub_radius_m"])
        * (new_depth - 0.005)
        * materials["materials"]["pa12"]["density_kg_m3"]
    )
    return {
        "scope": "Explicit candidate estimates under measured current loads; not a proven globally minimum redesign. Bumpers must bypass weak covers into positive retained supports and require new contact/CAD acceptance.",
        "foam_energy_density_to_50percent_J_m3": energy_density,
        "front_back_pair_mass_kg": pair_mass,
        "bond_film_mass_kg": bonds,
        "single_foam_pad_storage_to_50percent_J": pair_volume / 2 * energy_density,
        "outer_fork_slot_fill_mass_kg_6061": 2 * slot_volume * al,
        "existing_fork_7075_density_correction_kg": density_correction,
        "7075_fork_and_outer_slot_remedy_mass_kg": fork_remedy,
        "fork_and_front_back_bumper_mass_kg": fork_remedy + pair_mass + bonds,
        "current_allocated_mass_margin_kg": ceiling_margin,
        "hub_spoke_depth_optimistic_same_load_m": new_depth,
        "hub_spoke_extra_mass_optimistic_same_load_kg": spoke_extra,
        "hub_estimate_scope": "Optimistic depth-only fix assumes every stress term falls asdepth^-2; terms fallingasdepth^-1 make actualrequireddepthhigher. Changedimpactloads could insteadreduce requiredreinforcement.",
    }


def closure_engineering(shared, materials=None):
    """Reproducible unadopted sizing; never turns an estimate into CAD mass."""
    materials = materials or load_materials()
    proposal = shared["m9_closure"]
    t = proposal["thermal_candidate"]
    width, height, thickness = t["plate_size_m"]
    area = (
        math.pi
        * (t["stator_face_outer_radius_m"] ** 2 - t["stator_face_inner_radius_m"] ** 2)
        * t["contact_fraction"]
    )
    enamel = t["enamel_thickness_m"] / (t["enamel_conductivity_W_m_K_assumption"] * area)
    interface = t["potting_thickness_m"] / (t["potting_conductivity_W_m_K"] * area)
    cap = math.log(0.02465 / t["stator_face_outer_radius_m"]) / (
        2 * math.pi * t["al_conductivity_W_m_K"] * t["rear_cap_thickness_m"]
    )
    ribs = t["spoke_length_m"] / (
        t["al_conductivity_W_m_K"] * t["spokes"] * t["spoke_width_m"] * t["spoke_thickness_m"]
    )
    surface, air = t["surface_C_for_radiation"] + 273.15, t["air_C"] + 273.15
    radiation = (
        t["emissivity_assumption"]
        * t["view_factor_assumption"]
        * 5.670374419e-8
        * (surface**2 + air**2)
        * (surface + air)
    )
    h = t["convection_W_m2_K"] + radiation
    fin_m = math.sqrt(2 * h / (t["al_conductivity_W_m_K"] * thickness))
    fin_length = max(width, height) / 2
    fin_efficiency = math.tanh(fin_m * fin_length) / (fin_m * fin_length)
    effective = (
        2 * (width * height - t["plate_open_area_m2"]) * fin_efficiency
        + t["housing_area_credit_m2"]
    )
    air_path = 1 / (h * effective)
    thermal_mass = (
        6 * (width * height - t["plate_open_area_m2"]) * thickness * t["al_density_kg_m3"]
        + 6 * area * t["potting_thickness_m"] * t["potting_density_kg_m3"]
        + t["extra_rib_mass_kg"]
        + t["mounts_mass_kg_allowance"]
    )
    foam_mass = sum(
        np.prod(pad["size_m"]) * materials["materials"]["ld45_upper"]["density_kg_m3"]
        for pad in proposal["candidate_bumpers"]
    )
    old_factor = 0.34446347284501977
    # Six independent6mm lanes give an optimistic width-capacity multiplier6.
    # This is NOT a catalog-qualified36mm endless belt or load-sharing design.
    # Stock12mm gives multiplier2 and still fails. Its mass lower bound alone
    # screens the evaluated width-only option before attempting geometry.
    components = json.loads((ROOT / "artifacts/milestone-9/cad/components.json").read_text())
    current_belt_mass = sum(c["mass_kg"] for c in components if c["name"].endswith("_JointBelt"))
    belt_extra = 5 * current_belt_mass
    joint = shared["m9_cad"]["parameters"]["joint_drive"]
    # Ignore hubs/flanges and pocket floors: only four added tooth-stock rings.
    outer = joint["large_teeth"] * joint["pitch"] / (2 * math.pi) - 0.000254
    inner = outer - joint["large_pulley_radial_stock_m"]
    pulley_extra_lower = (
        4
        * math.pi
        * (outer**2 - inner**2)
        * (0.036 - 0.006)
        * materials["materials"]["aluminium"]["density_kg_m3"]
    )
    lower_total = foam_mass + thermal_mass + belt_extra + pulley_extra_lower
    allocation = shared["m9_cad"]["allocated_mass_kg"]
    # Give even the unadopted hardware ledger its full possible credit for this
    # best-case budget comparison. It cannot rescue the width-only option.
    possible_credit = hardware_ledger(materials)["allocation_margin_kg"]
    return {
        "status": "Sizing only; no geometry, mass or tensor adoption. A lighter isolated-input clutch is an unqualified alternative, not ruled out by this bound.",
        "thermal": {
            "contact_area_m2": area,
            "enamel_K_W": enamel,
            "interface_K_W": interface,
            "cap_K_W": cap,
            "ribs_K_W": ribs,
            "contact_spreading_K_W": t["contact_spreading_K_W_assumption"],
            "convection_W_m2_K": t["convection_W_m2_K"],
            "radiation_W_m2_K": radiation,
            "fin_efficiency": fin_efficiency,
            "effective_air_area_m2": effective,
            "air_K_W": air_path,
            "winding_to_air_K_W": enamel
            + interface
            + cap
            + ribs
            + t["contact_spreading_K_W_assumption"]
            + air_path,
            "mass_kg": float(thermal_mass),
            "conditions": t["conditions"],
            "hardware_qualified": False,
        },
        "bumper_foam_mass_kg": float(foam_mass),
        "width_only_belt_option": {
            "existing_conservative_factor": old_factor,
            "stock_12mm_factor": 2 * old_factor,
            "optimistic_36mm_factor": 6 * old_factor,
            "extra_belt_mass_kg": belt_extra,
            "extra_four_large_tooth_stock_rings_mass_lower_bound_kg": pulley_extra_lower,
            "foam_thermal_belt_pulley_subtotal_lower_bound_kg": float(lower_total),
            "excludes": "Small pulleys, shaft/journal/fork/hub/axle reinforcement, mounts/retention/containment and increased pretension/alignment loads; not a complete passing remedy.",
            "possible_hardware_credit_kg": possible_credit,
            "best_case_allocated_mass_lower_bound_kg": float(
                allocation + lower_total - possible_credit
            ),
            "excess_over_authorized_1850g_lower_bound_kg": float(
                allocation + lower_total - possible_credit - 1.85
            ),
            "required_reserve_draw_lower_bound_kg": float(
                allocation + lower_total - possible_credit - 1.8
            ),
        },
    }


def completion_reinforcement_budget(shared, results, materials=None):
    """Budget one explicit fixed-load shaft remedy before CAD adoption.

    This is an optimistic lower bound for a symmetric, bored H900 replacement
    of the existing four journals. It is not a minimum over other load paths:
    changing the impact/contact architecture can change the demands entirely.
    Credit every measured VM stress as bending/torsion (section modulus); axial
    stress falls more slowly with diameter, so actual sizing can only be larger
    at these fixed loads. Bearings, enlarged clutch and mounts are excluded.
    """
    materials = materials or load_materials()
    results = {
        name: row
        for name, row in results.items()
        if row.get("fall_classification") == "passive_design"
    }
    if not results:
        raise ValueError("Shaft budget requires at least one classified design fall")
    h = shared["m9_cad"]["parameters"]["joint_drive"]
    components = json.loads((ROOT / "artifacts/milestone-9/cad/components.json").read_text())
    credit = sum(
        c["mass_kg"]
        for c in components
        if c["name"].endswith("_Journal") or "_SteelSleeve" in c["name"]
    )
    original_od, bore, length = h["shaft_min_aluminium_od_m"], 0.006, 0.0502 - 0.0091
    old_section = hollow_section(original_od, bore)[2]
    rows = []
    for index in range(2):
        demand, governing = max(
            (r["peak_demands"][f"{side}_crank{index}_shaft"], name)
            for name, r in results.items()
            for side in ("left", "right")
        )
        required_section = (
            old_section
            * materials["required_factor"]
            * demand
            / allowable(materials, "steel174_h900")
        )
        lo, hi = original_od, 0.1
        for _ in range(80):
            mid = (lo + hi) / 2
            if hollow_section(mid, bore)[2] < required_section:
                lo = mid
            else:
                hi = mid
        mass = (
            2
            * hollow_section(hi, bore)[0]
            * length
            * materials["materials"]["steel174_h900"]["density_kg_m3"]
        )
        rows.append(
            {
                "hip_index": index,
                "governing_case": governing,
                "fixed_load_original_VM_Pa": demand,
                "minimum_OD_m": hi,
                "pair_mass_lower_bound_kg": mass,
            }
        )
    shaft_mass = sum(r["pair_mass_lower_bound_kg"] for r in rows)
    allocated = shared["m9_cad"]["allocated_mass_kg"]
    limit = shared["m9_cad"]["allocated_mass_ceiling_kg"]
    foam = closure_engineering(shared, materials)["bumper_foam_mass_kg"]
    heat = closure_engineering(shared, materials)["thermal"]["mass_kg"]
    candidate = allocated - credit + shaft_mass + foam + heat
    return {
        "status": "STOP before CAD adoption"
        if candidate > limit
        else "Incomplete sizing; continue full load-path design",
        "scope": "Fixed measured loads, symmetric H900 bored-shaft remedy only; not a global mass minimum or a passing structural design",
        "shaft_rows": rows,
        "original_journal_and_sleeve_credit_kg": credit,
        "allocated_with_shaft_remedy_lower_bound_kg": allocated - credit + shaft_mass,
        "foam_kg": foam,
        "thermal_kg": heat,
        "candidate_allocated_lower_bound_kg": candidate,
        "allocated_limit_kg": limit,
        "excess_lower_bound_kg": max(0, candidate - limit),
        "excluded": "Larger journals, cassette/clutch/shaft interfaces, forks/hubs/axles, belt stages and tooth-jump/alignment remedy, all bumper mounts, retention and fairings",
        "alternatives": [
            "Change the frame impact load path and arrest continuing downhill rolling, then remeasure forces",
            "Design a qualified lower-inertia isolated belt input and impact bypass",
            "Authorize a larger allocation for this fixed-load reinforcement option",
        ],
    }


def run_case(
    shared,
    direction="forward",
    speed=0.0,
    output=None,
    stiffness_scale=1.0,
    restitution=0.3,
    duration=3.0,
    dt=None,
    backing_stiffness_scale=1.0,
    powered=None,
    candidate_bumpers=False,
    library=None,
):
    from .m9_kinematics import leg_fk
    from .m9_plant import (
        WHEELS,
        MechanicalDrives,
        _refit_world_bvh,
        build,
        generate_xml,
        parameters,
        reset,
        truth,
    )

    p, materials = parameters(shared), load_materials()
    dt = p["simulation"]["physics_dt"] if dt is None else dt
    p["simulation"]["physics_dt"] = dt
    if candidate_bumpers:
        from .mass_properties import combine

        pads = shared["m9_closure"]["candidate_bumpers"]
        p["candidate_bumpers"] = copy.deepcopy(pads)
        pieces = []
        for pad in pads:
            size = np.array(pad["size_m"])
            mass = float(np.prod(size) * materials["materials"]["ld45_upper"]["density_kg_m3"])
            pieces.append(
                {
                    "mass_kg": mass,
                    "com_m": pad["center_m"],
                    "inertia_kg_m2": np.diag(mass / 12 * (sum(size**2) - size**2)).tolist(),
                }
            )
        foam_inertials = (
            [p["candidate_foam_inertial"]] if "candidate_foam_inertial" in p else pieces
        )
        p["rigid_bodies"]["chassis"] = combine([p["rigid_bodies"]["chassis"], *foam_inertials])
        root = ET.fromstring(generate_xml(p))
        base = root.find(".//body[@name='chassis']")
        for pad in pads:
            ET.SubElement(
                base,
                "geom",
                name=pad["name"],
                type="box",
                mass="0",
                pos=" ".join(map(str, pad["center_m"])),
                size=" ".join(str(x / 2) for x in pad["size_m"]),
                rgba=".2 .2 .2 1",
            )
        m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        d = mujoco.MjData(m)
    else:
        m, d = build(p)
    reset(m, d, p, angle_deg=40)
    # At-rest release from a tilted nominal-length mechanism, no initial spin.
    # Roll32deg puts the COM beyond the tire-edge support. Pitch10deg seeds
    # forward/backward tipping of the wheel-supported free base.
    angle = (
        0 if powered else math.radians({"forward": 10, "backward": -10, "sideways": 32}[direction])
    )
    axis = np.array([1, 0, 0]) if direction == "sideways" else np.array([0, 1, 0])
    tilt = np.r_[math.cos(angle / 2), axis * math.sin(angle / 2)]
    quat = np.zeros(4)
    mujoco.mju_mulQuat(quat, tilt, d.qpos[3:7].copy())
    d.qpos[3:7] = quat
    mujoco.mj_forward(m, d)
    ground = m.geom("ground").id
    cross_slope = math.radians(powered.get("cross_slope_deg", 0)) if powered else 0
    if cross_slope:
        m.geom_quat[ground] = [math.cos(cross_slope / 2), math.sin(cross_slope / 2), 0, 0]
        _refit_world_bvh(m, ground)
        mujoco.mj_forward(m, d)
    distance = np.zeros(6)
    # Establish the lowest tire surface on the plane as an initial condition.
    # Robot coordinates are never subsequently projected or clamped.
    depths = [
        mujoco.mj_geomDistance(m, d, ground, m.geom(side + "_tire").id, 1.0, distance)
        for side in ["left", "right"]
    ]
    d.qpos[2] -= min(depths) / math.cos(cross_slope)
    d.qvel[0] = speed
    for name in WHEELS:
        d.qvel[m.joint(name).dofadr[0]] = speed / p["geometry"]["wheel_radius"]
    for side in ["left", "right"]:
        d.qvel[m.joint(side + "_rotor_joint").dofadr[0]] = 4 * speed / p["geometry"]["wheel_radius"]
    mujoco.mj_forward(m, d)
    drives = MechanicalDrives(m, p)
    contact_model = MaterialContacts(
        p, materials, stiffness_scale, restitution, backing_stiffness_scale=backing_stiffness_scale
    )
    contact_model.configure(m)
    drives.contacts = contact_model
    requested = np.zeros(6)
    response = {
        "detected_s": None,
        "reason": None,
        "command_off_s": None,
        "current_below_0p01_A_s": None,
        "first_non_tire_contact_s": None,
        "first_non_tire_contact_energy": None,
        "first_non_tire_contact_names": [],
        "crouch_reached_s": None,
        "minimum_mechanism_height_before_contact_m": None,
        "encoder_or_health_fault": None,
    }
    control_trace = []
    if powered:
        from .firmware_control import FirmwareController, FirmwareEstimator, FirmwareLocalFeedback
        from .m9_kinematics import posture
        from .robustness_control import build_candidate

        gains = json.loads((ROOT / "config/controller_m9.json").read_text())
        nominal = parameters(shared)
        # The controller uses the adopted nominal, including for the explicitly
        # unadopted bumper mass experiment. It never reads physical truth.
        library = library or build_candidate(nominal, gains, ROOT / ".local-tools/m9-impact")
        sensors = json.loads((ROOT / "config/sensors-m9.json").read_text())
        drives.forward(m, d)
        estimator = FirmwareEstimator(m, d, nominal, sensors, sensors["seed"], library)
        local = FirmwareLocalFeedback(d, sensors, library)
        controller = FirmwareController(nominal, gains, library)
        qref, qdref = posture(40, nominal), np.zeros(4)
        estimate = estimator.update(d)
        host_steps, servo_steps = round(0.005 / dt), round(0.001 / dt)
        if not math.isclose(host_steps * dt, 0.005) or not math.isclose(servo_steps * dt, 0.001):
            raise ValueError("Fall timestep must divide host and local sample periods")
    initial = mechanical_energy(m, d, drives)
    minimum, peak_demands = {}, {}
    peak_contacts, impulse_contacts = {}, {}
    numerical_peak = initial["total_J"]
    peak_kinetic, peak_rotation = initial["kinetic_J"], initial["body_rotation_J"]
    trace = []
    endpoint_peak = np.zeros(6)
    endpoint_residual = np.zeros(6)
    motor_work = 0.0
    crouch_exposure = 0.0
    load_trace = []
    for step in range(round(duration / dt)):
        if powered:
            drives.forward(m, d)
            estimator.preprocess(d, dt)
            if step % host_steps == 0:
                try:
                    estimate = estimator.update(d)
                    requested, _, _, _ = controller.command(estimate, 0, 0, qref, qdref, 0.005)
                except ValueError as exc:
                    requested = controller.protect(estimate, 0.005, valid=False)
                    if response["encoder_or_health_fault"] is None:
                        response["encoder_or_health_fault"] = {
                            "time_s": float(d.time),
                            "error": str(exc),
                        }
                if controller.protection.latched and response["detected_s"] is None:
                    from .protective_fall import REASONS

                    response["detected_s"] = float(d.time)
                    response["reason"] = REASONS[controller.protection.reason]
            if step % servo_steps == 0:
                try:
                    controller.gate(estimate, d.time)
                    q, qd = local.update(d, 0.001)
                    requested[:4] = controller.leg_torques(
                        {"q": q, "qd": qd, "pitch": estimate["pitch"], "roll": estimate["roll"]},
                        qref,
                        qdref,
                        controller.feedforward,
                    )
                except ValueError as exc:
                    requested[:] = 0
                    controller.protect(estimate, 0.005, valid=False)
                    if response["encoder_or_health_fault"] is None:
                        response["encoder_or_health_fault"] = {
                            "time_s": float(d.time),
                            "error": str(exc),
                        }
                if not controller.supervisor.torque_enabled:
                    requested[:] = 0
            if controller.protection.torque_off and response["command_off_s"] is None:
                response["command_off_s"] = float(d.time)
            d.xfrc_applied[:] = 0
            if 2 - 1e-9 <= d.time < 2.1 - 1e-9:
                d.xfrc_applied[m.body("chassis").id, 0] = powered.get("push_Ns", 0) / 0.1
            if step % round(0.01 / dt) == 0:
                control_trace.append(
                    [
                        d.time,
                        estimate["pitch"],
                        estimate["pitch_rate"],
                        estimate["roll"],
                        estimate["accel_norm"],
                        *requested,
                        *controller.protection.qref,
                        controller.protection.latched,
                        controller.protection.torque_off,
                    ]
                )
        drives.apply(m, d, requested, contact_callback=contact_model)
        motor_work += float(drives.motor_torque @ d.qvel[drives.rv]) * dt
        mujoco.mj_step1(m, d)
        contact_model(m, d)
        mujoco.mj_step2(m, d)
        drives.observe_after_step(m, d)
        gap, relative = drives._coordinates(d), drives.J @ d.qvel
        law = drives.stiffness * np.sign(gap) * np.maximum(
            abs(gap) - drives.half, 0
        ) + drives.damping * relative * (abs(gap) > drives.half)
        endpoint_peak = np.maximum(endpoint_peak, abs(law[:6]))
        endpoint_residual = np.maximum(endpoint_residual, abs(law[:6] - drives.last_forces[:6]))
        if (
            powered
            and response["command_off_s"] is not None
            and response["current_below_0p01_A_s"] is None
            and max(abs(drives.phase_current)) <= 0.01
        ):
            response["current_below_0p01_A_s"] = float(d.time)
        # RNE follows the physical constraint solve; force peaks sampled at2kHz.
        factors, demands = structure_sample(m, d, p, drives, materials)
        mechanism_height = -float(
            np.mean([leg_fk(d.qpos[drives.oq[:4]][j : j + 2], p)[1] for j in (0, 2)])
        )
        if response["first_non_tire_contact_s"] is None:
            old = response["minimum_mechanism_height_before_contact_m"]
            response["minimum_mechanism_height_before_contact_m"] = min(
                mechanism_height, old if old is not None else math.inf
            )
        if powered and controller.protection.latched:
            crouch_exposure = (
                crouch_exposure + dt
                if mechanism_height <= p["geometry"]["crouched_height"] + 0.005
                else 0.0
            )
            if crouch_exposure >= 0.02 and response["crouch_reached_s"] is None:
                response["crouch_reached_s"] = float(d.time) - 0.02
        for name, factor in factors.items():
            minimum[name] = min(minimum.get(name, math.inf), factor)
        for name, demand in demands.items():
            peak_demands[name] = max(peak_demands.get(name, 0), abs(demand))
        normal_total = 0.0
        instantaneous = {}
        for index in range(d.ncon):
            contact = d.contact[index]
            if ground not in contact.geom:
                continue
            gid = contact.geom[1] if contact.geom[0] == ground else contact.geom[0]
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gid)
            force = np.zeros(6)
            mujoco.mj_contactForce(m, d, index, force)
            normal = abs(float(force[0]))
            if "tire" not in name and normal > 1e-6:
                if response["first_non_tire_contact_s"] is None:
                    response["first_non_tire_contact_s"] = float(d.time)
                    response["first_non_tire_contact_energy"] = mechanical_energy(m, d, drives)
                if (
                    abs(d.time - response["first_non_tire_contact_s"]) <= dt / 2
                    and name not in response["first_non_tire_contact_names"]
                ):
                    response["first_non_tire_contact_names"].append(name)
            normal_total += normal
            instantaneous[name] = instantaneous.get(name, 0) + normal
            impulse_contacts[name] = impulse_contacts.get(name, 0) + normal * dt
        for name, normal in instantaneous.items():
            peak_contacts[name] = max(peak_contacts.get(name, 0), normal)
        energy = mechanical_energy(m, d, drives)
        peak_kinetic = max(peak_kinetic, energy["kinetic_J"])
        peak_rotation = max(peak_rotation, energy["body_rotation_J"])
        numerical_peak = max(numerical_peak, energy["total_J"] + drives.plastic_energy)
        if step % max(1, round(0.01 / dt)) == 0:
            load_trace.append((float(d.time), mechanism_height, factors.copy()))
            state = truth(m, d, p)
            trace.append(
                [
                    d.time,
                    state["pitch"],
                    state["roll"],
                    d.subtree_com[1, 2],
                    energy["potential_J"],
                    energy["kinetic_J"],
                    energy["body_rotation_J"],
                    energy["elastic_J"],
                    drives.plastic_energy,
                    normal_total,
                    min(factors.values()),
                ]
            )
    final = mechanical_energy(m, d, drives)
    # Shell/cover stress uses the actual per-component resultant contact force;
    # impulse or historical3J does not replace measured peak force.
    for name, force in peak_contacts.items():
        category = contact_model.category(name)
        if category == "shell":
            minimum[name + "_printed_strength"] = contact_model.shell["capacity_N"] / max(
                force, 1e-12
            )
        elif category == "cover":
            c = materials["contacts"]
            stress = (
                1.5
                * force
                * c["cover_beam_span_m"]
                / (c["cover_effective_width_m"] * p["structure"]["cover_wall"] ** 2)
            )
            minimum[name + "_printed_strength"] = allowable(materials, "pa12") / max(stress, 1e-12)
        elif category == "pad":
            minimum[name + "_TPU_strength"] = (
                allowable(materials, "tpu") * 0.015 * 0.020 / max(force, 1e-12)
            )
    structural_failures = [
        name
        for name, factor in minimum.items()
        if factor < materials["required_factor"]
        and "printed_strength" not in name
        and "_TPU_strength" not in name
    ]
    # The amended gate exempts replaceable skins from factor2, but retention
    # and containment still require actual adopted frame attachment features.
    containment_failures = ["retention_and_cosmetic_containment_not_adopted"]
    for i in range(6):
        if (
            peak_demands[f"belt_{i}_required_installation_tension_N"]
            > peak_demands[f"belt_{i}_installed_tension_N"]
        ):
            structural_failures.append(f"belt_{i}_installation_tension")
    for name, depth in contact_model.peak_depth.items():
        category = contact_model.category(name)
        limit = contact_model.shell["maximum_travel_m"] if category == "shell" else math.inf
        if depth > limit:
            containment_failures.append(name + "_stroke")
    injected = numerical_peak - initial["total_J"]
    if not powered and injected > max(
        0.03, 0.05 * max(initial["kinetic_J"] + initial["elastic_J"], 0.1)
    ):
        structural_failures.append("passive_energy_increase")
    # The implicit belt solve is not timestep-converged at impact. Do not
    # certify using its smaller applied-force peak. Combine independent
    # endpoint demand/speed extrema with the previously measured uncertainty
    # bound; this conservative screen is deliberately not a resolved force.
    envelope = shared.get("m9_closure", {}).get("belt_load_uncertainty", {})
    table = materials["belt"]
    conservative = []
    for i in range(6):
        kind = "hip" if i < 4 else "wheel"
        demand = max(
            endpoint_peak[i],
            drives.peak_belt_torque[i],
            envelope.get(kind + "_output_demand_Nm", 0),
        )
        rpm = max(
            peak_demands[f"belt_{i}_rotor_speed_rpm"], envelope.get(kind + "_rotor_speed_rpm", 0)
        )
        rating = (
            float(np.interp(rpm, table["rpm"], table[kind + "_input_rating_lb_in"]))
            * table["pound_inch_Nm"]
            * table[kind + "_length_factor"]
        )
        design = p["joint_drive"] if i < 4 else p["drive"]
        small_radius = design["small_teeth"] * design["pitch"] / (2 * math.pi)
        required = 1.21 * (demand / drives.ratio) / (2 * small_radius)
        tension = design["pretension_per_span_N"]
        factor = rating / max(demand / drives.ratio, 1e-12) if rpm <= table["rpm"][-1] else 0.0
        conservative.append(
            {
                "output_demand_Nm": float(demand),
                "rotor_rpm": float(rpm),
                "factor": float(factor),
                "rating_input_Nm": rating,
                "required_tension_N": float(required),
                "installed_tension_N": tension,
                "slack_tension_N": float(tension - demand / drives.ratio / (2 * small_radius)),
            }
        )
        if factor < 2 or required > tension:
            structural_failures.append(f"belt_{i}_conservative_uncertainty")
    # Bumper-first amendment: factor2 is required only for passive design
    # tips. Powered falls retain every load-path diagnostic for information,
    # while acceptance depends on containment (still unadopted here).
    failures = containment_failures + ([] if powered else structural_failures)
    result = {
        "direction": direction,
        "speed_m_s": speed,
        "duration_s": duration,
        "physics_dt_s": dt,
        "trace_dt_s": 0.01,
        "powered_case": powered,
        "candidate_bumpers": candidate_bumpers,
        "protective_response": response if powered else None,
        "contact_response": response,
        "fall_classification": (
            "passive_design"
            if not powered
            else "no_body_contact"
            if response["first_non_tire_contact_energy"] is None
            else "extreme_containment"
        ),
        "protective_posture_demonstrated": bool(
            powered
            and response["first_non_tire_contact_s"] is not None
            and (
                (
                    response["crouch_reached_s"] is not None
                    and response["crouch_reached_s"] < response["first_non_tire_contact_s"]
                )
                or all(
                    n.startswith(("left_", "right_"))
                    and any(
                        token in n
                        for token in ("Crank", "Rod", "WheelFork", "LegImpactPad", "RO40Housing")
                    )
                    for n in response["first_non_tire_contact_names"]
                )
            )
        ),
        "motor_work_J": motor_work,
        "endpoint_belt_demand_peak_Nm": endpoint_peak.tolist(),
        "endpoint_belt_residual_peak_Nm": endpoint_residual.tolist(),
        "conservative_belt_screen": conservative,
        "shell_backing_stiffness_scale": backing_stiffness_scale,
        "initial_tilt_deg": math.degrees(angle),
        "initial_energy": initial,
        "final_energy": final,
        "maximum_passive_energy_increase_J": injected,
        "peak_kinetic_energy_J": peak_kinetic,
        "peak_rotation_energy_J": peak_rotation,
        "peak_contact_force_N": peak_contacts,
        "contact_impulse_Ns": impulse_contacts,
        "peak_contact_depth_m": contact_model.peak_depth,
        "contact_effective_mass_range_kg": contact_model.effective_mass_range,
        "minimum_factors": minimum,
        "cosmetic_factors": {
            k: v for k, v in minimum.items() if "printed_strength" in k or "_TPU_strength" in k
        },
        "structural_factors": {
            k: v
            for k, v in minimum.items()
            if "printed_strength" not in k and "_TPU_strength" not in k
        },
        "peak_demands": peak_demands,
        "structural_factor_gate_applies": not bool(powered),
        "informational_structural_failures": structural_failures if powered else [],
        "drives": drives.metrics(),
        "failures": failures,
        "retention_adopted": False,
        "passed": not failures,
        "limitations": "Conditional actual CAD inertia and exterior hull fall with stated constitutive and strength assumptions. Retention schedule is proposed until CAD fastener and attachment adoption. No hardware validation. Filled concavities in convex collision hulls may change contact sequence.",
    }
    if output is not None:
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        (output / "metrics.json").write_text(json.dumps(result, indent=2) + "\n")
        with (output / "loads.csv").open("w") as handle:
            writer = csv.writer(handle)
            names = sorted({name for _, _, row in load_trace for name in row})
            writer.writerow(["time_s", "mechanism_height_m", *names])
            writer.writerows([t, h, *[row.get(n, "") for n in names]] for t, h, row in load_trace)
        if powered:
            with (output / "control.csv").open("w") as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    [
                        "time_s",
                        "estimated_pitch_rad",
                        "pitch_rate_rad_s",
                        "roll_rad",
                        "accel_norm",
                        "hip0L_Nm",
                        "hip1L_Nm",
                        "hip0R_Nm",
                        "hip1R_Nm",
                        "wheelL_Nm",
                        "wheelR_Nm",
                        "q0L_ref",
                        "q1L_ref",
                        "q0R_ref",
                        "q1R_ref",
                        "latched",
                        "off",
                    ]
                )
                writer.writerows(control_trace)
        with (output / "trace.csv").open("w") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "time_s",
                    "pitch_rad",
                    "roll_rad",
                    "COM_z_m",
                    "potential_J",
                    "kinetic_J",
                    "rotation_J",
                    "elastic_J",
                    "clutch_dissipated_J",
                    "ground_normal_force_N",
                    "minimum_factor",
                ]
            )
            writer.writerows(trace)
    return result


def main():
    from .m9_plant import parameters

    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="artifacts/milestone-9/impact")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument(
        "--dt", type=float, help="Physics timestep; defaults to the shared M9 plant"
    )
    parser.add_argument("--stiffness-scale", type=float, default=1.0)
    parser.add_argument("--restitution", type=float, default=0.3)
    parser.add_argument("--shell-backing-stiffness-scale", type=float, default=1.0)
    parser.add_argument(
        "--direction", choices=["forward", "backward", "sideways", "all"], default="all"
    )
    parser.add_argument("--speed", type=float)
    parser.add_argument(
        "--powered",
        action="store_true",
        help="Add sensor/C controlled over-limit pushes and a physical cross-slope",
    )
    parser.add_argument(
        "--candidate-bumpers",
        action="store_true",
        help="Unadopted frame-bumper sizing experiment; not CAD acceptance",
    )
    args = parser.parse_args()
    output = ROOT / args.output
    output.mkdir(parents=True, exist_ok=True)
    shared = json.loads((ROOT / "config/robot.json").read_text())
    materials = load_materials()
    m9_parameters = parameters(shared)
    if args.dt is None:
        args.dt = m9_parameters["simulation"]["physics_dt"]
    configuration = {
        "robot": shared,
        "m9_parameters": m9_parameters,
        "materials": materials,
        "experiment": vars(args),
        "passive_behavior": "Unpoweredopenphase,zero commanded current; finite native rotor/belt inertia,plain-journal/bearing dryfriction,elastic belt lash and hip slip couplings",
    }
    if args.powered:
        configuration["gains"] = json.loads((ROOT / "config/controller_m9.json").read_text())
        configuration["sensors"] = json.loads((ROOT / "config/sensors-m9.json").read_text())
    (output / "configuration.json").write_text(json.dumps(configuration, indent=2) + "\n")
    cases = {}
    directions = (
        ["forward", "backward", "sideways"] if args.direction == "all" else [args.direction]
    )
    speeds = [0.0, 0.3] if args.speed is None else [args.speed]
    for direction in directions:
        for speed in speeds:
            name = direction + ("_rest" if speed == 0 else "_0p3")
            cases[name] = run_case(
                shared,
                direction,
                speed,
                output / name,
                args.stiffness_scale,
                args.restitution,
                args.duration,
                args.dt,
                args.shell_backing_stiffness_scale,
                candidate_bumpers=args.candidate_bumpers,
            )
            print(
                json.dumps(
                    {
                        "case": name,
                        "passed": cases[name]["passed"],
                        "minimum_factor": min(cases[name]["minimum_factors"].values()),
                        "failures": cases[name]["failures"],
                    }
                ),
                flush=True,
            )
    if args.powered:
        from .robustness_control import build_candidate

        gains = json.loads((ROOT / "config/controller_m9.json").read_text())
        library = build_candidate(m9_parameters, gains, ROOT / ".local-tools/m9-impact")
        for name, case in [
            ("powered_push_plus6", {"push_Ns": 6}),
            ("powered_push_minus6", {"push_Ns": -6}),
            ("powered_cross_slope40", {"cross_slope_deg": 40}),
        ]:
            cases[name] = run_case(
                shared,
                output=output / name,
                stiffness_scale=args.stiffness_scale,
                restitution=args.restitution,
                duration=max(args.duration, 6),
                dt=args.dt,
                backing_stiffness_scale=args.shell_backing_stiffness_scale,
                powered=case,
                candidate_bumpers=args.candidate_bumpers,
                library=library,
            )
            print(
                json.dumps(
                    {
                        "case": name,
                        "response": cases[name]["protective_response"],
                        "passed": cases[name]["passed"],
                    }
                ),
                flush=True,
            )
    result = {
        "cases": cases,
        "hardware_ledger": hardware_ledger(materials),
        "candidate_remedies": remedy_estimates(
            configuration["m9_parameters"], cases, materials, shared["m9_cad"]
        ),
        "passed": all(row["passed"] for row in cases.values()),
    }
    if "m9_closure" in shared:
        (output / "closure-engineering.json").write_text(
            json.dumps(closure_engineering(shared, materials), indent=2) + "\n"
        )
    (output / "acceptance.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
