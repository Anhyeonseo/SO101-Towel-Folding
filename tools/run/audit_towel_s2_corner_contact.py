#!/usr/bin/env python3
"""Read-only S2 contact audit; never imports Isaac or robot execution APIs.

Compare the planner and authored pad frames, then test ONE alternative:
pinch the raised S1 crease corner of the upper S2 bundle.  A bounded multiseed
IK solve is numerical branch discovery, not repeated cloth trials.  Rejected
geometry must not be converted into an execution replay.
"""
from __future__ import annotations

import argparse
import ast
from hashlib import sha256
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.grasp_yaw_kinematics import (
    FIXED_JAW_PAD_NORMAL_IN_GRIPPER,
    GraspYawKinematics,
    _rpy_matrix,
)


def unit(vector):
    value = np.asarray(vector, dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)) or np.linalg.norm(value) < 1e-12:
        raise ValueError("normal must be a finite nonzero XYZ vector")
    return value / np.linalg.norm(value)


def angle_deg(first, second):
    return float(np.degrees(np.arccos(np.clip(unit(first) @ unit(second), -1, 1))))


def literal_assignment(path, name):
    """Read simulation constants without launching its module-level app."""
    values = [node.value for node in ast.parse(path.read_text()).body
              if isinstance(node, ast.Assign)
              and any(isinstance(t, ast.Name) and t.id == name for t in node.targets)]
    if len(values) != 1:
        raise ValueError(f"expected one literal assignment for {name}")
    return ast.literal_eval(values[0])


def load_locked_source(record):
    path = Path(record["path"])
    if sha256(path.read_bytes()).hexdigest() != record["sha256"]:
        raise ValueError(f"source hash mismatch: {path}")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("refusing to overwrite audit output")
    replay = json.loads(args.replay.read_text())
    if replay.get("motion_authorized") is not False:
        raise ValueError("requires a simulation-only source")
    sources = replay["sources"]
    raw_path = load_locked_source(sources["raw_result"])
    urdf = load_locked_source(sources["urdf"])
    table_path = load_locked_source(sources["worktable"])
    import yaml
    board = yaml.safe_load(table_path.read_text())["board"]
    table_z = float(board["table_z_in_left_base_link_m"])
    table_xy_min = np.array(board["origin_in_left_base_link_xy_m"])
    table_xy_max = table_xy_min + np.array(board["calibrated_span_m"])
    raw = json.loads(raw_path.read_text())
    nodes = np.asarray(raw["final_cloth_shape_local_m_env_0"], dtype=float)
    if nodes.shape != (4096, 3) or not np.all(np.isfinite(nodes)):
        raise ValueError("this bounded diagnostic requires the measured 64x64 checkpoint")
    runner = ROOT / "tools/setup/isaac/run_towel_s1_vertex_patch_lift.py"
    authored = unit(literal_assignment(runner, "JAW_PAD_NORMALS_PARENT")["right"]["fixed"])
    center_local = np.array(literal_assignment(runner, "FIXED_JAW_PAD_CENTER_PARENT_M"))
    planned = FIXED_JAW_PAD_NORMAL_IN_GRIPPER["right_"]
    kin = GraspYawKinematics(urdf, "right_")
    root_rotation, root_translation = kin._root_from_base()

    def pose(q):
        rotation, translation = kin._compose(kin._chain, dict(zip(kin.arm_joints, q)))
        return root_rotation @ rotation, root_translation + root_rotation @ translation

    baseline = []
    for name in ("second_correction_contact", "second_correction_translate"):
        phase = next(p for p in replay["phases"] if p["name"] == name)
        rotation, translation = pose(phase["joint_positions_rad"][6:11])
        baseline.append({"phase": name, "pad_center_m": (translation + rotation @ center_local).tolist(),
                         "planned_normal": (rotation @ planned).tolist(),
                         "authored_normal": (rotation @ authored).tolist()})

    # Two locally aligned points on opposite S1 topology halves near the raised
    # crease, away from the separated low-X free ends. No residual-based XYZ tuning.
    lower_point, upper_point = 63 * 64 + 28, 63 * 64 + 34
    limits = json.loads((ROOT / "config/bimanual_operational_limits.json").read_text())["arms"]["right"]
    names = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")
    low = np.array([limits[n]["minimum_urad"] * 1e-6 for n in names]) + .025
    high = np.array([limits[n]["maximum_urad"] * 1e-6 for n in names]) - .025
    # Preserve the actual 2.2 mm pad. Do not repair the S1 collider in this audit.
    face_local = center_local + .0011 * authored
    rng = np.random.default_rng(0)
    seeds = [0.5 * (low + high), *rng.uniform(low, high, (16, 5))]
    candidates = []
    for fixed, moving in ((lower_point, upper_point), (upper_point, lower_point)):
        normal = unit(nodes[moving] - nodes[fixed])

        def residual(q):
            rotation, translation = pose(q)
            return np.r_[(translation + rotation @ face_local - nodes[fixed]) / .0005,
                         (rotation @ authored - normal) / .05]

        fits = [least_squares(residual, seed, bounds=(low, high), max_nfev=200) for seed in seeds]
        fit = min(fits, key=lambda f: np.linalg.norm(residual(f.x)))
        rotation, translation = pose(fit.x)
        position_error = float(np.linalg.norm(translation + rotation @ face_local - nodes[fixed]))
        normal_error = angle_deg(rotation @ authored, normal)
        candidate = {"fixed_particle": fixed, "moving_particle": moving,
                     "joint_positions_rad": fit.x.tolist(), "position_error_m": position_error,
                     "normal_error_deg": normal_error, "ik_pass": position_error <= .0025 and normal_error <= 4.0}
        if candidate["ik_pass"]:
            import trimesh
            # Evaluate the actual registered wrist-camera collision STL, not a TCP proxy.
            link = next(link for link in kin._robot.links if link.name == "right_gripper_link")
            collision = next(c for c in link.collisions if "wrist_cam_mount" in getattr(c.geometry, "filename", ""))
            mesh_path = ROOT / "ros2_ws/src/so101_description/meshes" / Path(collision.geometry.filename).name
            mesh = trimesh.load_mesh(mesh_path)
            mesh_rotation = _rpy_matrix(*collision.origin.rpy)
            mesh_translation = np.array(collision.origin.xyz)
            mesh_vertices = (mesh.vertices @ mesh_rotation.T + mesh_translation) @ rotation.T + translation
            minimum_z = float(mesh_vertices[:, 2].min())
            over_table = np.all((mesh_vertices[:, :2] >= table_xy_min)
                                & (mesh_vertices[:, :2] <= table_xy_max), axis=1)
            table_penetration = max(0., table_z - float(mesh_vertices[over_table, 2].min())) if over_table.any() else 0.
            local_cloth = ((nodes - translation) @ rotation - mesh_translation) @ mesh_rotation
            _, distances, _ = trimesh.proximity.closest_point(mesh, local_cloth)
            candidate.update({"camera_mesh_sha256": sha256(mesh_path.read_bytes()).hexdigest(),
                              "camera_minimum_z_m": minimum_z,
                              "camera_table_penetration_m": table_penetration,
                              "camera_lowest_vertex_xyz_m": mesh_vertices[np.argmin(mesh_vertices[:, 2])].tolist(),
                              "camera_minimum_cloth_surface_distance_m": float(distances.min()),
                              "camera_closest_cloth_particle": int(np.argmin(distances)),
                              "status": "REJECTED_CAMERA_TABLE_INTERFERENCE" if table_penetration > 0 else "UNVALIDATED_FULL_COLLISION_PATH"})
        else:
            candidate["status"] = "REJECTED_PAD_POSE_IK"
        candidates.append(candidate)
    result = {"record_kind": "towel_s2_corner_contact_geometry_audit", "motion_authorized": False,
              "automatic_execution_permitted": False, "isaac_executed": False,
              "status": "NO_EXECUTABLE_CANDIDATE", "raw_source": sources["raw_result"],
              "replay_sha256": sha256(args.replay.read_bytes()).hexdigest(),
              "audit_script_sha256": sha256(Path(__file__).read_bytes()).hexdigest(),
              "runner_sha256": sha256(runner.read_bytes()).hexdigest(), "urdf_source": sources["urdf"],
              "table_z_m": table_z, "pad_normal_disagreement_deg": angle_deg(planned, authored),
              "baseline_pad_frames": baseline, "corner_candidates": candidates,
              "limits": "Finite corner-pinch screen only; no proof that every pinch/push/pull is impossible. No whole-arm or dynamic cloth validation.",
              "next_gate": "Reconcile physical pad frame versus authored collider without silently invalidating accepted S1; design a collision-free full-gripper approach before another cloth run."}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
