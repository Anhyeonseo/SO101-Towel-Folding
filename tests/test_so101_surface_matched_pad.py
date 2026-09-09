from hashlib import sha256
from pathlib import Path
import json
import numpy as np
import pytest
import trimesh
from tools.lib.so101_surface_matched_pad import (
    load_surface_matched_pad, require_pad_checkpoint_identity, author_surface_matched_pad,
)
from tools.run.build_so101_surface_matched_pad import extrude_face

ROOT = Path(__file__).resolve().parents[1]
GEOMETRY = ROOT / 'artifacts/bimanual/planning/so101_surface_matched_pad_20260906/geometry.json'


def test_pad_is_full_face_outward_extrusion_with_correct_volume():
    data = load_surface_matched_pad(GEOMETRY, ROOT)
    mesh = trimesh.Trimesh(vertices=data['vertices_m'], faces=data['faces'], process=False)
    assert mesh.is_watertight and mesh.is_winding_consistent
    assert mesh.bounds[:,0] == pytest.approx([-.0079,-.0057])
    assert mesh.volume == pytest.approx(data['source_patch_area_m2']*.0022, rel=1e-8)
    assert np.ptp(mesh.vertices[:,1]) > .009
    assert np.ptp(mesh.vertices[:,2]) > .009
    # The round profile occupies less area than its bounding rectangle.
    assert data['source_patch_area_m2'] < .8*np.prod(np.ptp(mesh.vertices[:,1:],axis=0))
    stl = trimesh.load_mesh(ROOT/data['mesh_path'])
    assert stl.bounds == pytest.approx(mesh.bounds, abs=1e-8)
    assert data['gripper_command_mapping_changed'] is False


def test_extrusion_preserves_a_concave_face_instead_of_convexifying():
    yz = np.array([[0,0],[2,0],[2,1],[1,1],[1,2],[0,2]],dtype=float)*.001
    vertices = np.column_stack([np.full(6,-.0079),yz])
    face = trimesh.Trimesh(vertices=vertices, faces=[[0,1,3],[1,2,3],[0,3,5],[3,4,5]],process=False)
    pad = extrude_face(face,.0022)
    assert pad.volume == pytest.approx(3e-6*.0022)
    assert pad.is_watertight
    with pytest.raises(ValueError):
        extrude_face(face,-.0022)


def test_legacy_contact_checkpoint_cannot_validate_new_pad():
    expected = sha256(GEOMETRY.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match='revalidation'):
        require_pad_checkpoint_identity({'gripper_candidate':{}}, expected)
    require_pad_checkpoint_identity({'gripper_candidate':{'surface_matched_pad_sha256':expected}},expected)
    require_pad_checkpoint_identity({'gripper_candidate':{}},None)


def test_isaac_usd_uses_the_exact_planning_mesh_at_identity():
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    data = load_surface_matched_pad(GEOMETRY,ROOT)
    stage = Usd.Stage.CreateInMemory()
    material = UsdShade.Material.Define(stage,'/Rubber')
    mesh = author_surface_matched_pad(stage,'/Gripper/TowelFixedJawCollider',data,material)
    assert np.asarray(mesh.GetPointsAttr().Get()) == pytest.approx(np.asarray(data['vertices_m']),abs=1e-8)
    assert list(mesh.GetFaceVertexIndicesAttr().Get()) == [i for face in data['faces'] for i in face]
    assert not UsdGeom.Xformable(mesh).GetOrderedXformOps()
    assert UsdPhysics.CollisionAPI(mesh.GetPrim()).GetCollisionEnabledAttr().Get()
    assert UsdPhysics.MeshCollisionAPI(mesh.GetPrim()).GetApproximationAttr().Get() == 'none'
