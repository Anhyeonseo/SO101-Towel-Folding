"""Shared, simulation-only fixed-pad mesh contract for planning and Isaac."""
from hashlib import sha256
import json
from pathlib import Path
import numpy as np

FIXED_PAD_OUTWARD_NORMAL = (1.0, 0.0, 0.0)


def load_surface_matched_pad(path: Path, root: Path) -> dict:
    document = json.loads(path.read_text())
    if (document.get('record_kind') != 'so101_surface_matched_fixed_pad'
            or document.get('motion_authorized') is not False
            or document.get('frame') != 'gripper_link'
            or tuple(document.get('outward_normal', ())) != FIXED_PAD_OUTWARD_NORMAL
            or document.get('thickness_m') != .0022):
        raise ValueError('invalid surface-matched fixed-pad contract')
    for key in ('source_mesh', 'mesh'):
        filename = document['source_mesh' if key == 'source_mesh' else 'mesh_path']
        if sha256((root / filename).read_bytes()).hexdigest() != document[key + '_sha256']:
            raise ValueError('surface-matched pad mesh identity differs')
    vertices = np.asarray(document['vertices_m'], dtype=float)
    faces = np.asarray(document['faces'])
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or not np.all(np.isfinite(vertices))
            or faces.ndim != 2 or faces.shape[1] != 3 or not np.issubdtype(faces.dtype, np.integer)
            or faces.size == 0 or faces.min() < 0 or faces.max() >= len(vertices)
            or not np.allclose([vertices[:, 0].min(), vertices[:, 0].max()], [-.0079, -.0057], atol=1e-9, rtol=0)):
        raise ValueError('invalid surface-matched pad mesh arrays')
    return document


def require_pad_checkpoint_identity(document: dict, expected_sha256: str | None) -> None:
    actual = document.get('gripper_candidate', {}).get('surface_matched_pad_sha256')
    if actual != expected_sha256:
        raise ValueError('checkpoint fixed-pad geometry differs; contact revalidation required')


def author_surface_matched_pad(stage, path, geometry, material):
    """Author the exact triangle mesh at identity in the gripper-link frame."""
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in geometry['vertices_m']])
    mesh.CreateFaceVertexCountsAttr([3] * len(geometry['faces']))
    mesh.CreateFaceVertexIndicesAttr([int(i) for face in geometry['faces'] for i in face])
    mesh.CreateSubdivisionSchemeAttr().Set('none')
    mesh.MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr().Set(True)
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set('none')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material, materialPurpose='physics')
    return mesh
