import json
import numpy as np
import trimesh
from tools.run.audit_r2_pad_intersection import audit


def test_surface_crossing_is_detected_even_when_all_vertices_are_outside(tmp_path):
    mesh = trimesh.creation.box(extents=[2, 2, 2])
    pad = tmp_path/'pad.json'
    pad.write_text(json.dumps({'vertices_m': mesh.vertices.tolist(), 'faces': mesh.faces.tolist()}))
    (tmp_path/'input.json').write_text(json.dumps({'plan': {'pad_path': str(pad)}}))
    (tmp_path/'result.json').write_text(json.dumps({'recording_body_labels': ['/left_gripper_link'],
        'stages': [{'name': 'crossing', 'time_s': 0}, {'name': 'clear', 'time_s': 1}]}))
    np.save(tmp_path/'cloth_triangles.npy', [[0, 1, 2]])
    triangle = np.array([[-2, 0, 0], [2, -2, 0], [2, 2, 0]])
    for name, points in [('crossing', triangle), ('clear', triangle+[0, 0, 3])]:
        np.savez(tmp_path/(name+'_solver_contacts.npz'), particle_q=points,
                 body_q=np.array([[0, 0, 0, 0, 0, 0, 1]]))
    result = audit(tmp_path)['stages']
    assert result['crossing']['cloth_vertices_inside_pad'] == []
    assert result['crossing']['triangles_with_interior_inside_and_vertices_outside'] == [0]
    assert result['crossing']['maximum_sampled_interior_depth_m'] > 0
    assert result['clear']['cloth_vertices_inside_pad'] == []
    assert result['clear']['triangles_with_interior_inside_and_vertices_outside'] == []
