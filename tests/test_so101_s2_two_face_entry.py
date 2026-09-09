from types import SimpleNamespace
import numpy as np
import pytest
from tools.run.plan_native_second_fold_two_face_entry import planar_moving_face


def test_finite_face_preserves_edge_distance():
    vertices=np.array([[-.01,0,0],[-.01,.008,0],[-.01,.008,.006],[-.01,0,.006]])
    x,equations=planar_moving_face(SimpleNamespace(triangles=vertices[[[0,1,2],[0,2,3]]]))
    assert x==pytest.approx(-.01)
    pts=np.array([[.004,.003],[.004,-.001]])
    margins=-(pts@equations[:,:2].T+equations[:,2]).max(1)
    assert margins==pytest.approx([.003,-.001])


def test_disconnected_surface_must_not_be_bridged_by_convex_hull():
    triangles=np.array([[[-.01,0,0],[-.01,.001,0],[-.01,0,.001]],[[-.01,.005,0],[-.01,.006,0],[-.01,.005,.001]]])
    with pytest.raises(ValueError,match='filled convex face'):
        planar_moving_face(SimpleNamespace(triangles=triangles))
