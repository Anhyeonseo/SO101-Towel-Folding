import numpy as np
import pytest
from tools.lib.towel_motion_timing import smooth_path_samples


def test_retiming_preserves_segments_and_endpoint_and_slows_endpoints():
    points=np.array([[0.,0.],[1.,2.],[3.,2.]])
    rows=smooth_path_samples(points,4.,.01)
    np.testing.assert_allclose(rows[-1],points[-1])
    first=rows[:,0]<=1
    np.testing.assert_allclose(rows[first,1],2*rows[first,0])
    np.testing.assert_allclose(rows[~first,1],2)
    steps=np.linalg.norm(np.diff(np.vstack([points[0],rows]),axis=0),axis=1)
    assert steps[0]<steps.max()/100 and steps[-1]<steps.max()/100
    assert np.all(np.diff(rows[:,0])>=0)


def test_rejects_invalid_timing():
    with pytest.raises(ValueError):smooth_path_samples([[0],[1]],0,.01)
