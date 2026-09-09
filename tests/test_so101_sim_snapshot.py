from types import SimpleNamespace
import numpy as np
from tools.lib.so101_sim_snapshot import ArrayStateSnapshot


def test_snapshot_restores_arrays_aliases_buffers_and_counters():
    import warp as wp
    import torch
    a=wp.array([1.,2.],dtype=wp.float32,device='cpu')
    b=wp.array([3.,4.],dtype=wp.float32,device='cpu')
    root=SimpleNamespace(state0=a,state1=b,counter=4,extra={'array':np.array([5.,6.])}, tensor=torch.tensor([7.]))
    snapshot=ArrayStateSnapshot({'root':root})
    a.fill_(9.);b.fill_(10.);root.state0,root.state1=b,a
    root.counter=99;root.extra['array'][:]=0;root.extra['new']=True;root.tensor[:]=0
    snapshot.restore()
    assert root.state0 is a and root.state1 is b and root.counter==4
    np.testing.assert_equal(a.numpy(),[1.,2.]);np.testing.assert_equal(b.numpy(),[3.,4.])
    np.testing.assert_equal(root.extra['array'],[5.,6.]);assert 'new' not in root.extra
    assert root.tensor.item()==7.
