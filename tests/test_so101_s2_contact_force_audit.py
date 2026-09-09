import numpy as np
import pytest
from tools.lib.so101_s2_contact_force_audit import force_law


def test_no_adhesion_or_friction_without_normal_contact():
    r=force_law(-.001,[0,0,1],[.01,0,-.01],100,1,.5,.01,.001)
    assert r['force_world_n']==[0,0,0]


def test_friction_opposes_sliding_and_is_bounded_by_elastic_coulomb_load():
    for slip in [1e-9,1e-6,.01]:
        r=force_law(.001,[0,0,1],[slip,0,0],100,0,.5,.01,.001)
        assert r['force_world_n'][0]<0
        assert r['tangential_n']<=.5*r['normal_elastic_n']+1e-12
        assert r['friction_utilization']<=1+1e-12


def test_damping_resists_approach_without_tensile_force_on_separation():
    args=(.001,[0,0,1])
    into=force_law(*args,[0,0,-.001],100,.01,.5,.01,.001)
    away=force_law(*args,[0,0,.001],100,.01,.5,.01,.001)
    assert into['normal_damping_n']>0 and away['normal_damping_n']==0
    with pytest.raises(ValueError):force_law(*args,[np.nan,0,0],100,.01,.5,.01,.001)
