from copy import deepcopy
import numpy as np
import pytest
from tools.lib.so101_contact_direction_feedback import contact_direction_score, propose_contact_translation


def contacts():
    result=[]
    for i in (0,3):
        for fixed in (True,False):
            n=np.array([1.,0,0] if fixed else [-.8,.6,0])
            result.append(dict(shape='/Robot/left_gripper_link/TowelFixedJawCollider' if fixed else '/Robot/left_moving_jaw_link/moving_jaw_so101',particle=i,particle_world_m=[0,0,0],surface_world_m=(-.002*n).tolist(),normal_world=n.tolist(),normal_local=n.tolist(),surface_local_m=[-.0057,0,0],radius_m=.003,effective_friction_coefficient=.3 if not fixed else 8.))
    return result


FROZEN={0:{'pair':[0,0],'triangle':0},1:{'pair':[3,3],'triangle':1}}


def test_proposal_improves_local_model_without_changing_observations():
    c=contacts();original=deepcopy(c);r=propose_contact_translation(c,FROZEN)
    assert r['accepted'] and r['score_predicted'] < r['score_before']
    assert np.linalg.norm(r['translation_world_m'])<=.000100001
    assert c==original and r['native_validation_required']


def test_missing_original_layer_cannot_borrow_other_points():
    c=contacts()
    for v in c:
        if v['particle']==3:v['particle']=4
    assert not propose_contact_translation(c,FROZEN)['accepted']


def test_rejects_unbounded_and_nonfinite_proposals():
    for maximum in [0,.001,float('nan')]:
        with pytest.raises(ValueError):propose_contact_translation(contacts(),FROZEN,maximum)
    with pytest.raises(ValueError):contact_direction_score(contacts(),FROZEN,[float('nan'),0,0])


def test_optimizer_roundoff_is_projected_without_relaxing_step_limit(monkeypatch):
    from types import SimpleNamespace
    import tools.lib.so101_contact_direction_feedback as module
    baseline=propose_contact_translation(contacts(),FROZEN)
    x=np.array(baseline['translation_world_m'])*1000*1.00002
    monkeypatch.setattr(module,'minimize',lambda *a,**k:SimpleNamespace(x=x,success=True))
    result=propose_contact_translation(contacts(),FROZEN)
    assert result['accepted']
    assert np.linalg.norm(result['translation_world_m'])<=.0001+1e-15


def test_actual_contact_diagnostic_does_not_certify_angle_or_accept_new_nodes():
    from tools.lib.so101_bimanual_native_contact import observed_material_bilateral_contacts
    c=contacts()
    for x in c:
        x.update(penetration_m=.001,signed_distance_m=.002)
        if 'moving_jaw_link' in x['shape']:x['normal_world']=[1.,0,0]
    observed=observed_material_bilateral_contacts(c,FROZEN)
    assert observed['passed'] and observed['diagnostic_only']
    assert not observed['friction_or_force_closure_validated']
    replaced=deepcopy(c);replaced[-1]['particle']=4
    assert not observed_material_bilateral_contacts(replaced,FROZEN)['passed']
    speculative=deepcopy(c);speculative[-1]['penetration_m']=0
    assert not observed_material_bilateral_contacts(speculative,FROZEN)['passed']
    backface=deepcopy(c);backface[0]['surface_local_m'][0]=-.0079
    assert not observed_material_bilateral_contacts(backface,FROZEN)['passed']
