import numpy as np
import trimesh
from tools.run.plan_native_second_fold_contact_balance import contact_state


def plane(x,reverse=False):
    vertices=np.array([[x,-.01,-.01],[x,.01,-.01],[x,.01,.01],[x,-.01,.01]])
    faces=np.array([[0,1,2],[0,2,3]])
    return trimesh.Trimesh(vertices=vertices,faces=faces[:,::-1] if reverse else faces,process=False)


def test_frozen_lateral_shift_can_restore_contact_without_closing_jaw():
    # Analytic parallel planes: opening stays 5 mm, but the hand moves 0.6 mm.
    points=np.array([[.0002,0,0],[.0018,0,0],[.0025,.002,0]])
    core={0:[2],1:[1]};region={0:[0,2],1:[1]}
    fixed,moving=plane(0),plane(.005,True)
    before=contact_state(points,points,region,core,fixed,moving)
    assert not before['passed_retention_geometry']
    assert before['minimum_active_fixed_margin_m']<.0005
    shifted=points+np.array([.0006,0,0])
    after=contact_state(shifted,points,region,core,fixed,moving)
    # Use the upper peripheral original-region node as new bilateral support.
    # The original core still must pass its independent 3 mm motion bound.
    # Here the upper core leaves fixed contact and no upper replacement reaches
    # moving contact, so the audit must reject, despite restoring lower contact.
    assert after['retained_particles']['1']==[1]
    assert not after['passed_retention_geometry']
    assert after['minimum_active_fixed_margin_m']>.0005


def test_inside_moving_mesh_and_excess_core_motion_are_not_contact_success():
    fixed,moving=plane(0),plane(.005,True)
    xyz=np.array([[.0025,0,0],[.0025,.002,0]])
    region=core={0:[0],1:[1]}
    assert contact_state(xyz,xyz,region,core,fixed,moving)['passed_retention_geometry']
    embedded=xyz.copy();embedded[1,0]=.0051
    assert not contact_state(embedded,xyz,region,core,fixed,moving)['passed_retention_geometry']
    displaced_capture=xyz.copy();displaced_capture[1,1]-=.0031
    assert not contact_state(xyz,displaced_capture,region,core,fixed,moving)['passed_retention_geometry']
