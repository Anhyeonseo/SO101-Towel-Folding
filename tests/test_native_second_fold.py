from copy import deepcopy
from tools.run.run_native_second_fold import layered_gate
from tools.lib.so101_mesh_pinch import ActualMeshPinchGate
import numpy as np
from tools.run.run_native_second_fold import material_retention, retained_surface_contacts
from tools.lib.so101_second_fold_initialization import diagnostic_initial_shape, initial_shape_digest
import pytest
from tools.run.run_native_second_fold import closure_step_count
from tools.run.run_native_second_fold import qualified_capture_patch, initial_surface_patch, retention_closure_guard
from tools.run.run_native_second_fold import core_region_retention
from tools.run.run_native_second_fold import retention_step_accepted

def test_diagnostic_continuation_keeps_failed_verdict_and_requires_contacts():
 evidence={'passed':False,'opposing_geometry_passed':True,'layer_material_slip_m':{'0':.0031,'1':.001}}
 assert not retention_step_accepted(evidence)
 assert retention_step_accepted(evidence,observe_core_exceedance=True)
 assert not evidence['passed']
 evidence['opposing_geometry_passed']=False
 assert not retention_step_accepted(evidence,observe_core_exceedance=True)
 evidence['opposing_geometry_passed']=True;evidence['layer_material_slip_m']['0']=np.nan
 assert not retention_step_accepted(evidence,observe_core_exceedance=True)

def test_free_edge_capture_uses_planned_material_row():
 contacts=single_node_contacts()
 for c in contacts:c['particle']+=256
 gate=ActualMeshPinchGate([[4051,4052,4053],[4076,4077,4078]])
 assert not layered_gate(contacts,gate)['passed']
 assert layered_gate(contacts,gate,row=63)['passed']
 assert initial_surface_patch(contacts,columns={0:19,1:44},row=63)=={0:[4051],1:[4076]}
 assert qualified_capture_patch(contacts,gate,columns={0:19,1:44},row=63)=={0:[4051],1:[4076]}

def test_core_bound_is_not_relaxed_when_region_supports_another_point():
 original=np.zeros((4,3));now=original.copy();now[0,0]=.0031
 result=core_region_retention(now,original,{0:[0],1:[2]},{0:[0,1],1:[2,3]},{'passed':True})
 assert not result['passed']
 assert result['limit_m']==.003

def test_surrounding_deformation_is_distinct_from_core_slip():
 original=np.zeros((4,3));now=original.copy();now[1,0]=.003046;now[0,0]=.001467
 result=core_region_retention(now,original,{0:[0],1:[2]},{0:[0,1],1:[2,3]},{'passed':True})
 assert result['passed']
 assert result['region_maximum_relative_motion_m']['0']==.003046
 assert result['layer_material_slip_m']['0']==.001467
 assert not core_region_retention(now,original,{0:[0],1:[2]},{0:[0,1],1:[2,3]},{'passed':False})['passed']
 now[1,0]=np.nan
 assert not core_region_retention(now,original,{0:[0],1:[2]},{0:[0,1],1:[2,3]},{'passed':True})['passed']

def test_core_must_be_nonempty_and_inside_original_region():
 original=np.zeros((4,3))
 for core in ({0:[],1:[2]},{0:[3],1:[2]}):
  with pytest.raises(ValueError):core_region_retention(original,original,core,{0:[0,1],1:[2,3]},{'passed':True})

def test_closure_guard_stops_before_embedding_and_does_not_hide_embedded_nodes():
 contacts=single_node_contacts();ids={0:[3795],1:[3820]}
 for c in contacts:c['signed_distance_m']=.0008
 assert retention_closure_guard(contacts,ids)['allow_additional_closure']
 for distance in (.0005,.0002,-.000018,float('nan')):
  contacts[2]['signed_distance_m']=distance
  assert not retention_closure_guard(contacts,ids)['allow_additional_closure']

def test_closure_guard_does_not_guess_pressure_when_layer_contact_is_missing():
 contacts=single_node_contacts()
 for c in contacts:c['signed_distance_m']=.0008
 assert not retention_closure_guard(contacts[:2],{0:[3795],1:[3820]})['allow_additional_closure']

def test_initial_region_includes_real_unilateral_support_only():
 contacts=single_node_contacts();support=deepcopy(contacts[0]);support['particle']=3731
 bad=[]
 for particle,changes in [(3732,{'penetration_m':0.}), (3733,{'signed_distance_m':-.001}), (3700,{})]:
  c=deepcopy(support);c.update(particle=particle,**changes);bad.append(c)
 assert initial_surface_patch(contacts+[support]+bad,columns={0:19,1:44})=={0:[3731,3795],1:[3820]}

def test_initial_unilateral_node_can_later_support_bilateral_retention():
 contacts=single_node_contacts();support=deepcopy(contacts[0]);support['particle']=3731
 frozen=initial_surface_patch(contacts+[support],columns={0:19,1:44})
 later=deepcopy(contacts)
 for c in later[:2]:c['particle']=3731
 assert retained_surface_contacts(later,frozen,allow_contact_redistribution=True)['passed']
 for c in later[:2]:c['particle']=3732
 assert not retained_surface_contacts(later,frozen,allow_contact_redistribution=True)['passed']

def single_node_contacts():
 contacts=pairs()
 for c in contacts:
  c['particle']=3795 if c['particle']<3800 else 3820
  c['particle_world_m']=[.0005,0,0]
 return contacts

def test_capture_patch_includes_only_initial_qualified_nearby_points():
 contacts=single_node_contacts();additional=deepcopy(contacts[:2])
 for c in additional:c['particle']=3731
 remote=deepcopy(contacts[:2])
 for c in remote:c['particle']=3700
 invalid=deepcopy(contacts[:2])
 for c in invalid:c['particle']=3796
 invalid[-1]['penetration_m']=0.
 gate=ActualMeshPinchGate([[3795,3796,3797],[3731,3732,3733],[3700,3701,3702],[3820,3821,3822]])
 assert qualified_capture_patch(contacts+additional+remote+invalid,gate,columns={0:19,1:44})=={0:[3731,3795],1:[3820]}

def test_contact_can_redistribute_only_within_frozen_capture_set():
 contacts=single_node_contacts();ids={0:[3731,3795],1:[3820]}
 assert retained_surface_contacts(contacts,ids,allow_contact_redistribution=True)['passed']
 assert not retained_surface_contacts(contacts,ids)['passed']
 for c in contacts[:2]:c['particle']=3796
 assert not retained_surface_contacts(contacts,ids,allow_contact_redistribution=True)['passed']

def test_patch_cannot_combine_unopposed_contacts_on_separate_nodes():
 contacts=single_node_contacts();contacts[1]['particle']=3731
 assert not retained_surface_contacts(contacts,{0:[3731,3795],1:[3820]},allow_contact_redistribution=True)['passed']

def test_patch_motion_limit_still_applies_to_no_longer_contacting_nodes():
 original=np.zeros((4,3));now=original.copy();now[0,2]=.0031
 assert not material_retention(now,original,{0:[0,1],1:[2,3]},{'passed':True})['passed']

def test_wider_opening_does_not_increase_closing_speed():
 for opening in (.186588,.7):
  n=closure_step_count(opening,.00698,.179608,1/240)
  assert (opening-.00698)/(n/240)<=.179608
 assert closure_step_count(.7,.00698,.179608,1/240)>900

def test_invalid_closure_speed_is_rejected():
 for speed in (0.,-1.,float('nan')):
  with pytest.raises(ValueError):closure_step_count(.7,.007,speed,1/240)

def test_diagnostic_translation_preserves_source_and_relative_shape():
 source=np.array([[.32,.1,.012],[.35,.2,.004],[.34,.11,.009]])
 before=source.copy();shifted=diagnostic_initial_shape(source,-.05)
 np.testing.assert_array_equal(source,before)
 np.testing.assert_allclose(shifted-source,np.tile([-.05,0,0],(3,1)),atol=1e-16)
 np.testing.assert_allclose(np.diff(shifted,axis=0),np.diff(source,axis=0),atol=1e-16)
 assert initial_shape_digest(source)!=initial_shape_digest(shifted)

def test_invalid_diagnostic_translation_rejected():
 for shift in (float('nan'),float('inf'),-.101):
  with pytest.raises(ValueError):diagnostic_initial_shape([[0.,0.,0.]],shift)

def pairs():
 result=[]
 for col in (19,44):
  for offset,x,normal,shape in [(0,0.,1.,'gripper_link/TowelFixedJawCollider'),(1,.001,-1.,'moving_jaw_link/moving_jaw_so101_v1')]:
   result.append(dict(shape='/Robot/left_'+shape,particle=59*64+col+offset,
    surface_world_m=[x,0,0],surface_local_m=[-.0057 if offset==0 else 0,0,0],
    particle_world_m=[.0002 if offset==0 else .0008,0,0],normal_world=[normal,0,0],normal_local=[normal,0,0],
    penetration_m=.0008,signed_distance_m=.0002,radius_m=.001,effective_friction_coefficient=.8))
 return result

def test_one_sheet_contacts_do_not_prove_both_folded_layers():
 gate=ActualMeshPinchGate([[3795,3796,3797],[3820,3821,3822]])
 assert not layered_gate(pairs()[:2],gate)['passed']
 assert layered_gate(pairs(),gate)['passed']

def test_layer_lost_after_capture_cannot_reuse_initial_pass():
 gate=ActualMeshPinchGate([[3795,3796,3797],[3820,3821,3822]])
 c=pairs();assert layered_gate(c,gate,columns={0:19,1:44})['passed']
 c[-1]['penetration_m']=0
 assert not layered_gate(c,gate,columns={0:19,1:44})['passed']

def test_remote_contacts_cannot_replace_selected_material_patch():
 c=pairs();c[-2]['particle']-=10;c[-1]['particle']-=10
 gate=ActualMeshPinchGate([[3795,3796,3797],[3810,3811,3812]])
 assert not layered_gate(c,gate,columns={0:19,1:44})['passed']

def test_measured_retention_rejects_slip_even_when_contacts_remain():
 capture=np.zeros((4,3));current=capture.copy();current[2,2]=.0031
 assert not material_retention(current,capture,{0:[0,1],1:[2,3]},{'passed':True})['passed']

def test_measured_retention_requires_contacts_and_finite_motion():
 points=np.zeros((2,3));ids={0:[0],1:[1]}
 assert material_retention(points,points,ids,{'passed':True})['passed']
 assert not material_retention(points,points,ids,{'passed':False})['passed']
 bad=points.copy();bad[1,0]=np.nan
 assert not material_retention(bad,points,ids,{'passed':True})['passed']

def test_retained_surfaces_require_both_jaws_on_original_nodes():
 contacts=pairs()
 for c in contacts:c['particle']=3795 if c['particle']<3800 else 3820
 ids={0:[3795],1:[3820]}
 assert retained_surface_contacts(contacts,ids)['passed']
 contacts[-1]['particle']+=1
 assert not retained_surface_contacts(contacts,ids)['passed']

def test_rigid_embedded_or_speculative_contact_cannot_prove_retention():
 contacts=pairs()
 for c in contacts:c['particle']=3795 if c['particle']<3800 else 3820
 contacts[-1]['signed_distance_m']=-.0001
 assert not retained_surface_contacts(contacts,{0:[3795],1:[3820]})['passed']
 contacts[-1]['signed_distance_m']=.001;contacts[-1]['penetration_m']=0
 assert not retained_surface_contacts(contacts,{0:[3795],1:[3820]})['passed']


def test_edge_feed_timing_is_bounded_and_rejects_invalid_schedule():
 from tools.run.run_native_second_fold import edge_feed_progress
 fractions=np.linspace(0,1,241)
 progress=np.array([edge_feed_progress(x,.6) for x in fractions])
 assert progress[0]==0 and progress[-1]==1
 assert np.all(np.diff(progress)>=0) and np.max(np.diff(progress))<=1/144+1e-12
 assert edge_feed_progress(.6,.6)==1
 for bad in (0,-1,np.nan,1.1):
  with pytest.raises(ValueError):edge_feed_progress(.5,bad)


def test_capture_waits_when_triangle_pair_cannot_supply_required_core():
 from tools.run.run_native_second_fold import initial_core_if_ready
 contacts=single_node_contacts()
 gate=ActualMeshPinchGate([[3795,3796,3797],[3820,3821,3822]])
 evidence=layered_gate(contacts,gate)
 assert initial_core_if_ready(contacts,evidence,gate)=={0:[3795],1:[3820]}
 # The upper layer still has an opposing pair on one triangle, but no
 # individual upper particle meets the already-required core condition.
 moving=next(c for c in contacts if c['particle']==3795 and 'moving_jaw' in c['shape'])
 moving['particle']=3796
 evidence=layered_gate(contacts,gate)
 assert evidence['passed']
 assert initial_core_if_ready(contacts,evidence,gate) is None
