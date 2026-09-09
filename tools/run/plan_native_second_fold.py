#!/usr/bin/env python3
"""Plan one left-arm S2 candidate from the current, hash-locked S1 result."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares, minimize
from scipy.spatial.transform import Rotation, Slerp
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages','/usr/lib/python3/dist-packages'])
from tools.lib.so101_mesh_pinch import load_closure_guard
from tools.lib.so101_towel_edge_alignment import ContactPairPlanner
from tools.lib.so101_second_fold_initialization import diagnostic_initial_shape, initial_shape_digest
from tools.lib.so101_gripper_geometry import load_gripper_geometry_candidate

def main():
 b=ROOT/'artifacts/bimanual/planning/so101_surface_matched_pad_20260906';out=b/'second_fold_native'
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--contact-row',type=int,default=59,choices=range(59,64),help='Move the grasp toward the actual free material edge; default preserves prior plans')
 parser.add_argument('--contact-column',type=int,default=19,choices=(15,16,19,23))
 parser.add_argument('--fit-contact-clearance',action='store_true')
 parser.add_argument('--crease-grasp',action='store_true',help='Grasp the existing raised S1 crease between material columns 31 and 32')
 parser.add_argument('--crease-yaw-deg',type=float,default=30.)
 parser.add_argument('--crease-release-height-m',type=float,default=.012)
 parser.add_argument('--diagnostic-towel-x-shift-m',type=float,default=0.,help='Translate only the copied initial shape; does not plan or validate towel repositioning')
 parser.add_argument('--approach-open-model-rad',type=float,default=.18658800423145294)
 parser.add_argument('--entry-standoff-m',type=float,default=0.,help='Descend outside the fixed pad face, then insert sideways at contact height')
 parser.add_argument('--output',type=Path,required=True)
 opt=parser.parse_args()
 if not np.isfinite(opt.entry_standoff_m) or not 0<=opt.entry_standoff_m<=.01:parser.error('entry standoff must be within 0 to 10 mm')
 if opt.output.exists():parser.error('refusing to overwrite existing plan')
 recipe=json.loads((b/'validated_native_recipe.json').read_text())
 for name,digest in recipe['input_sha256'].items():
  if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest:raise RuntimeError('S1 input changed: '+name)
 source=Path(recipe['result']);assert hashlib.sha256(source.read_bytes()).hexdigest()==recipe['result_sha256']
 initial=diagnostic_initial_shape(json.loads(source.read_text())['final_cloth_shape_local_m_env_0'],opt.diagnostic_towel_x_shift_m)
 nodes=initial.reshape(64,64,3)
 pad=json.loads((b/'geometry.json').read_text());urdf=ROOT/'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf'
 guard,_=load_closure_guard(pad,urdf,ROOT,'left');op=.18658800423145294;approach_open=opt.approach_open_model_rad
 candidate=load_gripper_geometry_candidate(ROOT/'config/so101_gripper_geometry.candidate.json')
 import xml.etree.ElementTree as ET
 urdf_upper=float(ET.parse(urdf).getroot().find("joint[@name='left_gripper_joint']/limit").get('upper'))
 if not np.isfinite(approach_open) or not op<=approach_open<=min(urdf_upper,candidate.model_limits_rad('left')[1]):raise ValueError('approach opening outside calibrated command mapping/URDF limits')
 closure=guard.derive_stop(-.023567,approach_open)
 face=np.array(pad['contact_center_parent_m']);face[0]=pad['outer_x_m'];local=face+np.array([.0027,0,0])
 planner=ContactPairPlanner(ROOT,urdf,'left',[local],pad,closure['minimum_model_angle_rad'],op);k=planner.k
 scan=json.loads((out/'static_contact_scan.json').read_text());chosen=min((c for c in scan['candidates'] if c['yaw_deg']==30),key=lambda c:abs(c['col']-opt.contact_column))
 q=np.array(chosen['q']);R0=np.array(chosen['R']);start=nodes[opt.contact_row,opt.contact_column].copy();target=nodes[63-opt.contact_row,opt.contact_column].copy();target[2]=.012
 if (opt.contact_row!=chosen['row'] or opt.contact_column!=chosen['col']) and not opt.crease_grasp:
  # The cached seed is at row 59. First fit the requested edge location so
  # the constrained clearance solve does not start 19 mm away from its target.
  def edge_seed_loss(q1):
   R,t=k._compose(planner.chain,dict(zip(k.arm_joints,q1)))
   return np.r_[(R@local+t-start)/.00025,Rotation.from_matrix(R@R0.T).as_rotvec()/.15]
  seed_fit=least_squares(edge_seed_loss,np.clip(q,planner.lo+1e-6,planner.hi-1e-6),bounds=(planner.lo+1e-6,planner.hi-1e-6),max_nfev=150)
  q=seed_fit.x
 contact_fit=None
 joint_margin=.025 if opt.crease_grasp else 1e-6
 if opt.crease_grasp:
  start=nodes[opt.contact_row,[31,32]].mean(0);target=nodes[63-opt.contact_row,[31,32]].mean(0);target[2]=opt.crease_release_height_m
  desired=Rotation.from_euler('z',opt.crease_yaw_deg,degrees=True).as_matrix()
  q[4]=np.clip(q[4]+np.radians(opt.crease_yaw_deg-30),planner.lo[4]+joint_margin,planner.hi[4]-joint_margin)
  def crease_loss(q1):
   R,t=k._compose(planner.chain,dict(zip(k.arm_joints,q1)))
   return np.r_[(R@local+t-start)/.00025,Rotation.from_matrix(R@desired.T).as_rotvec()/.15]
  fit=least_squares(crease_loss,np.clip(q,planner.lo+joint_margin,planner.hi-joint_margin),bounds=(planner.lo+joint_margin,planner.hi-joint_margin),max_nfev=150)
  q=fit.x;R0,t0=k._compose(planner.chain,dict(zip(k.arm_joints,q)))
  loc=(nodes[opt.contact_row,[31,32]]-t0)@R0
  margins=-(loc[:,1:]@guard.equations[:,:2].T+guard.equations[:,2]).max(1)
  contact_fit={'method':'existing raised S1 crease; vertical approach with tilted gripper','approach_tilt_deg':float(np.degrees(np.arccos(R0[2,2]))),'layer_face_interior_margins_m':margins.tolist(),'position_error_m':float(np.linalg.norm(planner.points(q)[0]-start)),'joint_limit_margin_rad':joint_margin}
  if min(margins)<.0005 or contact_fit['approach_tilt_deg']>30 or contact_fit['position_error_m']>.001:raise RuntimeError('raised crease placement is infeasible: '+str(contact_fit))
  start=planner.points(q)[0].copy()
 if opt.fit_contact_clearance:
  reference=R0.copy();angles=np.linspace(closure['minimum_model_angle_rad'],op,9)
  def loss(q1):
   R,t=k._compose(planner.chain,dict(zip(k.arm_joints,q1)))
   return float(np.sum(((R@local+t-start)*10000)**2)+np.sum(Rotation.from_matrix(R@reference.T).as_rotvec()**2))
  fit=minimize(loss,q,method='SLSQP',bounds=list(zip(planner.lo+1e-6,planner.hi-1e-6)),constraints=[{'type':'ineq','fun':lambda q1: np.array([planner.clearance(q1,a)-.00075 for a in angles])*1000}],options={'maxiter':100,'ftol':1e-10})
  q=fit.x;R0,t0=k._compose(planner.chain,dict(zip(k.arm_joints,q)))
  contact_fit={'method':'fit orientation to measured layer position with full mesh table clearance constraint','message':str(fit.message),'position_error_m':float(np.linalg.norm(planner.points(q)[0]-start)),'approach_tilt_deg':float(np.degrees(np.arccos(np.clip(R0[2,2],-1,1)))),'minimum_table_clearance_m':min(planner.clearance(q,a) for a in angles)}
  bottom_column=int(np.linalg.norm(nodes[opt.contact_row,33:,:2]-start[:2],axis=1).argmin()+33)
  layer_points=nodes[opt.contact_row,[opt.contact_column,bottom_column]]
  local_points=(layer_points-t0)@R0
  face_margins=-(local_points[:,1:]@guard.equations[:,:2].T+guard.equations[:,2]).max(axis=1)
  contact_fit['layer_face_interior_margins_m']=face_margins.tolist()
  contact_fit['material_target_m']=start.tolist()
  contact_fit['pad_reference_offset_m']=(planner.points(q)[0]-start).tolist()
  # Cloth must lie inside the finite face, rather than at its arbitrary centre.
  # This permits a <=2 mm reference shift only when BOTH material layers retain
  # a positive, explicit face-interior margin. Physical contact gates are unchanged.
  if contact_fit['position_error_m']>.002 or min(face_margins)<.00025 or contact_fit['approach_tilt_deg']>12 or contact_fit['minimum_table_clearance_m']<.00025:raise RuntimeError('constrained contact pose is infeasible: '+str(contact_fit))
  start=planner.points(q)[0].copy()
 clear=json.loads((b/'grasp_review/native_span_fold_replay.json').read_text())['selected_candidate']['first_fold'][-1]['joint_positions_rad'];clear[5]=approach_open;clear[11]=op
 oldpath=Path('/home/an-hyeonseo/Documents/GitHub/SO101-Can-Towel-Manipulation/tmp/towel_second_fold_single_left_actual_s1_r63_yaw30_z3_strict_moveit_20260905.json')
 old=json.loads(oldpath.read_text());last=next(p for p in old['selected_candidate']['second_fold'] if p['name']=='second_fold_09');oldq=np.array(last['moveit']['target_positions_rad'][:5]);Rf,_=k._compose(planner.chain,dict(zip(k.arm_joints,oldq)))
 endpoint_fit=None
 if opt.crease_grasp:
  original_Rf=Rf@Rotation.from_euler('z',opt.crease_yaw_deg-30,degrees=True).as_matrix()
  oldq[4]=np.clip(oldq[4]+np.radians(opt.crease_yaw_deg-30),planner.lo[4]+joint_margin,planner.hi[4]-joint_margin)
  def endpoint_loss(q1):
   R,t=k._compose(planner.chain,dict(zip(k.arm_joints,q1)))
   return np.r_[(R@local+t-target)/.00025,Rotation.from_matrix(R@original_Rf.T).as_rotvec()/.15]
  fit=least_squares(endpoint_loss,np.clip(oldq,planner.lo+joint_margin,planner.hi-joint_margin),bounds=(planner.lo+joint_margin,planner.hi-joint_margin),max_nfev=150)
  Rf,_=k._compose(planner.chain,dict(zip(k.arm_joints,fit.x)))
  achieved=planner.points(fit.x)[0]
  endpoint_fit={'method':'nearest reachable crease endpoint; explicit residual reserved for camera observation and right correction','change_from_old_orientation_deg':float(np.degrees(Rotation.from_matrix(Rf@original_Rf.T).magnitude())),'position_error_m':float(np.linalg.norm(achieved-target)),'desired_material_target_m':target.tolist(),'planned_offset_m':(achieved-target).tolist(),'q_rad':fit.x.tolist()}
  if endpoint_fit['position_error_m']>.02:raise RuntimeError('crease endpoint requires more than 20 mm placement correction')
  target=achieved.copy()
 slerp=Slerp([0.,1.],Rotation.from_matrix(np.array([R0,Rf])))
 checks=[];phases=[]
 def solve(name,target,Rdes,seed,opening=False):
  def residual(q1):
   R,t=k._compose(planner.chain,dict(zip(k.arm_joints,q1)))
   return np.r_[(R@local+t-target)/.00025,Rotation.from_matrix(R@Rdes.T).as_rotvec()/.15]
  fit=least_squares(residual,np.clip(seed,planner.lo+joint_margin,planner.hi-joint_margin),bounds=(planner.lo+joint_margin,planner.hi-joint_margin),max_nfev=150,ftol=1e-10,xtol=1e-10,gtol=1e-10)
  err=float(np.linalg.norm(planner.points(fit.x)[0]-target));minimum=min(planner.clearance(fit.x,a) for a in np.linspace(closure['minimum_model_angle_rad'],op,9));R,_=k._compose(planner.chain,dict(zip(k.arm_joints,fit.x)));change=float(np.degrees(Rotation.from_matrix(R@Rdes.T).magnitude()))
  record={'name':name,'q_rad':fit.x.tolist(),'target_m':target.tolist(),'position_error_m':err,'rotation_error_deg':change,'minimum_table_clearance_m':minimum,'passed':bool(err<=.001 and minimum>=.00025 and change<=30)};checks.append(record)
  if not record['passed']:return fit.x,False
  return fit.x,True
 # Descend without pad contact, then make the final side insertion. The default
 # zero standoff preserves the previous direct vertical entry exactly.
 outside=start-R0[:,0]*opt.entry_standoff_m
 entry_targets=[('staging',outside+np.array([0,0,.03]))]
 if opt.entry_standoff_m:entry_targets.append(('entry_clear',outside))
 entry_targets.extend([('contact',start),('lift_probe',start+np.array([0,0,.01]))])
 for name,point in entry_targets:
  qi,ok=solve(name,point,R0,q);phases.append({'name':name,'q_rad':qi.tolist(),'seconds':.3 if name=='contact' and opt.entry_standoff_m else .6})
 q=np.array(phases[-1]['q_rad']);rise=.5*abs(target[1]-start[1])
 for i in range(1,10):
  a=i/9;theta=np.pi*a;point=start+(target-start)*(.5-.5*np.cos(theta));point[2]=start[2]*(1-a)+target[2]*a+rise*np.sin(theta)
  q,ok=solve('fold_%02d'%i,point,slerp(a).as_matrix(),q);phases.append({'name':'fold_%02d'%i,'q_rad':q.tolist(),'seconds':.35})
  if not ok:break
 if all(c['passed'] for c in checks):
  for name,dz in [('retreat',.03)]:
   q,ok=solve(name,target+np.array([0,0,dz]),slerp(1.).as_matrix(),q,True);phases.append({'name':name,'q_rad':q.tolist(),'seconds':.6})
 # Check every interpolated robot segment against all left collision meshes and pad.
 segments=[]
 for a,z in zip(phases,phases[1:]):
  qa,qz=np.array(a['q_rad']),np.array(z['q_rad']);minimum=min(planner.clearance(qa+t*(qz-qa),angle) for t in np.linspace(0,1,21) for angle in [op,closure['minimum_model_angle_rad']]);segments.append({'from':a['name'],'to':z['name'],'minimum_table_clearance_m':minimum,'passed':minimum>=.00025})
 # Wider opening is used only during entry and stationary closure. Transport
 # and release retain the separately verified narrow opening envelope.
 contact_index=next(i for i,p in enumerate(phases) if p['name']=='contact');contact_q=np.array(phases[contact_index]['q_rad'])
 approach_clearance=min(planner.clearance(np.array(a['q_rad'])+t*(np.array(z['q_rad'])-a['q_rad']),approach_open) for a,z in zip(phases[:contact_index],phases[1:contact_index+1]) for t in np.linspace(0,1,41))
 closing_clearance=min(planner.clearance(contact_q,a) for a in np.linspace(closure['minimum_model_angle_rad'],approach_open,81))
 opening_checks={'entry_minimum_table_clearance_m':approach_clearance,'stationary_closure_minimum_table_clearance_m':closing_clearance,'passed':min(approach_clearance,closing_clearance)>=.00025}
 passed=all(c['passed'] for c in checks) and all(c['passed'] for c in segments) and opening_checks['passed']
 report={'status':'S2_NATIVE_CANDIDATE_STATIC_PASS' if passed else 'S2_NATIVE_CANDIDATE_STATIC_BRANCH','motion_authorized':False,'active_arm':'left','source_result':str(source),'source_result_sha256':recipe['result_sha256'],'pad_path':str(b/'geometry.json'),'pad_sha256':hashlib.sha256((b/'geometry.json').read_bytes()).hexdigest(),'closure':closure,'old_angle_rigid_overlap_m':-guard.clearance(-.023567),'clear_model_rad':clear,'contact_row':opt.contact_row,'contact_column':19,'contact_local_point_m':local.tolist(),'phases':phases,'pose_checks':checks,'segment_checks':segments,'scope':'All left collision-mesh vertices including wrist camera and pad checked against table; not a fresh full-robot MoveIt certificate. Cloth contact and native transport require simulation.','strategy':'one left grasp/fold, release, clear, observe; right correction only after new observation'}
 report['contact_column']=opt.contact_column;report['contact_clearance_fit']=contact_fit
 report['opening_checks']=opening_checks
 report['entry_standoff_m']=opt.entry_standoff_m
 report['release_open_model_rad']=op
 report['maximum_transport_gripper_model_rad']=op
 report['closing_speed_rad_s']=op-closure['minimum_model_angle_rad']
 report['diagnostic_towel_x_shift_m']=opt.diagnostic_towel_x_shift_m
 report['initial_cloth_sha256']=initial_shape_digest(initial)
 report['towel_repositioning_motion_executed']=False
 report['grasp_structure']='existing raised crease' if opt.crease_grasp else 'new pucker in folded flat region'
 report['crease_yaw_deg']=opt.crease_yaw_deg if opt.crease_grasp else None
 report['joint_limit_margin_rad']=joint_margin
 report['endpoint_orientation_fit']=endpoint_fit
 if opt.crease_grasp:report['contact_column']=31
 bottom_column=int(np.linalg.norm(nodes[opt.contact_row,33:,:2]-start[:2],axis=1).argmin()+33)
 report['initial_layer_separation_m']=float(nodes[opt.contact_row,opt.contact_column,2]-nodes[opt.contact_row,bottom_column,2]);report['paired_bottom_column']=bottom_column
 if opt.crease_grasp:report['paired_bottom_column']=32;report['initial_layer_separation_m']=float(nodes[opt.contact_row,31,2]-nodes[opt.contact_row,32,2])
 opt.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'status':report['status'],'contact_fit':contact_fit,'failed':[c for c in checks+segments if not c['passed']],'minimum_table_clearance_m':min(c['minimum_table_clearance_m'] for c in checks+segments)},indent=2))
if __name__=='__main__':main()
