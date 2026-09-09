#!/usr/bin/env python3
"""Fit a supporting fixed-face orientation before a direct top-entry grasp."""
import argparse,copy,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.optimize import minimize,least_squares
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages','/usr/lib/python3/dist-packages'])
from tools.lib.so101_towel_edge_alignment import ContactPairPlanner
from tools.lib.so101_mesh_pinch import load_closure_guard

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base-plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--maximize-feasible-support',action='store_true');a=p.parse_args()
 if a.output.exists():p.error('refusing to overwrite a plan')
 base=json.loads(a.base_plan.read_text());assert base['status']=='S2_NATIVE_CANDIDATE_STATIC_PASS' and base['contact_row']==63 and not base.get('edge_feed')
 recipe=json.loads((ROOT/'artifacts/bimanual/planning/so101_surface_matched_pad_20260906/validated_native_recipe.json').read_text())
 for name,digest in recipe['input_sha256'].items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
 assert hashlib.sha256(Path(base['source_result']).read_bytes()).hexdigest()==base['source_result_sha256']
 assert hashlib.sha256(Path(base['pad_path']).read_bytes()).hexdigest()==base['pad_sha256']
 pad=json.loads(Path(base['pad_path']).read_text());local=np.array(base['contact_local_point_m']);urdf=ROOT/'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf'
 planner=ContactPairPlanner(ROOT,urdf,'left',[local],pad,base['closure']['minimum_model_angle_rad'],base['closure']['open_model_angle_rad']);guard,_=load_closure_guard(pad,urdf,ROOT,'left');k=planner.k
 old={v['name']:v for v in base['phases']};oldchecks={v['name']:v for v in base['pose_checks']};q0=np.array(old['contact']['q_rad']);R0,t0=k._compose(planner.chain,dict(zip(k.arm_joints,q0)))
 cloth=np.array(json.loads(Path(base['source_result']).read_text())['final_cloth_shape_local_m_env_0']).reshape(64,64,3);layers=cloth[63,[base['contact_column'],base['paired_bottom_column']]];target=layers[0];Rwant=R0@Rotation.from_euler('y',-15,degrees=True).as_matrix();angles=np.linspace(planner.closed,planner.open,9)
 def data(q):
  R,t=k._compose(planner.chain,dict(zip(k.arm_joints,q)));loc=(layers-t)@R;mar=-(loc[:,1:]@guard.equations[:,:2].T+guard.equations[:,2]).max(1)
  return R,t,loc,mar
 def loss(q):
  R,t,loc,mar=data(q)
  if a.maximize_feasible_support:return float(-100*R[2,0]+.001*np.sum((q-q0)**2))
  return float(np.sum(((R@local+t-target)/.001)**2)+np.sum((Rotation.from_matrix(R@Rwant.T).as_rotvec()/.15)**2))
 def constraints(q):
  R,t,loc,mar=data(q);delta=R@local+t-target
  return np.r_[(mar-.001)*1000,[(planner.clearance(q,x)-.00035)*1000 for x in angles],(loc[:,0]-pad['outer_x_m']-.0003)*1000,(.003-(loc[:,0]-pad['outer_x_m']))*1000,(.002-np.linalg.norm(delta))*1000,(R[2,0]-(R0[2,0] if a.maximize_feasible_support else .15))*10,(R[2,2]-np.cos(np.radians(25)))*10]
 sol=minimize(loss,q0,method='SLSQP',bounds=list(zip(planner.lo+1e-6,planner.hi-1e-6)),constraints=[{'type':'ineq','fun':constraints}],options={'maxiter':200,'ftol':1e-9})
 qc=sol.x;R,t,loc,mar=data(qc);contact=R@local+t
 formation={'method':'direct top approach with supporting fixed-face orientation established before closure','optimizer_message':str(sol.message),'maximize_feasible_support':a.maximize_feasible_support,'fixed_face_upward_component':float(R[2,0]),'baseline_fixed_face_upward_component':float(R0[2,0]),'tilt_deg':float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))),'layer_face_interior_margins_m':mar.tolist(),'layer_fixed_plane_distances_m':(loc[:,0]-pad['outer_x_m']).tolist(),'position_error_m':float(np.linalg.norm(contact-target)),'minimum_table_clearance_m':min(planner.clearance(qc,x) for x in np.linspace(planner.closed,planner.open,81)),'minimum_scaled_constraint':float(constraints(qc).min()),'base_plan':str(a.base_plan.resolve()),'retention_origin_reset':False,'q_rad':qc.tolist(),'constraint_numeric_tolerance_m':1e-7}
 # Geometric constraints are scaled to mm: allow 0.1 micrometre solver roundoff,
 # while the independent 0.25 mm table safety bound remains exact.
 formation['passed']=bool(constraints(qc).min()>=-1e-4 and formation['minimum_table_clearance_m']>=.00025)
 result=copy.deepcopy(base);result.update(status='S2_NATIVE_CANDIDATE_STATIC_BRANCH',supported_entry=formation)
 if not formation['passed']:
  a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'status':result['status'],'supported_entry':formation},indent=2));return
 phases=[];checks=[]
 def fit(name,point,Rdes,seed,seconds):
  def f(q):
   rr,tt=k._compose(planner.chain,dict(zip(k.arm_joints,q)))
   return np.r_[(rr@local+tt-point)/.00025,Rotation.from_matrix(rr@Rdes.T).as_rotvec()/.15]
  fit=least_squares(f,np.clip(seed,planner.lo+1e-6,planner.hi-1e-6),bounds=(planner.lo+1e-6,planner.hi-1e-6),max_nfev=180,ftol=1e-10,xtol=1e-10,gtol=1e-10);q=fit.x;rr,tt=k._compose(planner.chain,dict(zip(k.arm_joints,q)));err=float(np.linalg.norm(rr@local+tt-point));re=float(np.degrees(Rotation.from_matrix(rr@Rdes.T).magnitude()));clear=min(planner.clearance(q,x) for x in angles)
  phases.append(dict(name=name,q_rad=q.tolist(),seconds=seconds));checks.append(dict(name=name,q_rad=q.tolist(),target_m=point.tolist(),position_error_m=err,rotation_error_deg=re,minimum_table_clearance_m=clear,passed=bool(err<=.001 and re<=30 and clear>=.00025)));return q
 fit('staging',contact+np.array([0,0,.03]),R,qc,.6)
 phases.append(dict(name='contact',q_rad=qc.tolist(),seconds=.6));checks.append(dict(name='contact',q_rad=qc.tolist(),target_m=contact.tolist(),position_error_m=0.,rotation_error_deg=0.,minimum_table_clearance_m=formation['minimum_table_clearance_m'],passed=True))
 q=fit('lift_probe',contact+np.array([0,0,.01]),R,qc,.6)
 Rf,_=k._compose(planner.chain,dict(zip(k.arm_joints,old['fold_09']['q_rad'])));interp=Slerp([0.,1.],Rotation.from_matrix([R,Rf]))
 for i in range(1,10):
  name=f'fold_{i:02}';q=fit(name,np.array(oldchecks[name]['target_m']),interp(i/9).as_matrix(),q,.35)
 phases.append(copy.deepcopy(old['retreat']));checks.append(copy.deepcopy(oldchecks['retreat']))
 segments=[]
 for x,z in zip(phases,phases[1:]):
  qa,qz=np.array(x['q_rad']),np.array(z['q_rad']);clear=min(planner.clearance(qa+t*(qz-qa),angle) for t in np.linspace(0,1,41) for angle in angles);segments.append(dict(from_phase=x['name'],to_phase=z['name'],minimum_table_clearance_m=clear,passed=bool(clear>=.00025)))
 result.update(phases=phases,pose_checks=checks,segment_checks=segments,status='S2_NATIVE_CANDIDATE_STATIC_PASS' if all(c['passed'] for c in checks+segments) else 'S2_NATIVE_CANDIDATE_STATIC_BRANCH')
 a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(status=result['status'],supported_entry=formation,failed=[c for c in checks+segments if not c['passed']]),indent=2))
if __name__=='__main__':main()
