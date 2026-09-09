#!/usr/bin/env python3
"""Add one post-lift seating posture to a verified isolated S2 plan."""
import argparse,copy,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages','/usr/lib/python3/dist-packages'])
from tools.lib.so101_towel_edge_alignment import ContactPairPlanner

def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--base-plan',type=Path,required=True)
 parser.add_argument('--output',type=Path,required=True)
 parser.add_argument('--pivot',type=Path,help='Use a measured initial lower-core point as the seating rotation centre')
 parser.add_argument('--tilt-deg',type=float,default=-15.)
 opt=parser.parse_args()
 if opt.output.exists():parser.error('refusing to overwrite a plan')
 if not np.isfinite(opt.tilt_deg) or not -25<=opt.tilt_deg<=-5:parser.error('seating tilt must be -25 to -5 degrees')
 base=json.loads(opt.base_plan.read_text());assert base['status']=='S2_NATIVE_CANDIDATE_STATIC_PASS'
 recipe=json.loads((ROOT/'artifacts/bimanual/planning/so101_surface_matched_pad_20260906/validated_native_recipe.json').read_text())
 for name,digest in recipe['input_sha256'].items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
 pad=json.loads(Path(base['pad_path']).read_text());local=np.array(base['contact_local_point_m'])
 planner=ContactPairPlanner(ROOT,ROOT/'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf','left',[local],pad,base['closure']['minimum_model_angle_rad'],base['closure']['open_model_angle_rad'])
 k=planner.k;old={v['name']:v for v in base['phases']};oldchecks={v['name']:v for v in base['pose_checks']}
 def rotation(q):return k._compose(planner.chain,dict(zip(k.arm_joints,q)))[0]
 q0=np.array(old['lift_probe']['q_rad']);R0=rotation(q0);Rf=rotation(old['fold_09']['q_rad']);Rwanted=R0@Rotation.from_euler('y',opt.tilt_deg,degrees=True).as_matrix()
 angles=np.linspace(planner.closed,planner.open,9)
 def fit(name,target,Rtarget,seed,anchor=None):
  anchor=local if anchor is None else anchor
  def residual(q):
   R,t=k._compose(planner.chain,dict(zip(k.arm_joints,q)))
   return np.r_[(R@anchor+t-target)/.00025,Rotation.from_matrix(R@Rtarget.T).as_rotvec()/.15]
  sol=least_squares(residual,np.clip(seed,planner.lo+1e-6,planner.hi-1e-6),bounds=(planner.lo+1e-6,planner.hi-1e-6),max_nfev=200,ftol=1e-10,xtol=1e-10,gtol=1e-10)
  R,t=k._compose(planner.chain,dict(zip(k.arm_joints,sol.x)));error=float(np.linalg.norm(R@anchor+t-target));rot_error=float(np.degrees(Rotation.from_matrix(R@Rtarget.T).magnitude()));clearance=min(planner.clearance(sol.x,a) for a in angles)
  rotation_limit=10. if name=='seat' else 30. # Seat intent is strict; fold retains the base planner's bound.
  c={'name':name,'q_rad':sol.x.tolist(),'target_m':np.asarray(target).tolist(),'position_error_m':error,'rotation_error_deg':rot_error,'rotation_error_limit_deg':rotation_limit,'minimum_table_clearance_m':clearance,'fixed_face_upward_component':float(R[2,0]),'passed':bool(error<=.001 and rot_error<=rotation_limit and clearance>=.00025)}
  return sol.x,c
 pivot_data=json.loads(opt.pivot.read_text()) if opt.pivot else None
 anchor=np.array(pivot_data['point_local_m']) if pivot_data else local
 if not np.isfinite(anchor).all() or anchor.shape!=(3,) or np.linalg.norm(anchor-local)>.02:raise ValueError('invalid measured seating pivot')
 Rbase,tbase=k._compose(planner.chain,dict(zip(k.arm_joints,q0)))
 target=Rbase@anchor+tbase;q,seat=fit('seat',target,Rwanted,q0,anchor);Rseat=rotation(q)
 seat['constraint_local_point_m']=anchor.tolist()
 seat['planned_tcp_m']=planner.points(q)[0].tolist()
 support={'before_fixed_face_upward_component':float(R0[2,0]),'after_fixed_face_upward_component':float(Rseat[2,0]),'actual_rotation_deg':float(np.degrees(Rotation.from_matrix(Rseat@R0.T).magnitude()))}
 support['passed']=bool(Rseat[2,0]>=.1 and Rseat[2,0]-R0[2,0]>=.08)
 seat['passed']=bool(seat['passed'] and support['passed'])
 lift_index=next(i for i,v in enumerate(base['phases']) if v['name']=='lift_probe')
 phases=copy.deepcopy(base['phases'][:lift_index+1]);checks=[copy.deepcopy(oldchecks[v['name']]) for v in phases]
 phases.append({'name':'seat','q_rad':q.tolist(),'seconds':.5});checks.append(seat)
 interpolation=Slerp([0.,1.],Rotation.from_matrix([Rseat,Rf]))
 for i in range(1,10):
  name=f'fold_{i:02}';q,c=fit(name,np.array(oldchecks[name]['target_m']),interpolation(i/9).as_matrix(),q)
  phases.append({**old[name],'q_rad':q.tolist()});checks.append(c)
 phases.append(copy.deepcopy(old['retreat']));checks.append(copy.deepcopy(oldchecks['retreat']))
 segments=[]
 for a,z in zip(phases,phases[1:]):
  qa,qz=np.array(a['q_rad']),np.array(z['q_rad']);clearance=min(planner.clearance(qa+t*(qz-qa),angle) for t in np.linspace(0,1,21) for angle in [planner.closed,planner.open])
  segments.append({'from':a['name'],'to':z['name'],'minimum_table_clearance_m':clearance,'passed':clearance>=.00025})
 result=copy.deepcopy(base);result.update(phases=phases,pose_checks=checks,segment_checks=segments,seating={'base_plan':str(opt.base_plan.resolve()),'base_plan_sha256':hashlib.sha256(opt.base_plan.read_bytes()).hexdigest(),'requested_local_y_tilt_deg':opt.tilt_deg,'after_lift':True,'measured_pivot':pivot_data,'retention_origin_reset':False,'support_orientation':support,'scope':'Static support-normal orientation only; actual material seating requires native contact validation.'})
 result['status']='S2_NATIVE_CANDIDATE_STATIC_PASS' if all(c['passed'] for c in checks+segments) else 'S2_NATIVE_CANDIDATE_STATIC_BRANCH'
 opt.output.write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({'status':result['status'],'seat':seat,'support':support,'failed':[c for c in checks+segments if not c['passed']]},indent=2))
if __name__=='__main__':main()
