#!/usr/bin/env python3
"""Add one static hold and 3 mm horizontal load probe to baseline S2."""
import argparse,copy,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT));sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages','/usr/lib/python3/dist-packages'])
from tools.lib.so101_towel_edge_alignment import ContactPairPlanner

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base-plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--continue-fold',action='store_true');a=p.parse_args()
 if a.output.exists():p.error('refusing to overwrite')
 base=json.loads(a.base_plan.read_text());assert base['status']=='S2_NATIVE_CANDIDATE_STATIC_PASS'
 pad=json.loads(Path(base['pad_path']).read_text());local=np.array(base['contact_local_point_m']);pl=ContactPairPlanner(ROOT,ROOT/'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf','left',[local],pad,base['closure']['minimum_model_angle_rad'],base['closure']['open_model_angle_rad']);k=pl.k
 old={x['name']:x for x in base['phases']};q0=np.array(old['lift_probe']['q_rad']);R0,t0=k._compose(pl.chain,dict(zip(k.arm_joints,q0)));start=pl.points(q0)[0];direction=pl.points(old['fold_01']['q_rad'])[0]-start;direction[2]=0;direction/=np.linalg.norm(direction);target=start+.003*direction
 def fun(q):
  R,t=k._compose(pl.chain,dict(zip(k.arm_joints,q)))
  return np.r_[(R@local+t-target)/.0001,Rotation.from_matrix(R@R0.T).as_rotvec()/.15]
 sol=least_squares(fun,q0,bounds=(pl.lo+1e-6,pl.hi-1e-6),max_nfev=150,ftol=1e-10,xtol=1e-10,gtol=1e-10);q=sol.x;R,t=k._compose(pl.chain,dict(zip(k.arm_joints,q)));err=float(np.linalg.norm(pl.points(q)[0]-target));rot=float(np.degrees(Rotation.from_matrix(R@R0.T).magnitude()));checks=[]
 for name,qa,qz in [('lift_to_probe',q0,q),('probe_to_fold01',q,np.array(old['fold_01']['q_rad']))]:
  clear=min(pl.clearance(qa+x*(qz-qa),angle) for x in np.linspace(0,1,41) for angle in np.linspace(pl.closed,pl.open,9));checks.append(dict(name=name,minimum_table_clearance_m=clear,passed=clear>=.00025))
 passed=err<=.00025 and rot<=1 and all(c['passed'] for c in checks)
 result=copy.deepcopy(base);i=next(i for i,v in enumerate(result['phases']) if v['name']=='lift_probe');result['phases'].insert(i+1,dict(name='load_probe',q_rad=q.tolist(),seconds=.3))
 result['grasp_probe']={'hold_after_lift_s':.4,'distance_m':.003,'direction_world':direction.tolist(),'hold_after_probe_s':.15,'stop_after_probe':not a.continue_fold,'position_error_m':err,'rotation_error_deg':rot,'checks':checks,'base_plan':str(a.base_plan.resolve()),'base_plan_sha256':hashlib.sha256(a.base_plan.read_bytes()).hexdigest(),'passed':passed}
 result['status']='S2_NATIVE_CANDIDATE_STATIC_PASS' if passed else 'S2_NATIVE_CANDIDATE_STATIC_BRANCH';a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result['grasp_probe'],indent=2))
if __name__=='__main__':main()
