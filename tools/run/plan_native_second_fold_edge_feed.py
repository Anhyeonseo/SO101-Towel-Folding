#!/usr/bin/env python3
"""One free-end horizontal entry candidate; preserve the baseline fold arc."""
import argparse,copy,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages','/usr/lib/python3/dist-packages'])
from tools.lib.so101_towel_edge_alignment import ContactPairPlanner
from tools.lib.so101_mesh_pinch import load_closure_guard

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base-plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if a.output.exists():p.error('refusing to overwrite a plan')
 base=json.loads(a.base_plan.read_text());assert base['status']=='S2_NATIVE_CANDIDATE_STATIC_PASS' and base['contact_row']==63
 recipe=json.loads((ROOT/'artifacts/bimanual/planning/so101_surface_matched_pad_20260906/validated_native_recipe.json').read_text())
 for name,digest in recipe['input_sha256'].items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
 assert hashlib.sha256(Path(base['source_result']).read_bytes()).hexdigest()==base['source_result_sha256']
 assert hashlib.sha256(Path(base['pad_path']).read_bytes()).hexdigest()==base['pad_sha256']
 pad=json.loads(Path(base['pad_path']).read_text());local=np.array(base['contact_local_point_m']);urdf=ROOT/'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf'
 planner=ContactPairPlanner(ROOT,urdf,'left',[local],pad,base['closure']['minimum_model_angle_rad'],base['closure']['open_model_angle_rad']);guard,_=load_closure_guard(pad,urdf,ROOT,'left');k=planner.k
 old={v['name']:v for v in base['phases']};q0=np.array(old['contact']['q_rad']);R0,t0=k._compose(planner.chain,dict(zip(k.arm_joints,q0)));start=planner.points(q0)[0]
 # Spend only 0.4 mm of the baseline 0.75 mm rigid-table clearance to place
 # both layers further inside the finite face. Hard clearance remains 0.25 mm.
 contact=start-np.array([0,0,.0004]);checks=[]
 def fit(name,target,seed):
  def fun(q):
   R,t=k._compose(planner.chain,dict(zip(k.arm_joints,q)))
   return np.r_[(R@local+t-target)/.0001,Rotation.from_matrix(R@R0.T).as_rotvec()/.15]
  sol=least_squares(fun,seed,bounds=(planner.lo+1e-6,planner.hi-1e-6),max_nfev=180,ftol=1e-10,xtol=1e-10,gtol=1e-10)
  R,t=k._compose(planner.chain,dict(zip(k.arm_joints,sol.x)));err=float(np.linalg.norm(planner.points(sol.x)[0]-target));rot=float(np.degrees(Rotation.from_matrix(R@R0.T).magnitude()));clear=min(planner.clearance(sol.x,x) for x in np.linspace(planner.closed,planner.open,21))
  checks.append(dict(name=name,q_rad=sol.x.tolist(),target_m=target.tolist(),position_error_m=err,rotation_error_deg=rot,minimum_table_clearance_m=clear,passed=bool(err<=.00025 and rot<=10 and clear>=.00025)))
  return sol.x
 qc=fit('contact',contact,q0)
 # Determine outside from actual low collision vertices, not the pad X normal.
 points=[];pos=dict(zip(k.arm_joints,qc));pos['left_gripper_joint']=planner.open
 for chain,v in planner.geometry:
  R,t=k._compose(chain,pos);points.extend(v@R.T+t)
 points=np.array(points);cloth=np.array(json.loads(Path(base['source_result']).read_text())['final_cloth_shape_local_m_env_0']).reshape(64,64,3)
 edge_height=float(cloth[-4:,base['contact_column']-3:base['contact_column']+4,2].max()+.003)
 low=points[points[:,2]<=edge_height];free_y=float(cloth[-1,:,1].max());outside_distance=max(.012,free_y+.003-float(low[:,1].min()))
 if outside_distance>.05:raise RuntimeError('free-end descent needs more than 50 mm clearance')
 outside=contact+np.array([0,outside_distance,0]);feed=contact+np.array([0,.003,0])
 phases=[]
 for name,point,seconds in [('staging',outside+np.array([0,0,.03]),.6),('entry_clear',outside,.6),('entry_feed',feed,.6)]:
  q=fit(name,point,qc);phases.append(dict(name=name,q_rad=q.tolist(),seconds=seconds))
 phases.append(dict(name='contact',q_rad=qc.tolist(),seconds=.4))
 phases.extend(copy.deepcopy([v for v in base['phases'] if v['name']=='lift_probe' or v['name'].startswith('fold_') or v['name']=='retreat']))
 names={c['name'] for c in checks};checks.extend(copy.deepcopy([v for v in base['pose_checks'] if v['name'] not in names]))
 segments=[]
 for x,z in zip(phases,phases[1:]):
  qa,qz=np.array(x['q_rad']),np.array(z['q_rad']);clear=min(planner.clearance(qa+t*(qz-qa),angle) for t in np.linspace(0,1,41) for angle in np.linspace(planner.closed,planner.open,9))
  segments.append(dict(from_phase=x['name'],to_phase=z['name'],minimum_table_clearance_m=clear,passed=bool(clear>=.00025)))
 R,t=k._compose(planner.chain,dict(zip(k.arm_joints,qc)));layerlocal=(cloth[63,[base['contact_column'],base['paired_bottom_column']]]-t)@R;margins=-(layerlocal[:,1:]@guard.equations[:,:2].T+guard.equations[:,2]).max(1)
 # Independently verify that every low vertex is outside the free edge at descent.
 qe=np.array(next(v['q_rad'] for v in phases if v['name']=='entry_clear'));pos=dict(zip(k.arm_joints,qe));pos['left_gripper_joint']=planner.open;low_y=[]
 for chain,v in planner.geometry:
  R,t=k._compose(chain,pos);w=v@R.T+t;low_y.extend(w[w[:,2]<=edge_height,1])
 outside_margin=float(min(low_y)-free_y)
 formation=dict(method='descend outside actual +Y free edge; horizontal insertion; final 3 mm insertion during closure',base_plan=str(a.base_plan.resolve()),base_plan_sha256=hashlib.sha256(a.base_plan.read_bytes()).hexdigest(),outside_distance_m=outside_distance,outside_low_mesh_margin_m=outside_margin,local_flat_edge_height_threshold_m=edge_height,contact_lowering_m=.0004,layer_face_interior_margins_m=margins.tolist(),insertion_complete_closure_fraction=.6,retention_origin_reset=False)
 passed=all(c['passed'] for c in checks+segments) and min(margins)>=.001 and outside_margin>=.002
 result=copy.deepcopy(base);result.update(status='S2_NATIVE_CANDIDATE_STATIC_PASS' if passed else 'S2_NATIVE_CANDIDATE_STATIC_BRANCH',phases=phases,pose_checks=checks,segment_checks=segments,edge_feed=formation)
 a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(status=result['status'],edge_feed=formation,failed=[c for c in checks+segments if not c['passed']]),indent=2))
if __name__=='__main__':main()
