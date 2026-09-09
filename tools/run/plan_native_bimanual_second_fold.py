#!/usr/bin/env python3
"""Geometry-only current-S1 bimanual fold, using the user's crossed jaw axes.

No node attachments or execution. The original S1 arch is preserved for a
later native-contact descent test; a flattened copy is never substituted.
"""
import json,hashlib,sys,argparse,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT));sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages','/usr/lib/python3/dist-packages'])
from tools.lib.grasp_yaw_kinematics import GraspYawKinematics
from tools.lib.so101_mesh_pinch import load_closure_guard
from tools.lib import towel_bimanual_then_single_planning as geometry
from tools.lib import towel_task_pose_planning as task
from tools.lib import desk_task_planning as planning
from tools.run.diagnose_towel_fold_kinematics import solve_phases


def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
 if a.output.exists():ap.error('refusing overwrite')
 b=ROOT/'artifacts/bimanual/planning/so101_surface_matched_pad_20260906';recipe=json.loads((b/'validated_native_recipe.json').read_text())
 for path,digest in recipe['input_sha256'].items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,path
 assert hashlib.sha256(Path(recipe['result']).read_bytes()).hexdigest()==recipe['result_sha256']
 cloth=np.array(json.loads(Path(recipe['result']).read_text())['final_cloth_shape_local_m_env_0']);bounds=[float(cloth[:,0].min()),float(cloth[:,0].max()),float(cloth[:,1].min()),float(cloth[:,1].max())]
 reference=json.loads((b/'second_fold_native/plan_stack_observed_support.json').read_text());clear=reference['clear_model_rad'];pad=json.loads((b/'geometry.json').read_text());urdf=ROOT/'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf';kin={s:GraspYawKinematics(urdf,s+'_') for s in ['left','right']};closure={};local=reference['contact_local_point_m']
 for side,k in kin.items():
  # Only the task reference changes in memory. Registered collision origins,
  # camera mount meshes and URDF bytes remain unchanged.
  k._tcp_chain[-1].origin.xyz=list(local)
  arm=clear[:5] if side=='left' else clear[6:11];pos=dict(zip(k.arm_joints,arm));_,xyz=k.tcp_pose_in_root(pos);finger=k.finger_axis_in_root(pos)
  geometry.OBSERVE_CLEAR_TCP_BY_ARM_M[side]=tuple(xyz);geometry.OBSERVE_CLEAR_JAW_YAW_BY_ARM_RAD[side]=math.atan2(finger[1],finger[0])
  guard,_=load_closure_guard(pad,urdf,ROOT,side);closure[side]=guard.derive_stop(-.023567,reference['closure']['open_model_angle_rad'])
 # Contact is an intended LOW endpoint, not an assertion that the untouched
 # arch remains at that height. Native descent must assess compressed layers.
 geometry.SECOND_BIMANUAL_LEFT_CONTACT_HEIGHT_ADDITION_M=.003-(-.005+geometry.SECOND_LAYER_TCP_Z_OFFSET_M)
 geometry.SECOND_BIMANUAL_RIGHT_CONTACT_HEIGHT_ADDITION_M=geometry.SECOND_BIMANUAL_LEFT_CONTACT_HEIGHT_ADDITION_M
 phases,final=geometry.build_bimanual_second_fold(bounds,-.005,direction='left_to_right')
 result={'status':'BIMANUAL_CURRENT_S1_IK_RUNNING','simulation_only':True,'motion_commands':0,'source_result':recipe['result'],'source_result_sha256':recipe['result_sha256'],'pad_path':str(b/'geometry.json'),'pad_sha256':hashlib.sha256((b/'geometry.json').read_bytes()).hexdigest(),'urdf_path':str(urdf),'source_hashes':recipe['input_sha256'],'planning_reference_in_gripper_m':local,'clear_model_rad':clear,'closure':closure,'first_fold_bounds_xyxy_m':bounds,'assignment':{'left':{'region':'lower-X end of high-Y free edge (user left/lower in reviewed view)','jaw_yaw_rad':math.pi/2},'right':{'region':'higher-X end of same free edge (user left/upper in reviewed view)','jaw_yaw_rad':0}},'arch_handling':'Use original S1 nodes. A later physical descent may compress the S1 arch; require both layers to remain graspable afterward. No prescribed flattening.','expected_final_bounds':final,'collision_checked':False,'cloth_contact_validated':False,'one_flip_completed':False,'phases':[]}
 try:
  records=solve_phases(phases,tuple(clear),kin,{s:planning.load_arm_joint_bounds(s) for s in kin},prefer_continuous_seed=True)
  for record in records:
   record['attachment_event']=None
   record['joint_positions_rad'][5]=closure['left']['open_model_angle_rad'];record['joint_positions_rad'][11]=closure['right']['open_model_angle_rad']
  result.update(status='BIMANUAL_CURRENT_S1_IK_PASS_COLLISION_PENDING',phases=records)
 except Exception as error:result.update(status='BIMANUAL_CURRENT_S1_IK_BRANCH',reason=str(error))
 a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'status':result['status'],'phase_count':len(result['phases']),'reason':result.get('reason'),'bounds':bounds},indent=2))
if __name__=='__main__':main()
