#!/usr/bin/env python3
"""One bounded native-contact S2 trial using the verified S1 scene initializer.

The S1 runner is loaded as a module without running its S1 trajectory. This
entry point replaces only the isolated S2 experiment callback. The original
S1 source and its verified input hashes remain unchanged. Saved cloth shape
and zero velocity initialize this experiment, not a complete solver restart.
"""
import argparse,hashlib,importlib.util,json,sys,time,traceback
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from tools.lib.so101_mesh_pinch import ActualMeshPinchGate
from tools.lib.so101_second_fold_initialization import diagnostic_initial_shape, initial_shape_digest


def closure_step_count(open_rad, minimum_rad, maximum_speed_rad_s, dt_s):
 values=[open_rad,minimum_rad,maximum_speed_rad_s,dt_s]
 if not np.isfinite(values).all() or open_rad<=minimum_rad or min(maximum_speed_rad_s,dt_s)<=0:
  raise ValueError('finite ordered closure angles and positive speed/time required')
 return max(1,int(np.ceil((open_rad-minimum_rad)/(maximum_speed_rad_s*dt_s))))


def edge_feed_progress(closure_fraction, complete_fraction):
 """Complete the short insertion before normal pinch capture, without jumps."""
 if not np.isfinite([closure_fraction,complete_fraction]).all() or not 0<complete_fraction<=1 or not 0<=closure_fraction<=1:
  raise ValueError('finite closure fraction and ordered insertion timing required')
 return min(closure_fraction/complete_fraction,1.)


def layered_gate(contacts,gate,*,row=59,columns=None,friction_cone_check=True):
 result={}
 for half in (0,1):
  selected=[c for c in contacts if (int(c['particle'])%64>=32)==bool(half)
            and abs(int(c['particle'])//64-row)<=3
            and (columns is None or abs(int(c['particle'])%64-columns[half])<=3)]
  result[str(half)]=gate.evaluate(selected,'left',.02,allow_same_particle=True,
      allow_outer_face_boundary=True,friction_cone_check=friction_cone_check)
 return {'passed':all(x['passed'] for x in result.values()),'layers':result}


def qualified_capture_patch(contacts,gate,*,columns,row=59):
 """Freeze only native same-particle pinches that qualify at capture time.

 Every included point independently passes the original friction-cone gate.
 This set is never enlarged during transport, even when other contacts appear.
 """
 result={}
 for half in (0,1):
  nearby=[c for c in contacts if '/left_' in c['shape']
          and (int(c['particle'])%64>=32)==bool(half)
          and abs(int(c['particle'])//64-row)<=3
          and abs(int(c['particle'])%64-columns[half])<=3]
  result[half]=[]
  for particle in sorted({int(c['particle']) for c in nearby}):
   evidence=gate.evaluate([c for c in nearby if c['particle']==particle],
       'left',.02,allow_same_particle=True,allow_outer_face_boundary=True,
       friction_cone_check=True)
   if evidence['passed']:result[half].append(particle)
  if not result[half]:raise RuntimeError('no individually qualified initial material contacts for a layer')
 return result


def initial_core_if_ready(contacts,evidence,gate,*,row=59):
 """Do not approve a capture that the existing immutable-core check rejects."""
 if not evidence['passed']:return None
 columns={h:int(evidence['layers'][str(h)]['selected_distinct_particles'][0])%64 for h in (0,1)}
 try:return qualified_capture_patch(contacts,gate,columns=columns,row=row)
 except RuntimeError as error:
  if str(error)=='no individually qualified initial material contacts for a layer':return None
  raise


def retained_surface_contacts(contacts,indices,*,allow_contact_redistribution=False):
 """Check native outer-face/STL contact on the frozen captured material set.

 No force-angle surrogate: this is used only with measured relative motion
 and an actual lift test. Neither speculative contacts nor rigid penetration
 of particle centres nor replacement material particles count as retention.
 """
 layers={}
 for h,ids in indices.items():
  records=[];retained=[]
  for particle in ids:
   fixed=[];moving=[]
   for c in contacts:
    if c['particle']!=particle or '/left_' not in c['shape']:continue
    if not (c['penetration_m']>0 and c['signed_distance_m']>=-1e-7):continue
    if 'TowelFixedJawCollider' in c['shape'] and abs(c['surface_local_m'][0]+.0057)<=1e-6 and c['normal_local'][0]>=.5:fixed.append(c)
    elif 'moving_jaw_link/' in c['shape'] and 'moving_jaw_so101' in c['shape']:moving.append(c)
   if fixed and moving:retained.append(particle)
   records.extend(fixed+moving)
  ok=bool(retained) if allow_contact_redistribution else bool(ids) and len(retained)==len(ids)
  layers[str(h)]={'passed':ok,'actual_contacts':records,'retained_particles':retained}
 return {'passed':all(x['passed'] for x in layers.values()),'layers':layers}


def initial_surface_patch(contacts,*,columns,row=59):
 """Freeze actual initial contact with either jaw, including unilateral support.

 The grasp itself still needs the separate strict two-layer pinch gate.
 Unilateral support at capture may later become bilateral; speculative contact,
 embedded centres and new material points are never added to this region.
 """
 candidates={h:sorted({int(c['particle']) for c in contacts
                      if (int(c['particle'])%64>=32)==bool(h)
                      and abs(int(c['particle'])//64-row)<=3
                      and abs(int(c['particle'])%64-columns[h])<=3}) for h in (0,1)}
 geometry=retained_surface_contacts(contacts,candidates,allow_contact_redistribution=True)
 return {h:sorted({int(c['particle']) for c in geometry['layers'][str(h)]['actual_contacts']}) for h in (0,1)}


def material_retention(current_local,capture_local,indices,geometry_gate,limit_m=.003):
 """Measured material retention, not an analytic force-closure certificate.

 The originally captured nodes must stay within 3 mm in the actual fixed-jaw
 frame (less than one cloth grid cell), AND each original sheet must retain
 actual contact on both jaw surfaces within that frozen set. The surface gate
 specifies whether all original nodes or at least one per layer must contact.
 The motion bound continues to apply to ALL registered initial contact nodes.
 Rigid hand motion therefore cannot be mistaken for slip or for cloth lift.
 """
 slip={str(h):float(np.linalg.norm(current_local[ids]-capture_local[ids],axis=1).max())
       for h,ids in indices.items()}
 return {'passed':bool(geometry_gate['passed'] and all(np.isfinite(v) and v<=limit_m for v in slip.values())),
         'layer_material_slip_m':slip,'limit_m':limit_m,'opposing_geometry_passed':geometry_gate['passed']}


def retention_closure_guard(contacts,indices,minimum_margin_m=.0005):
 """Prevent further tightening near the pad; never relax retention checks.

 Include embedded centres in the minimum so they cannot disappear from this
 control guard just because the independent retention gate rejects them.
 No opening impulse or cloth constraint is introduced by this guard.
 """
 margins={}
 for h,ids in indices.items():
  distances=[float(c['signed_distance_m']) for c in contacts
             if c['particle'] in ids and '/left_' in c['shape']
             and 'TowelFixedJawCollider' in c['shape']
             and abs(c['surface_local_m'][0]+.0057)<=1e-6
             and c['normal_local'][0]>=.5 and c['penetration_m']>0]
  margins[str(h)]=min(distances) if distances else None
 return {'allow_additional_closure':all(v is not None and np.isfinite(v) and v>minimum_margin_m for v in margins.values()),
         'minimum_layer_pad_margin_m':margins,'minimum_margin_m':minimum_margin_m}


def core_region_retention(current_local,capture_local,core,region,geometry_gate,limit_m=.003):
 """Bound every initial strict bilateral seed; observe regional deformation.

 Both sets are frozen at capture, before transport. The unchanged contact
 gate requires bilateral support per layer within the initial region. A core
 point cannot be replaced, even if another region point later supports load.
 """
 if not all(core[h] and set(core[h]).issubset(region[h]) for h in (0,1)):
  raise ValueError('nonempty initial bilateral core must belong to its frozen region')
 result=material_retention(current_local,capture_local,core,geometry_gate,limit_m)
 delta=current_local-capture_local
 region_motion={str(h):float(np.linalg.norm(delta[ids],axis=1).max()) for h,ids in region.items()}
 deformation={str(h):float(np.linalg.norm(delta[ids]-delta[core[h]].mean(0),axis=1).max()) for h,ids in region.items()}
 result.update(region_maximum_relative_motion_m=region_motion,
               region_deformation_relative_to_core_m=deformation,
               motion_bound_applies_to='all immutable initial strict bilateral particles',
               region_deformation_is_diagnostic=True)
 result['passed']=bool(result['passed'] and all(np.isfinite(v) for v in list(region_motion.values())+list(deformation.values())))
 return result


def retention_step_accepted(measured,*,observe_core_exceedance=False):
 """Diagnostic continuation never changes the recorded retention verdict.

 Even in diagnostic mode, require opposing contact within the frozen region
 and finite motion. Only the stop on finite core displacement is deferred.
 """
 values=list(measured['layer_material_slip_m'].values())+list(measured.get('region_maximum_relative_motion_m',{}).values())
 return bool(measured['passed'] or (observe_core_exceedance and measured['opposing_geometry_passed'] and all(np.isfinite(v) for v in values)))


def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--plan',type=Path,required=True);parser.add_argument('--output-dir',type=Path,required=True);parser.add_argument('--gui',action='store_true');parser.add_argument('--measured-retention',action='store_true',help='After strict initial capture, use observed slip plus opposing contacts for a bounded native-physics experiment; keep friction-cone diagnostics');parser.add_argument('--capture-patch-retention',action='store_true',help='Freeze initial actual contacts with either jaw; require ongoing bilateral contact within that region');parser.add_argument('--guard-retention-closure',action='store_true',help='Stop additional tightening when initial contact nodes approach the fixed pad within 0.5 mm; keep all retention checks');parser.add_argument('--separate-core-retention',action='store_true',help='Keep the 3 mm bound on all initial strict bilateral seeds and record surrounding region deformation separately');parser.add_argument('--observe-core-exceedance',action='store_true',help='Diagnostic only: log 3 mm failures but continue while frozen-region bilateral contact remains; never mark retention validated');opt=parser.parse_args()
 if opt.observe_core_exceedance and not (opt.separate_core_retention and opt.measured_retention):parser.error('diagnostic continuation requires separate measured core retention')
 if opt.separate_core_retention and not (opt.measured_retention and opt.capture_patch_retention):parser.error('separate core retention requires measured frozen-region retention')
 if opt.capture_patch_retention and not opt.measured_retention:parser.error('capture-patch retention requires measured-retention checks')
 if opt.guard_retention_closure and not (opt.measured_retention and opt.capture_patch_retention):parser.error('closure guard requires measured frozen-region retention')
 plan=json.loads(opt.plan.read_text());assert plan['status']=='S2_NATIVE_CANDIDATE_STATIC_PASS' and plan['active_arm']=='left' and plan['motion_authorized'] is False
 out=opt.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
 source=Path(plan['source_result']);assert hashlib.sha256(source.read_bytes()).hexdigest()==plan['source_result_sha256']
 initial=diagnostic_initial_shape(json.loads(source.read_text())['final_cloth_shape_local_m_env_0'],plan.get('diagnostic_towel_x_shift_m',0.))
 if 'initial_cloth_sha256' in plan:assert initial_shape_digest(initial)==plan['initial_cloth_sha256']
 assert hashlib.sha256(Path(plan['pad_path']).read_bytes()).hexdigest()==plan['pad_sha256']
 base=ROOT/'artifacts/bimanual/planning/so101_surface_matched_pad_20260906';recipe=json.loads((base/'validated_native_recipe.json').read_text())
 for name,digest in recipe['input_sha256'].items():
  if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest:raise RuntimeError('S1 input changed: '+name)
 argv=list(recipe['argv'][1:]);argv[argv.index('--output')+1]=str(out/'unused_s1_result.json')
 if not opt.gui:
  # Headless setup captures CUDA before the GUI-only buffer compaction hook.
  # Retain full contact storage; collision flags and physics stay identical.
  argv=[a for a in argv if a!='--compact-soft-contact-buffers']
 # Runtime argument parsing remains that of the verified S1 scene. No old S2
 # contact targets or forged legacy MoveIt pass document are supplied.
 if opt.gui:argv=[a for a in argv if a!='--headless'];argv+=['--viz','kit','--live-render-pacing']
 (out/'input.json').write_text(json.dumps({'plan':plan,'plan_sha256':hashlib.sha256(opt.plan.read_bytes()).hexdigest(),'scene_argv':argv,'measured_retention':opt.measured_retention,'capture_patch_retention':opt.capture_patch_retention,'guard_retention_closure':opt.guard_retention_closure,'separate_core_retention':opt.separate_core_retention,'observe_core_exceedance':opt.observe_core_exceedance,'source_hashes':recipe['input_sha256'],'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},indent=2)+'\n')
 (out/'executed_runner.py').write_bytes(Path(__file__).read_bytes())
 sys.argv=argv
 spec=importlib.util.spec_from_file_location('so101_native_s2_scene',ROOT/argv[0])
 module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module
 spec.loader.exec_module(module);env=vars(module)
 # Select the established early experiment hook before S1 settling/approach.
 env['args'].second_fold_contact_only=True
 env['args'].keep_open=False
 report={'status':'RUNNING','motion_commands':0,'motion_authorized':False,'cloth_nodes_constrained':False,'s1_trajectory_replayed':False,'initialization':'accepted S1 nodal shape with zero velocities; not full solver checkpoint','stages':[],'contact_monitor':{'checks':0,'max_consecutive_loss_steps':0},'source_result_sha256':plan['source_result_sha256'],'pad_sha256':plan['pad_sha256']}
 report['diagnostic_core_motion_continuation']=opt.observe_core_exceedance
 report['retention_validated']=False if opt.observe_core_exceedance else None
 report.update(diagnostic_towel_x_shift_m=plan.get('diagnostic_towel_x_shift_m',0.),initial_cloth_sha256=initial_shape_digest(initial),towel_repositioning_motion_executed=False)
 if plan.get('diagnostic_towel_x_shift_m',0.):report['initialization']='diagnostically translated copy of accepted S1 shape, zero velocities; no physical repositioning motion or full solver checkpoint'
 np.save(out/'requested_initial_cloth.npy',initial)
 started=time.monotonic()
 recording={'cloth':[],'body_q':[],'joints':[],'time_s':[],'phase':[]}
 def save():
  report['elapsed_s']=time.monotonic()-started;(out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 def experiment(*,scene,sim,robot,cloth,joint_ids,source,environment_count,physics_dt_s,**unused):
  import torch
  from scipy.spatial.transform import Rotation
  manager=env['NewtonManager'];model=manager.get_model();gate=ActualMeshPinchGate(model.tri_indices.numpy())
  tensor=torch.tensor(initial,dtype=cloth.data.nodal_pos_w.torch.dtype,device=sim.device)
  state=torch.cat((tensor.unsqueeze(0)+scene.env_origins[:,None,:],torch.zeros_like(tensor).unsqueeze(0)),dim=-1)
  cloth.write_nodal_state_to_sim_index(state)
  row=torch.tensor([plan['clear_model_rad']],dtype=torch.float32,device=sim.device);zero=torch.zeros_like(row);phases={p['name']:p for p in plan['phases']};row[0,:5]=torch.tensor(phases['staging']['q_rad'],device=sim.device)
  env['write_scripted_arm_state_and_drive_targets'](robot,row,zero,joint_ids,initialize_arm_state=True)
  nodes=lambda:manager.get_state().particle_q.numpy()
  snap=lambda:env['newton_soft_contact_snapshot'](fresh_geometry=True)
  fixed_shapes=[i for i,label in enumerate(model.shape_label) if '/left_' in label and 'TowelFixedJawCollider' in label]
  assert len(fixed_shapes)==1
  fixed_body=int(model.shape_body.numpy()[fixed_shapes[0]])
  report['recording_body_labels']=list(model.body_label)
  tick=0
  force_audit_enabled=bool(plan.get('grasp_probe'))
  def local_nodes():
   transform=manager.get_state().body_q.numpy().reshape(-1,7)[fixed_body]
   return (nodes()-transform[:3])@Rotation.from_quat(transform[3:]).as_matrix()
  minq=plan['closure']['minimum_model_angle_rad'];openq=plan['closure']['open_model_angle_rad'];loss=0;columns=None;holding=False;lower_limit=minq;last={}
  def event(name,**fields):
   record={'name':name,'actual_joint_positions_rad':robot.data.joint_pos.torch[0,joint_ids].cpu().tolist(),**fields};report['stages'].append(record);np.save(out/(name+'.npy'),nodes());print('S2_NATIVE '+json.dumps(record),flush=True);save()
  def read_gate():
   nonlocal last
   s=snap();last=layered_gate(s['jaw_contact_records'],gate,row=plan.get('contact_row',59),columns=columns)
   return s,last
  def step():
   nonlocal loss,tick,force_audit_enabled
   env['write_scripted_arm_state_and_drive_targets'](robot,row,zero,joint_ids)
   scene.write_data_to_sim();sim.step();scene.update(physics_dt_s)
   tick+=1
   if tick%8==0:
    recording['cloth'].append(nodes().copy());recording['body_q'].append(manager.get_state().body_q.numpy().copy())
    recording['joints'].append(robot.data.joint_pos.torch[0,joint_ids].cpu().numpy().copy());recording['time_s'].append(tick*physics_dt_s);recording['phase'].append(report.get('active_phase','initializing'))
   if not torch.isfinite(cloth.data.nodal_pos_w.torch).all():raise RuntimeError('nonfinite cloth')
   if holding:
    s,e=read_gate();report['contact_monitor']['checks']+=1
    geometry=retained_surface_contacts(s['jaw_contact_records'],indices,allow_contact_redistribution=opt.capture_patch_retention)
    current_local=local_nodes()
    measured=(core_region_retention(current_local,capture_local,core_indices,indices,geometry)
              if opt.separate_core_retention else material_retention(current_local,capture_local,indices,geometry))
    report['last_material_retention']=measured
    report['last_retained_material_particles']={h:v['retained_particles'] for h,v in geometry['layers'].items()}
    monitor=report['contact_monitor'];monitor['friction_criterion_failed_steps']=monitor.get('friction_criterion_failed_steps',0)+int(not e['passed'])
    monitor['maximum_material_slip_m']=max(monitor.get('maximum_material_slip_m',0.),max(measured['layer_material_slip_m'].values()))
    if not measured['passed']:
     monitor['failed_measured_retention_steps']=monitor.get('failed_measured_retention_steps',0)+1
     if opt.observe_core_exceedance:report['retention_validated']=False
    accepted=retention_step_accepted(measured,observe_core_exceedance=opt.observe_core_exceedance) if opt.measured_retention else e['passed']
    loss=0 if accepted else loss+1
    report['contact_monitor']['max_consecutive_loss_steps']=max(loss,report['contact_monitor']['max_consecutive_loss_steps'])
    depths=[c['penetration_m'] for x in e['layers'].values() if x['passed'] for c in x['actual_contacts']]
    closure_guard=retention_closure_guard(s['jaw_contact_records'],indices)
    report['last_closure_guard']=closure_guard
    if plan.get('grasp_probe'):
     blocking=[]
     for c in s['jaw_contact_records']:
      h=int(c['particle']%64>=32)
      if c['particle'] in indices[h] and 'TowelFixedJawCollider' in c['shape'] and c['normal_local'][0]>=.5 and abs(c['surface_local_m'][0]+.0057)<=1e-6 and c['penetration_m']>0 and c['signed_distance_m']<=.0005:
       blocking.append({'particle':c['particle'],'layer':h,'is_core':c['particle'] in core_indices[h],'distance_m':c['signed_distance_m']})
     closure_guard['blocking_contacts']=blocking
     if force_audit_enabled and tick%4==0:
      try:
       from tools.lib.so101_s2_contact_force_audit import capture_native_force_audit
       report.setdefault('force_trace',[]).append({'time_s':tick*physics_dt_s,'phase':report.get('active_phase'),**capture_native_force_audit(manager,model,indices,core_indices)})
      except Exception as error:
       report['force_audit_error']=repr(error);force_audit_enabled=False
    report.setdefault('retention_trace',[]).append({'time_s':tick*physics_dt_s,'phase':report.get('active_phase'),
      **measured,'retained_particles':report['last_retained_material_particles'],'closure_guard':closure_guard})
    close_requested=not e['passed'] or (depths and min(depths)<.0012)
    if close_requested and (not opt.guard_retention_closure or closure_guard['allow_additional_closure']):row[0,5]=max(lower_limit,float(row[0,5])-.02*physics_dt_s)
    elif close_requested:monitor['prevented_tightening_steps']=monitor.get('prevented_tightening_steps',0)+1
    if loss*physics_dt_s>.02:
     np.save(out/'branch_material_local.npy',current_local)
     report['last_contact_gate']=e;report['last_jaw_contacts']=s['jaw_contact_records'];event('contact_branch',gate=e,material_retention=measured);raise RuntimeError('measured material retention failed for more than 20 ms' if opt.measured_retention else 'one S1 layer failed the sufficient opposing-contact friction criterion for more than 20 ms')
  def move(name,duration=None):
   report['active_phase']=name
   target=torch.tensor(phases[name]['q_rad'],dtype=row.dtype,device=sim.device);begin=row[0,:5].clone();count=max(2,round((duration or phases[name]['seconds'])/physics_dt_s))
   for i in range(1,count+1):row[0,:5]=begin+(i/count)*(target-begin);step()
   precise_entry=bool(plan.get('entry_standoff_m',0.) or plan.get('edge_feed')) and name in ('entry_clear','entry_feed','contact')
   arrival_limit=.002 if precise_entry else .02
   for i in range(round((.5 if precise_entry else .25)/physics_dt_s)):
    if env['maximum_arm_target_residual_rad'](robot,row,joint_ids)<arrival_limit:break
    step()
   env['require_arm_target_reached'](robot,row,joint_ids,name)
   if precise_entry and env['maximum_arm_target_residual_rad'](robot,row,joint_ids)>arrival_limit:raise RuntimeError('side entry did not settle within 0.002 rad before next motion')
   event(name,jaw_model_rad=float(row[0,5]),gate=last if holding else None,material_retention=report.get('last_material_retention'))
  for i in range(round(.2/physics_dt_s)):step()
  drift=float(np.linalg.norm(nodes()-initial,axis=1).max());event('initialized',maximum_node_drift_m=drift)
  if drift>.005:raise RuntimeError('saved S1 shape is not stable enough for isolated S2 initialization (5 mm drift)')
  if 'entry_clear' in phases:move('entry_clear')
  move('entry_feed' if plan.get('edge_feed') else 'contact')
  for i in range(round(.2/physics_dt_s)):step()
  event('preclose',jaw_contacts=snap()['jaw_contact_records'])
  preclose=nodes().copy();stable=0;found=False;count=closure_step_count(openq,minq,plan.get('closing_speed_rad_s',openq-minq),physics_dt_s);report['active_phase']='closure'
  report['closure_timing']={'steps':count,'maximum_duration_s':count*physics_dt_s,'speed_rad_s':(openq-minq)/(count*physics_dt_s)}
  feed_begin=row[0,:5].clone();feed_target=torch.tensor(phases['contact']['q_rad'],dtype=row.dtype,device=sim.device)
  for i in range(1,count+1):
   if plan.get('edge_feed'):
    progress=edge_feed_progress(i/count,plan['edge_feed']['insertion_complete_closure_fraction'])
    row[0,:5]=feed_begin+progress*(feed_target-feed_begin)
   row[0,5]=openq+(minq-openq)*i/count;step();s,e=read_gate();stable=stable+1 if e['passed'] else 0
   if i%12==0:report.setdefault('closure_trace',[]).append({'fraction':i/count,'model_rad':float(row[0,5]),'gate':e})
   if stable>=8:
    if opt.capture_patch_retention and initial_core_if_ready(s['jaw_contact_records'],e,gate,row=plan.get('contact_row',59)) is None:
     report['capture_waited_for_existing_core_condition_steps']=report.get('capture_waited_for_existing_core_condition_steps',0)+1
     continue
    found=True;break
  report['last_contact_gate']=e;report['last_jaw_contacts']=s['jaw_contact_records']
  report['initial_native_jaw_contacts']=s['jaw_contact_records']
  event('pinch',passed=found,model_rad=float(row[0,5]),gate=e)
  if not found:raise RuntimeError('safe closure limit reached without opposing contacts on both S1 layers; pinch geometry must change')
  if float(row[0,5])>plan.get('maximum_transport_gripper_model_rad',openq):raise RuntimeError('capture opening exceeds verified transport envelope')
  columns={h:int(e['layers'][str(h)]['selected_distinct_particles'][0])%64 for h in (0,1)}
  qcapture=float(row[0,5]);lower_limit=max(minq,qcapture-.04)
  holding=True;capture=nodes().copy();capture_local=local_nodes();lift_indices={h:e['layers'][str(h)]['selected_distinct_particles'] for h in (0,1)}
  indices=initial_surface_patch(s['jaw_contact_records'],columns=columns,row=plan.get('contact_row',59)) if opt.capture_patch_retention else lift_indices
  if not all(set(lift_indices[h]).issubset(indices[h]) for h in (0,1)):raise RuntimeError('initial surface region excludes strict pinch seeds')
  report['initial_bilateral_particles']=qualified_capture_patch(s['jaw_contact_records'],gate,columns=columns,row=plan.get('contact_row',59)) if opt.capture_patch_retention else lift_indices
  core_indices={int(h):list(ids) for h,ids in report['initial_bilateral_particles'].items()}
  report['capture_time_s']=tick*physics_dt_s
  np.save(out/'capture_material_local.npy',capture_local)
  report['captured_material_particles']=indices
  report['contact_redistribution_allowed']=opt.capture_patch_retention
  report['capture_set_can_expand']=False
  report['transport_validation']='observed material slip <=3 mm and ongoing opposing contacts on both layers; not force closure certified' if opt.measured_retention else 'pairwise sufficient friction criterion'
  if opt.capture_patch_retention:report['transport_validation']='all initial native surface-contact nodes stay within 3 mm; at least one node per layer from that frozen region retains simultaneous positive outer-face/STL contact; initial strict pinch required; initial set never expands; not a force-closure certificate'
  if opt.separate_core_retention:report['transport_validation']='all initial strict bilateral core nodes stay within 3 mm; ongoing simultaneous bilateral contact per layer within immutable initial surface region; surrounding deformation separately recorded; no force-closure certificate'
  event('captured_patch',material_particles=indices,original_lift_particles=lift_indices)
  # Freeze the core at first stable capture even if it precedes insertion's end.
  # Any remaining insertion is then subject to the same retention monitor.
  if plan.get('edge_feed'):move('contact')
  report['active_phase']='post_capture_hold'
  for i in range(round(.15/physics_dt_s)):step()
  move('lift_probe');lifts={str(h):float(np.median(nodes()[ids,2]-capture[ids,2])) for h,ids in lift_indices.items()};event('lift_check',layer_lift_m=lifts)
  if min(lifts.values())<.005:raise RuntimeError('both S1 layers did not rise at least 5 mm in the 10 mm probe')
  if plan.get('grasp_probe'):
   probe=plan['grasp_probe'];report['active_phase']='lifted_hold'
   for i in range(round(probe['hold_after_lift_s']/physics_dt_s)):step()
   event('lifted_hold',duration_s=probe['hold_after_lift_s'],material_retention=report.get('last_material_retention'))
   move('load_probe');report['active_phase']='load_probe_hold'
   for i in range(round(probe['hold_after_probe_s']/physics_dt_s)):step()
   event('load_probe_hold',duration_s=probe['hold_after_probe_s'],material_retention=report.get('last_material_retention'))
   if probe['stop_after_probe']:
    report['status']='S2_NATIVE_GRASP_HOLD_LOAD_PROBE_PASSED';report['one_flip_completed']=False
    event('grasp_probe_completed');return 0
  if 'contact_balance' in phases:
   move('contact_balance');report['active_phase']='contact_balance_hold'
   duration=plan['contact_balance']['hold_after_correction_s']
   for i in range(round(duration/physics_dt_s)):step()
   event('contact_balance_hold',duration_s=duration,material_retention=report.get('last_material_retention'))
  if 'seat' in phases:move('seat')
  for name in phases:
   if name.startswith('fold_'):move(name)
  for i in range(round(.3/physics_dt_s)):step()
  before=nodes().copy();support=float(np.mean(before[:,2]<=.006));event('laydown',supported_fraction=support)
  if support<.25:raise RuntimeError('less than 25 percent of cloth supported before release')
  holding=False;releaseq=float(row[0,5])
  release_open=plan.get('release_open_model_rad',openq)
  for i in range(1,61):row[0,5]=releaseq+(release_open-releaseq)*i/60;step()
  env['set_explicit_newton_fixed_pad_friction'](0.)
  for i in range(round(.25/physics_dt_s)):step()
  move('retreat');final=nodes();np.save(out/'final_cloth.npy',final)
  grid=final.reshape(64,64,3);residual=float(np.median(grid[-1,:,1])-np.median(grid[0,:,1]));height=float(final[:,2].max()+.005)
  report['shape']={'signed_edge_residual_m':residual,'maximum_height_m':height,'span_xy_m':np.ptp(final[:,:2],axis=0).tolist(),'camera_observation_used':False,'source':'simulation material-edge oracle after retreat'}
  report['status']='S2_NATIVE_FOLD_RELEASED_REOBSERVATION_PENDING' if height<=.03 else 'S2_NATIVE_FOLD_RELEASED_SHAPE_BRANCH'
  if opt.observe_core_exceedance:report['status']='S2_DIAGNOSTIC_ARC_RELEASED_NOT_RETENTION_VALIDATED'
  event('released_result',**report['shape']);return 0
 env['run_second_fold_contact_only']=experiment
 try:env['run']()
 except Exception as error:
  report['status']='S2_NATIVE_BRANCH_STOPPED';report['reason']=str(error);print('S2_NATIVE_BRANCH '+str(error),flush=True);traceback.print_exc();save()
 finally:
  if recording['time_s']:
   np.savez_compressed(out/'native_recording.npz',**{k:np.asarray(v) for k,v in recording.items()})
   report['recording']={'path':str(out/'native_recording.npz'),'frames':len(recording['time_s']),'physics_recomputation_required_for_playback':False}
  save();env['simulation_app'].close()
 print(json.dumps({'status':report['status'],'reason':report.get('reason'),'elapsed_s':report['elapsed_s'],'output':str(out)},indent=2))
 return 0 if report['status'].startswith('S2_NATIVE_FOLD_RELEASED') or report['status']=='S2_NATIVE_GRASP_HOLD_LOAD_PROBE_PASSED' else 2
if __name__=='__main__':raise SystemExit(main())
