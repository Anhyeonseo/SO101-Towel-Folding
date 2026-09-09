#!/usr/bin/env python3
"""Bounded frozen-state contact-balance audit and one small lateral correction.

No cloth or controller changes. A frozen-cloth geometric witness is not a
prediction that friction will allow the same relative displacement in motion.
"""
import argparse,copy,hashlib,json,sys
from pathlib import Path
import numpy as np
import trimesh
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages','/usr/lib/python3/dist-packages'])
from tools.lib.so101_mesh_pinch import load_closure_guard
from tools.lib.so101_towel_edge_alignment import ContactPairPlanner


def closest(mesh,points):
    try:return trimesh.proximity.closest_point(mesh,points)
    except ModuleNotFoundError:return trimesh.proximity.closest_point_naive(mesh,points)


def contact_state(points,capture,region,core,pad_mesh,moving_mesh):
    ids=sorted(set(sum(region.values(),[])))
    xyz=points[ids]
    pf,df,_=closest(pad_mesh,xyz);pm,dm,im=closest(moving_mesh,xyz)
    nf=(xyz-pf)/np.maximum(df[:,None],1e-12)
    signed_m=np.einsum('ij,ij->i',xyz-pm,moving_mesh.face_normals[im])
    # The fixed mesh consists only of the actual finite OUTER face. Require
    # its inward-X contact direction, matching the native boundary criterion.
    fixed=(nf[:,0]>=.5)&(df<.003)
    moving=(signed_m>=-1e-7)&(dm<.003)
    margins=df[fixed]
    retained={str(h):[p for i,p in enumerate(ids) if p in region[h] and fixed[i] and moving[i]] for h in region}
    reserve={str(h):max([min(.003-df[i],.003-dm[i]) for i,p in enumerate(ids) if p in core[h] and fixed[i] and moving[i]] or [-1.]) for h in core}
    motion={str(h):float(np.linalg.norm(points[indices]-capture[indices],axis=1).max()) for h,indices in core.items()}
    return {'retained_particles':retained,'core_contact_reserve_m':reserve,
            'minimum_active_fixed_margin_m':float(margins.min()) if len(margins) else None,
            'core_motion_m':motion,'passed_retention_geometry':bool(all(retained.values()) and max(motion.values())<=.003),
            'points':[{'particle':p,'fixed_distance_m':float(df[i]),'fixed_normal_x':float(nf[i,0]),'moving_distance_m':float(dm[i]),'moving_outside_projection_m':float(signed_m[i])} for i,p in enumerate(ids)]}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trial',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    if a.output.exists():ap.error('refusing to overwrite a plan')
    r=json.loads((a.trial/'result.json').read_text());source=json.loads((a.trial/'input.json').read_text());base=source['plan']
    assert r['active_phase']=='fold_02' and base['status']=='S2_NATIVE_CANDIDATE_STATIC_PASS'
    assert not source['observe_core_exceedance']
    for name,digest in source['source_hashes'].items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
    pad=json.loads(Path(base['pad_path']).read_text());urdf=ROOT/'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf'
    guard,hashes=load_closure_guard(pad,urdf,ROOT,'left')
    vertices=np.array(pad['vertices_m']);faces=np.array(pad['faces']);outer=faces[np.all(np.isclose(vertices[faces][...,0],pad['outer_x_m'],atol=1e-8),axis=1)]
    fixed=trimesh.Trimesh(vertices=vertices,faces=outer,process=False)
    def moving(angle):
        tri=guard.triangles@(guard.rotation@Rotation.from_rotvec(guard.axis*angle).as_matrix()).T+guard.translation
        return trimesh.Trimesh(vertices=tri.reshape(-1,3),faces=np.arange(tri.size//3).reshape(-1,3),process=False)
    branch=np.load(a.trial/'branch_material_local.npy');capture=np.load(a.trial/'capture_material_local.npy')
    region={int(h):v for h,v in r['captured_material_particles'].items()};core={int(h):v for h,v in r['initial_bilateral_particles'].items()}
    stages={x['name']:x for x in r['stages']};qb=np.array(stages['contact_branch']['actual_joint_positions_rad'][:5]);angle=stages['contact_branch']['actual_joint_positions_rad'][5];mesh=moving(angle)
    planner=ContactPairPlanner(ROOT,urdf,'left',[base['contact_local_point_m']],pad,base['closure']['minimum_model_angle_rad'],base['closure']['open_model_angle_rad']);k=planner.k;local=np.array(base['contact_local_point_m'])
    def pose(q):return k._compose(planner.chain,dict(zip(k.arm_joints,q)))
    def fit(q,shift):
        R,t=pose(q);target=R@local+t+R[:,0]*shift
        def residual(qnew):
            rr,tt=pose(qnew)
            return np.r_[(rr@local+tt-target)/.0001,Rotation.from_matrix(rr@R.T).as_rotvec()/.02]
        sol=least_squares(residual,q,bounds=(planner.lo+1e-6,planner.hi-1e-6),max_nfev=100,ftol=1e-11,xtol=1e-11,gtol=1e-11)
        rr,tt=pose(sol.x)
        return sol.x,{'position_error_m':float(np.linalg.norm(rr@local+tt-target)),'rotation_error_deg':float(np.degrees(Rotation.from_matrix(rr@R.T).magnitude()))}
    Rb,tb=pose(qb)
    candidates=[];selected=None
    # One bounded, one-dimensional scan, 0..1 mm; no trajectory/material sweep.
    for shift in -np.arange(0,1.0001,.05)*.001:
        qnew,ik=fit(qb,float(shift));rr,tt=pose(qnew)
        adjusted=(branch-(tt-tb)@Rb)@(Rb.T@rr)
        state=contact_state(adjusted,capture,region,core,fixed,mesh)
        margin=state['minimum_active_fixed_margin_m']
        passed=state['passed_retention_geometry'] and margin is not None and margin>=.0006 and min(state['core_contact_reserve_m'].values())>=.00015 and ik['position_error_m']<=.00005 and ik['rotation_error_deg']<=1
        item={'shift_local_x_m':float(shift),'ik':ik,'contact':state,'passed_with_reserve':bool(passed)};candidates.append(item)
        if passed and selected is None:selected=item
    audit={'trial':str(a.trial.resolve()),'result_sha256':hashlib.sha256((a.trial/'result.json').read_bytes()).hexdigest(),'mesh_hashes':hashes,'jaw_angle_unchanged_rad':angle,'frozen_shape_assumption':True,'scan':candidates,'selected':selected,'scope':'Reachable small hand translation with frozen material coordinates. Does not prove slip dynamics, force balance, hardware success or full-robot collision clearance.'}
    result=copy.deepcopy(base);result['status']='S2_NATIVE_CANDIDATE_STATIC_BRANCH';result['contact_balance']=audit
    if selected:
        shift=selected['shift_local_x_m'];phases=copy.deepcopy(base['phases']);checks=copy.deepcopy(base['pose_checks']);angles=np.linspace(planner.closed,planner.open,9)
        probe=next(x for x in phases if x['name']=='load_probe');qp=np.array(probe['q_rad']);balanced,ik=fit(qp,shift)
        phases.insert(phases.index(probe)+1,{'name':'contact_balance','q_rad':balanced.tolist(),'seconds':.25})
        checks.append({'name':'contact_balance',**ik,'passed':bool(ik['position_error_m']<=.00005 and ik['rotation_error_deg']<=1)})
        for phase in phases:
            if phase['name'].startswith('fold_') or phase['name']=='retreat':
                phase['q_rad'],ik=fit(np.array(phase['q_rad']),shift);phase['q_rad']=phase['q_rad'].tolist()
                check=next(c for c in checks if c['name']==phase['name']);check['original_target_m']=check.get('target_m');check['target_m']=planner.points(phase['q_rad'])[0].tolist();check.update(ik);check['passed']=bool(ik['position_error_m']<=.00005 and ik['rotation_error_deg']<=1)
        segments=[]
        for first,last in zip(phases,phases[1:]):
            q0,q1=np.array(first['q_rad']),np.array(last['q_rad'])
            clear=min(planner.clearance(q0+u*(q1-q0),v) for u in np.linspace(0,1,41) for v in angles)
            segments.append({'from':first['name'],'to':last['name'],'minimum_table_clearance_m':clear,'passed':bool(clear>=.00025)})
        for check in checks:
            phase=next(p for p in phases if p['name']==check['name'])
            check['q_rad']=phase['q_rad']
            check['minimum_table_clearance_m']=min(planner.clearance(phase['q_rad'],v) for v in angles)
            check['passed']=bool(check['passed'] and check['minimum_table_clearance_m']>=.00025)
            if phase['name'].startswith('fold_') or phase['name'] in ('contact_balance','retreat'):
                check['rotation_error_reference']='corresponding original phase orientation'
        # Verify the proposed slow correction at the previously measured probe
        # endpoint, before any fold motion, against frozen cloth geometry too.
        sp=stages['load_probe_hold'];qactual=np.array(sp['actual_joint_positions_rad'][:5]);Rp,tp=pose(qactual)
        probeworld=np.load(a.trial/'load_probe_hold.npy');probemesh=moving(sp['actual_joint_positions_rad'][5]);path=[]
        for u in np.linspace(0,1,11):
            rr,tt=pose(qactual+u*(balanced-qactual));points=(probeworld-tt)@rr
            e=contact_state(points,capture,region,core,fixed,probemesh);path.append({'fraction':float(u),'passed_retention_geometry':e['passed_retention_geometry'],'core_motion_m':e['core_motion_m'],'retained_particles':e['retained_particles']})
        # FK/world registration is independently checked against exact saved
        # branch local coordinates; reject if the model frame differs.
        fk_error=float(np.max(np.linalg.norm((np.load(a.trial/'contact_branch.npy')-tb)@Rb-branch,axis=1)))
        audit.update(correction_seconds=.25,hold_after_correction_s=.15,retention_origin_reset=False,probe_correction_path=path,branch_fk_registration_error_m=fk_error,correction_applied_to='after load probe, then all fold phases and retreat')
        result.update(phases=phases,pose_checks=checks,segment_checks=segments)
        if all(c['passed'] for c in checks+segments) and all(c['passed_retention_geometry'] for c in path) and fk_error<1e-5:result['status']='S2_NATIVE_CANDIDATE_STATIC_PASS'
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'status':result['status'],'shift_local_x_m':selected['shift_local_x_m'] if selected else None,'branch_contact':selected['contact'] if selected else None,'probe_path':audit.get('probe_correction_path'),'fk_error_m':audit.get('branch_fk_registration_error_m'),'failed_checks':[c for c in result['pose_checks']+result['segment_checks'] if not c['passed']]},indent=2))
if __name__=='__main__':main()
