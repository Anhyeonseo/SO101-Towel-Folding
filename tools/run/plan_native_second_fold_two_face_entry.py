#!/usr/bin/env python3
"""Fit pre-grasp pose against both finite jaw faces at unchanged S1 material points.

The predicted closed-jaw angle is a geometric witness only. Runtime closure
still uses native contacts and the existing compression/retention guards.
"""
import argparse,copy,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.optimize import minimize,least_squares
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages','/usr/lib/python3/dist-packages'])
from tools.lib.so101_mesh_pinch import load_closure_guard
from tools.lib.so101_towel_edge_alignment import ContactPairPlanner


def planar_moving_face(guard):
    x=float(guard.triangles[:,:,0].min())
    triangles=guard.triangles[np.all(abs(guard.triangles[:,:,0]-x)<1e-7,axis=1)]
    if not len(triangles):raise ValueError('moving jaw has no registered minimum-X plane')
    hull=ConvexHull(np.unique(triangles[:,:,1:].reshape(-1,2),axis=0))
    area=np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1).sum()/2
    if not np.isclose(area,hull.volume,rtol=1e-5):raise ValueError('moving plane is not a filled convex face')
    return x,hull.equations


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--base-plan',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--margin-limit-seed',type=Path,help='One bounded search maximizing moving-face interior margin, preserving all other constraints');a=ap.parse_args()
    if a.output.exists():ap.error('refusing to overwrite plan')
    base=json.loads(a.base_plan.read_text());assert base['status']=='S2_NATIVE_CANDIDATE_STATIC_PASS' and not base.get('grasp_probe')
    recipe=json.loads((ROOT/'artifacts/bimanual/planning/so101_surface_matched_pad_20260906/validated_native_recipe.json').read_text())
    for path,digest in recipe['input_sha256'].items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,path
    assert hashlib.sha256(Path(base['source_result']).read_bytes()).hexdigest()==base['source_result_sha256']
    pad=json.loads(Path(base['pad_path']).read_text());urdf=ROOT/'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf';guard,hashes=load_closure_guard(pad,urdf,ROOT,'left');moving_x,moving_eq=planar_moving_face(guard)
    local=np.array(base['contact_local_point_m']);pl=ContactPairPlanner(ROOT,urdf,'left',[local],pad,base['closure']['minimum_model_angle_rad'],base['closure']['open_model_angle_rad']);k=pl.k
    old={p['name']:p for p in base['phases']};oldchecks={p['name']:p for p in base['pose_checks']};q0=np.array(old['contact']['q_rad'])
    pose=lambda q:k._compose(pl.chain,dict(zip(k.arm_joints,q)))
    R0,t0=pose(q0);target=R0@local+t0
    cloth=np.array(json.loads(Path(base['source_result']).read_text())['final_cloth_shape_local_m_env_0']).reshape(64,64,3)
    row=base['contact_row'];columns=[base['contact_column'],base['paired_bottom_column']];layers=cloth[row,columns];angles=np.linspace(pl.closed,pl.open,9)
    def data(v):
        R,t=pose(v[:5]);fixed=(layers-t)@R;jointR=guard.rotation@Rotation.from_rotvec(guard.axis*v[5]).as_matrix();moving=(fixed-guard.translation)@jointR
        fm=-(fixed[:,1:]@guard.equations[:,:2].T+guard.equations[:,2]).max(1)
        mm=-(moving[:,1:]@moving_eq[:,:2].T+moving_eq[:,2]).max(1)
        return R,t,fm,mm,fixed[:,0]-pad['outer_x_m'],moving_x-moving[:,0]
    def constraints(v):
        R,t,fm,mm,fd,md=data(v)
        return np.r_[(fm-.0008)*1000,(mm-.0008)*1000,(fd-.0008)*1000,(.0026-fd)*1000,(md-.0008)*1000,(.0026-md)*1000,[(pl.clearance(v[:5],angle)-.0003)*1000 for angle in angles],(.003-np.linalg.norm(R@local+t-target))*1000,(R[2,2]-np.cos(np.radians(20)))*10,(np.radians(15)-Rotation.from_matrix(R@R0.T).magnitude())*10]
    def loss(v):
        R,t,fm,mm,fd,md=data(v)
        return float(np.sum(((R@local+t-target)/.002)**2)+np.sum((Rotation.from_matrix(R@R0.T).as_rotvec()/.2)**2)+.2*np.sum(((fd-md)/.002)**2))
    initial=np.r_[q0,.0608623]
    if a.margin_limit_seed:
        prior=json.loads(a.margin_limit_seed.read_text())['two_face_entry'];seed=np.r_[prior['q_rad'],prior['predicted_closure_angle_rad']]
        objective=lambda v:float(-1000*min(data(v)[3])+.001*loss(v))
        hard=lambda v:np.delete(constraints(v),[2,3])
    else:seed=initial;objective=loss;hard=constraints
    sol=minimize(objective,seed,method='SLSQP',bounds=list(zip(pl.lo+1e-6,pl.hi-1e-6))+[(pl.closed,.12)],constraints=[{'type':'ineq','fun':hard}],options={'maxiter':180,'ftol':1e-9})
    def summarize(v):
        R,t,fm,mm,fd,md=data(v)
        return {'q_rad':v[:5].tolist(),'predicted_closure_angle_rad':float(v[5]),'fixed_face_interior_margins_m':fm.tolist(),'moving_face_interior_margins_m':mm.tolist(),'fixed_plane_distances_m':fd.tolist(),'moving_plane_distances_m':md.tolist(),'contact_reference_shift_m':(R@local+t-target).tolist(),'tilt_from_vertical_deg':float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))),'rotation_from_previous_entry_deg':float(np.degrees(Rotation.from_matrix(R@R0.T).magnitude())),'minimum_scaled_constraint':float(constraints(v).min())}
    fit=summarize(sol.x);fit.update(method='both registered finite opposing faces fitted BEFORE direct top entry; no post-capture seating or translation',optimizer_message=str(sol.message),iterations=int(sol.nit),baseline=summarize(initial),material_indices=[row*64+c for c in columns],base_plan=str(a.base_plan.resolve()),base_plan_sha256=hashlib.sha256(a.base_plan.read_bytes()).hexdigest(),mesh_hashes=hashes,retention_origin_reset=False,closure_controller_override=False)
    fit['passed']=bool(constraints(sol.x).min()>=-1e-4)
    fit['margin_limit_search']=bool(a.margin_limit_seed)
    fit['minimum_other_constraint']=float(hard(sol.x).min())
    fit['global_optimum_certified']=False
    result=copy.deepcopy(base);result.update(status='S2_NATIVE_CANDIDATE_STATIC_BRANCH',two_face_entry=fit)
    if fit['passed']:
        qc=sol.x[:5];R,t=pose(qc);contact=R@local+t;phases=[];checks=[]
        def solve(name,target,Rdes,seed,seconds):
            def residual(q):
                rr,tt=pose(q)
                return np.r_[(rr@local+tt-target)/.00025,Rotation.from_matrix(rr@Rdes.T).as_rotvec()/.15]
            s=least_squares(residual,np.clip(seed,pl.lo+1e-6,pl.hi-1e-6),bounds=(pl.lo+1e-6,pl.hi-1e-6),max_nfev=180,ftol=1e-10,xtol=1e-10,gtol=1e-10);rr,tt=pose(s.x);err=float(np.linalg.norm(rr@local+tt-target));rot=float(np.degrees(Rotation.from_matrix(rr@Rdes.T).magnitude()));clear=min(pl.clearance(s.x,angle) for angle in angles)
            phases.append(dict(name=name,q_rad=s.x.tolist(),seconds=seconds));checks.append(dict(name=name,q_rad=s.x.tolist(),target_m=target.tolist(),position_error_m=err,rotation_error_deg=rot,minimum_table_clearance_m=clear,passed=bool(err<=.001 and rot<=(1 if name in ['staging','lift_probe'] else 30) and clear>=.00025)));return s.x
        solve('staging',contact+np.array([0,0,.03]),R,qc,.6)
        phases.append(dict(name='contact',q_rad=qc.tolist(),seconds=.6));clear=min(pl.clearance(qc,angle) for angle in np.linspace(pl.closed,pl.open,81));checks.append(dict(name='contact',q_rad=qc.tolist(),target_m=contact.tolist(),position_error_m=0.,rotation_error_deg=0.,minimum_table_clearance_m=clear,passed=bool(clear>=.00025)))
        q=solve('lift_probe',contact+np.array([0,0,.01]),R,qc,.6)
        Rf,_=pose(old['fold_09']['q_rad']);slerp=Slerp([0.,1.],Rotation.from_matrix([R,Rf]))
        for i in range(1,10):
            name=f'fold_{i:02}';q=solve(name,np.array(oldchecks[name]['target_m']),slerp(i/9).as_matrix(),q,old[name]['seconds'])
        phases.append(copy.deepcopy(old['retreat']));checks.append(copy.deepcopy(oldchecks['retreat']))
        segments=[]
        for first,last in zip(phases,phases[1:]):
            qa,qb=np.array(first['q_rad']),np.array(last['q_rad']);clear=min(pl.clearance(qa+u*(qb-qa),angle) for u in np.linspace(0,1,41) for angle in angles);segments.append(dict(from_phase=first['name'],to_phase=last['name'],minimum_table_clearance_m=clear,passed=bool(clear>=.00025)))
        result.update(phases=phases,pose_checks=checks,segment_checks=segments,opening_checks={'entry_minimum_table_clearance_m':segments[0]['minimum_table_clearance_m'],'stationary_closure_minimum_table_clearance_m':next(c['minimum_table_clearance_m'] for c in checks if c['name']=='contact'),'passed':all(c['passed'] for c in segments[:1]+[next(c for c in checks if c['name']=='contact')])})
        result['contact_clearance_fit']={'method':fit['method'],'layer_face_interior_margins_m':fit['fixed_face_interior_margins_m'],'moving_face_interior_margins_m':fit['moving_face_interior_margins_m'],'material_target_m':layers[0].tolist(),'approach_tilt_deg':fit['tilt_from_vertical_deg']}
        if all(c['passed'] for c in checks+segments):result['status']='S2_NATIVE_CANDIDATE_STATIC_PASS'
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'status':result['status'],'fit':fit,'failed':[c for c in result['pose_checks']+result['segment_checks'] if not c['passed']]},indent=2))
if __name__=='__main__':main()
