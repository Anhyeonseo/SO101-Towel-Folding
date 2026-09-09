"""Read-only end-substep force-law audit for Newton 1.2.1 one-way VBD."""
import numpy as np
from scipy.spatial.transform import Rotation


def force_law(depth, normal, displacement, ke, kd, mu, epsilon, dt):
    """Reevaluate Newton's penalty/damping/regularized friction law in NumPy.

    This is an end-state model estimate, not an integrated impulse or measured
    robot reaction. The caller supplies the actual final-substep coefficients.
    """
    values=np.r_[depth,normal,displacement,ke,kd,mu,epsilon,dt]
    if not np.isfinite(values).all() or min(ke,kd,mu)<0 or min(epsilon,dt)<=0:
        raise ValueError('invalid force audit inputs')
    n=np.asarray(normal);u=np.asarray(displacement)
    if not np.isclose(np.linalg.norm(n),1,atol=1e-4):raise ValueError('non-unit contact normal')
    if depth<=0:return dict(normal_elastic_n=0.,normal_damping_n=0.,tangential_n=0.,friction_utilization=0.,relative_tangent_speed_m_s=0.,force_world_n=[0.,0.,0.])
    normal_elastic=depth*ke;un=float(n@u);damping=-min(un,0.)*kd*ke/dt
    tangent=u-n*un;length=np.linalg.norm(tangent);eps=epsilon*dt
    scale=0. if length==0 else (1/length if length>eps else (2-length/eps)/eps)
    ft=-mu*normal_elastic*scale*tangent;capacity=mu*normal_elastic
    return dict(normal_elastic_n=float(normal_elastic),normal_damping_n=float(damping),tangential_n=float(np.linalg.norm(ft)),friction_utilization=float(np.linalg.norm(ft)/capacity) if capacity>0 else 0.,relative_tangent_speed_m_s=float(length/dt),force_world_n=(n*(normal_elastic+damping)+ft).tolist())


def capture_native_force_audit(manager, model, region, core):
    from isaaclab_contrib.deformable.coupled_mjwarp_vbd_manager import NewtonCoupledMJWarpVBDManager
    soft=NewtonCoupledMJWarpVBDManager._soft_solver
    contacts=manager.get_contacts();current=manager.get_state();previous=manager._state_1
    if not soft.integrate_with_external_rigid_solver or manager._num_substeps%2:
        raise RuntimeError('force audit requires verified one-way even-substep state buffers')
    count=int(contacts.soft_contact_count.numpy()[0]);shape=contacts.soft_contact_shape.numpy()[:count];particles=contacts.soft_contact_particle.numpy()[:count]
    surface=contacts.soft_contact_body_pos.numpy()[:count];normal=contacts.soft_contact_normal.numpy()[:count];body_vel=contacts.soft_contact_body_vel.numpy()[:count]
    q=current.particle_q.numpy();qprev=soft.particle_q_prev.numpy();body=current.body_q.numpy();bodyprev=previous.body_q.numpy();shape_body=model.shape_body.numpy();radii=model.particle_radius.numpy()
    k=soft.body_particle_contact_penalty_k.numpy();kd=soft.body_particle_contact_material_kd.numpy();mu=soft.body_particle_contact_material_mu.numpy();dt=manager._solver_dt
    member={int(i):int(h) for h,ids in region.items() for i in ids};records=[]
    for i in range(count):
        particle=int(particles[i]);label=model.shape_label[int(shape[i])]
        if particle not in member or '/left_' not in label:continue
        jaw='fixed' if 'TowelFixedJawCollider' in label else 'moving' if 'moving_jaw_link/' in label else None
        if jaw is None:continue
        bi=int(shape_body[int(shape[i])]);R=Rotation.from_quat(body[bi,3:]).as_matrix();Rp=Rotation.from_quat(bodyprev[bi,3:]).as_matrix()
        now=R@surface[i]+body[bi,:3];before=Rp@surface[i]+bodyprev[bi,:3]
        displacement=q[particle]-qprev[particle]-(now-before)-R@body_vel[i]*dt
        distance=float(normal[i]@(q[particle]-now));depth=float(radii[particle]-distance)
        if depth<=0:continue
        record=dict(particle=particle,layer=member[particle],jaw=jaw,shape=label,is_core=particle in core[member[particle]],signed_distance_m=distance,depth_m=depth,stiffness_n_m=float(k[i]),damping_coefficient=float(kd[i]),friction_coefficient=float(mu[i]),**force_law(depth,normal[i],displacement,k[i],kd[i],mu[i],soft.friction_epsilon,dt));records.append(record)
    summary={}
    for h in (0,1):
        summary[str(h)]={}
        for jaw in ('fixed','moving'):
            cs=[c for c in records if c['layer']==h and c['jaw']==jaw]
            summary[str(h)][jaw]={'contacts':len(cs),'sum_normal_elastic_n':sum(c['normal_elastic_n'] for c in cs),'sum_tangential_n':sum(c['tangential_n'] for c in cs),'maximum_friction_utilization':max([c['friction_utilization'] for c in cs],default=0.),'resultant_force_world_n':np.sum([c['force_world_n'] for c in cs],axis=0).tolist() if cs else [0.,0.,0.]}
    return {'scope':'last native substep, end-state reevaluation of installed VBD force law; 60 Hz sampling; no integrated impulse, motor reaction, or calibrated physical grip force','substep_dt_s':dt,'summary':summary,'contacts':records}
