"""Simulation material-pair alignment with URDF limits and table-clearance checks."""
from pathlib import Path
import json
import numpy as np


def mirrored_material_indices(indices, grid_side):
    if grid_side < 2 or any(i < 0 or i >= grid_side**2 for i in indices):
        raise ValueError('invalid cloth material indices')
    return [(i//grid_side)*grid_side + grid_side-1-i%grid_side for i in indices]


def aligned_pair_target(cloth, indices, local_pair, *, height_m=.006):
    cloth=np.asarray(cloth,dtype=float);n=int(round(len(cloth)**.5))
    if n*n!=len(cloth): raise ValueError('square cloth topology required')
    mirror=mirrored_material_indices(indices,n);opposite=cloth[mirror]
    delta=opposite[1,:2]-opposite[0,:2];norm=np.linalg.norm(delta)
    if norm < 1e-6: raise ValueError('opposite material pair is collapsed')
    length=np.linalg.norm(np.asarray(local_pair)[1]-local_pair[0])
    center=opposite.mean(0);center[2]=opposite[:,2].max()+height_m
    direction=np.r_[delta/norm,0.]
    return np.array([center-direction*length/2,center+direction*length/2]),mirror


class ContactPairPlanner:
    def __init__(self, root, urdf, side, local_pair, pad, closed_angle, open_angle):
        import trimesh
        from tools.lib.grasp_yaw_kinematics import GraspYawKinematics,_rpy_matrix
        self.side=side;self.local_pair=np.asarray(local_pair);self.closed=closed_angle;self.open=open_angle
        self.k=GraspYawKinematics(urdf,side+'_');self.chain=self.k._build_chain('workcell_base_link',side+'_gripper_link')
        limits=json.loads((Path(root)/'config/bimanual_operational_limits.json').read_text())['arms'][side]
        names=['base','shoulder','elbow','wrist_flex','wrist_roll']
        self.lo=np.array([limits[n]['minimum_urad']*1e-6 for n in names]);self.hi=np.array([limits[n]['maximum_urad']*1e-6 for n in names])
        self.geometry=[]
        for link in self.k._robot.links:
            if not link.name.startswith(side+'_') or not any(j.name in self.k.arm_joints for j in self.k._build_chain('workcell_base_link',link.name)):continue
            for c in link.collisions or []:
                if not hasattr(c.geometry,'filename'):continue
                mesh=trimesh.load_mesh(Path(root)/'ros2_ws/src/so101_description/meshes'/Path(c.geometry.filename).name)
                v=mesh.convex_hull.vertices@_rpy_matrix(*c.origin.rpy).T+np.array(c.origin.xyz)
                self.geometry.append((self.k._build_chain('workcell_base_link',link.name),v))
        self.geometry.append((self.chain,np.asarray(pad['vertices_m'])))

    def points(self,q):
        R,t=self.k._compose(self.chain,dict(zip(self.k.arm_joints,q)))
        return self.local_pair@R.T+t

    def clearance(self,q,angle):
        pos=dict(zip(self.k.arm_joints,q));pos[self.side+'_gripper_joint']=angle
        result=float('inf')
        for chain,v in self.geometry:
            R,t=self.k._compose(chain,pos);result=min(result,float((v@R.T+t)[:,2].min()+.005))
        return result

    def plan(self,start,target,*,opening=False,already_open=False):
        from scipy.optimize import minimize
        start=np.asarray(start);target=np.asarray(target)
        angles=np.linspace(self.closed,self.open,9) if opening else [self.open if already_open else self.closed]
        def loss(q):
            error=self.points(q)-target
            # Match the cloth in XY; permit a small upward clearance correction.
            return float(np.sum((error[:,:2]*1000)**2)+.1*np.sum((error[:,2]*1000)**2)+1e-6*np.sum((q-start)**2))
        def constraints(q):
            return np.r_[(self.points(q)[:,2]-target[:,2]) * 1000,
                         [(self.clearance(q,a)-.0005)*1000 for a in angles]]
        fit=minimize(loss,np.clip(start,self.lo,self.hi),method='SLSQP',bounds=list(zip(self.lo,self.hi)),constraints=[{'type':'ineq','fun':constraints}],options={'maxiter':180,'ftol':1e-9})
        q=fit.x;error=self.points(q)-target
        minimum=min(self.clearance(start+a*(q-start), self.open if already_open else self.closed) for a in np.linspace(0,1,31))
        if opening: minimum=min(minimum,min(self.clearance(q,a) for a in np.linspace(self.closed,self.open,41)))
        xy=float(np.linalg.norm(error[:,:2],axis=1).max());z=float(error[:,2].max())
        passed=bool(xy<=.001 and -.00001<=error[:,2].min() and z<=.02 and minimum>=.00025)
        return q,{'passed':passed,'maximum_xy_error_m':xy,'maximum_upward_offset_m':z,'minimum_sampled_table_clearance_m':minimum,'optimizer_message':str(fit.message),'q_rad':q.tolist(),'target_pair_m':target.tolist(),'achieved_pair_m':self.points(q).tolist()}


def surface_drag_phase_offset(name, delta, *, start_after_l_phase=4):
    """Preserve the successful surface-drag half-slip compensation geometry.

    A free-edge shift d advances the L turnaround by d/2, lowers it by d/2,
    and advances the final held edge by d. The return arc uses the same
    quarter-ellipse interpolation as the accepted surface-drag trajectory.
    """
    delta = np.asarray(delta, dtype=float)
    if delta.shape != (3,) or not np.isfinite(delta).all():
        raise ValueError('finite three-dimensional correction required')
    if start_after_l_phase not in range(7):
        raise ValueError('correction must start before the low sweep')
    if name.startswith('first_form_l_'):
        index = int(name.rsplit('_', 1)[1])
        if not start_after_l_phase < index <= 9:
            raise ValueError('L phase outside the remaining correction path')
        # Original successful path: five diagonal steps, one vertical step,
        # then three low horizontal steps. Normalize only the remaining path.
        base = min(start_after_l_phase, 5) / 14
        fx = index / 14 if index <= 5 else 5/14 + (index-6)/21
        fz = index / 14 if index <= 5 else .5
        wx = .5 * (fx-base) / (.5-base)
        wz = (-.5*(index-6)/3 if start_after_l_phase == 6
              else -.5*(fz-base)/(.5-base))
        wy = 2*wx
    elif name.startswith('first_gravity_laydown_'):
        index = int(name.rsplit('_', 1)[1])
        if not 1 <= index <= 9:
            raise ValueError('nine-step gravity laydown required')
        theta = index*np.pi/18
        wx = .5*(1+np.sin(theta)); wz = -.5*np.cos(theta); wy = 1.
    elif name.startswith(('first_gravity_overcenter_', 'first_gravity_preopen_clearance_')):
        wx, wy, wz = 1., 1., 0.
    else:
        raise ValueError('unsupported phase for surface-drag compensation')
    return np.array([wx*delta[0], wy*delta[1], wz*delta[0]])


def plan_observed_anchor_shift(phases, planners, starts, cloth, pair_indices,
                               *, compensate_surface_drag=False):
    """Shift the remaining L formation and fold arc to the observed free half.

    The shift is introduced over the remaining L-formation waypoints, before
    the return arc begins. Plans are returned only after every side and every
    segment has passed geometric checks; callers must not execute partial plans.
    """
    import copy
    nodes = np.asarray(cloth)
    n = int(round(len(nodes) ** .5))
    if n*n != len(nodes) or not phases:
        raise ValueError('square cloth and remaining phases required')
    ramp_count = sum(p['name'].startswith('first_form_l_') for p in phases)
    if not ramp_count:
        raise ValueError('anchor correction must start during L formation')
    result = copy.deepcopy(phases)
    report = {'passed': False, 'sides': {}, 'checks': [],
              'method': 'half_slip_turnaround_and_full_slip_release'
                        if compensate_surface_drag else 'uniform_path_translation'}
    start_after = int(phases[0]['name'].rsplit('_', 1)[1])-1
    for si, side in enumerate(('left', 'right')):
        planner = planners[side]
        offset = 6*si
        end_q = np.asarray(phases[-1]['joint_positions_rad'][offset:offset+5])
        mirror = mirrored_material_indices(pair_indices[side], n)
        desired = nodes[mirror]
        delta = np.r_[desired[:, :2].mean(0) - planner.points(end_q)[:, :2].mean(0), 0.]
        report['sides'][side] = {'shift_m': delta.tolist(), 'mirror_indices': mirror,
                                 'observed_pair_m': desired.tolist()}
        if np.linalg.norm(delta[:2]) > .06 or desired[:, 2].max() > .005:
            report['reason'] = ('observed mirrored pair is not yet table supported'
                                if desired[:, 2].max() > .005
                                else 'required endpoint correction exceeds 60 mm')
            return None, report
        previous = np.asarray(starts[side])
        for i, (phase, updated) in enumerate(zip(phases, result, strict=True)):
            baseline = np.asarray(phase['joint_positions_rad'][offset:offset+5])
            weight = min(1., (i+1)/ramp_count)
            shift = (surface_drag_phase_offset(phase['name'], delta,
                         start_after_l_phase=start_after)
                     if compensate_surface_drag else weight*delta)
            target = planner.points(baseline) + shift
            q, check = planner.plan(previous, target)
            # A translated path must preserve height as well as XY.
            check['maximum_position_error_m'] = float(np.linalg.norm(planner.points(q)-target,axis=1).max())
            check['passed'] = check['passed'] and check['maximum_position_error_m'] <= .001
            check.update(side=side, phase=phase['name'], applied_shift_m=shift.tolist())
            report['checks'].append(check)
            if not check['passed']:
                report['reason'] = 'translated fold segment failed preflight'
                return None, report
            updated['joint_positions_rad'][offset:offset+5] = q.tolist()
            updated['arm_joint_positions_rad'][side] = q.tolist()
            previous = q
    report['passed'] = True
    return result, report


def observed_fold_balance(cloth):
    """Measure excess upper-leaf material length at the maximum-X fold.

    For an inextensible U fold, advancing the held leaf by its excess length
    moves the fold by half that distance and removes the length imbalance.
    This supplies a geometric correction, not a fitted release offset.
    """
    nodes = np.asarray(cloth, dtype=float)
    n = int(round(len(nodes) ** .5))
    if n*n != len(nodes) or n < 8 or not np.isfinite(nodes).all():
        raise ValueError('finite square material grid with at least eight nodes per side required')
    grid = nodes.reshape(n,n,3)
    rows = grid[n//8:n-n//8]
    imbalances=[]; directions=[]; folds=[]
    for row in rows:
        fold=int(np.argmax(row[:,0]))
        if not n//4 <= fold <= 3*n//4:
            raise ValueError('a single central fold is required before balance correction')
        segments=np.linalg.norm(np.diff(row,axis=0),axis=1)
        # Sub-node crest estimate avoids a two-cell jump in the length
        # difference when alternating mesh rows choose adjacent peak nodes.
        a,b,c=row[fold-1:fold+2,0]
        curvature=a-2*b+c
        fraction=float(np.clip(.5*(a-c)/curvature,-.5,.5)) if curvature < -1e-12 else 0.
        material_crest=fold+fraction
        cumulative=np.r_[0.,np.cumsum(segments)]
        upper=float(np.interp(material_crest,np.arange(n),cumulative))
        imbalances.append(2*upper-float(cumulative[-1]))
        direction=row[fold,:2]-row[-1,:2]
        if np.linalg.norm(direction)<.05:
            raise ValueError('free half is not sufficiently extended')
        directions.append(direction/np.linalg.norm(direction));folds.append(material_crest)
    direction=np.median(directions,axis=0);direction/=np.linalg.norm(direction)
    return {'upper_minus_lower_length_m':float(np.median(imbalances)),
            'per_row_imbalance_m':imbalances,'median_fold_column':float(np.median(folds)),
            'advance_direction_xy':direction.tolist()}
