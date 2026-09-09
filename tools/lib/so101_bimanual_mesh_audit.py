"""Read-only full registered-mesh distance audit, including both wrist mounts.

No cross-arm pair is excluded by SRDF. Same-arm exclusions are inherited from
the saved SRDF; added rubber pads are still checked against their moving jaws.
Distances certify modeled geometry only, not tracking error or cloth contact.
"""
from pathlib import Path
import hashlib,xml.etree.ElementTree as ET
import numpy as np
import trimesh,coal
from scipy.spatial.transform import Rotation
from tools.lib.grasp_yaw_kinematics import GraspYawKinematics


def mesh_model(vertices,faces):
    vv=coal.StdVec_Vec3s();tt=coal.StdVec_Triangle()
    for v in vertices:vv.append(np.asarray(v,dtype=float))
    for a,b,c in faces:tt.append(coal.Triangle(int(a),int(b),int(c)))
    m=coal.BVHModelOBBRSS();m.beginModel(len(faces),len(vertices));m.addSubModel(vv,tt);m.endModel();return m


def aabb_distance(a,b):
    delta=np.maximum(np.maximum(a[0]-b[1],b[0]-a[1]),0.)
    return float(np.linalg.norm(delta))


class BimanualMeshAudit:
    def __init__(self,root,urdf,pad,table):
        self.k=GraspYawKinematics(urdf,'left_');self.entries=[];self.hashes={str(urdf):hashlib.sha256(Path(urdf).read_bytes()).hexdigest()}
        srdf=Path(root)/'ros2_ws/src/so101_moveit_config/config/so101_dual.srdf'
        self.hashes[str(srdf)]=hashlib.sha256(srdf.read_bytes()).hexdigest()
        allowed={frozenset((p.get('link1'),p.get('link2'))) for p in ET.parse(srdf).getroot().findall('disable_collisions')}
        def add(name,link,vertices,faces,kind=None,solid=None,object_offset=None):
            chain=self.k._build_chain('workcell_base_link',link) if link else []
            side='left' if link and link.startswith('left_') else 'right' if link and link.startswith('right_') else 'world'
            m=mesh_model(vertices,faces) if solid is None else solid;obj=coal.CollisionObject(m)
            vertices=np.asarray(vertices);lo,hi=vertices.min(0),vertices.max(0)
            corners=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
            radius=float(np.linalg.norm(vertices,axis=1).max());bounds={};suffix=radius
            for j in reversed(chain):
                if j.type in ('revolute','continuous'):bounds[j.name]=suffix
                suffix+=np.linalg.norm(j.origin.xyz) if j.origin and j.origin.xyz else 0.
            self.entries.append({'name':name,'link':link,'side':side,'chain':chain,'obj':obj,'corners':corners,'motion_radii':bounds,'kind':kind,'object_offset':np.zeros(3) if object_offset is None else np.asarray(object_offset)})
        robot=ET.parse(urdf).getroot()
        for link in robot.findall('link'):
            for i,c in enumerate(link.findall('collision')):
                geo=c.find('geometry');mesh=geo.find('mesh')
                if mesh is not None:
                    file=mesh.get('filename');prefix='package://so101_description/'
                    if not file.startswith(prefix):raise ValueError('unregistered mesh URI')
                    path=Path(root)/'ros2_ws/src/so101_description'/file[len(prefix):];self.hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest();m=trimesh.load_mesh(path,process=False);v=np.array(m.vertices)*np.fromstring(mesh.get('scale','1 1 1'),sep=' ');f=m.faces;label=Path(file).name
                elif geo.find('box') is not None:
                    m=trimesh.creation.box(np.fromstring(geo.find('box').get('size'),sep=' '));v=m.vertices;f=m.faces;label='box'
                else:raise ValueError('unsupported collision geometry; refuse incomplete audit')
                origin=c.find('origin');R=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix() if origin is not None else np.eye(3);t=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
                add(f'{link.get("name")}:{i}:{label}',link.get('name'),v@R.T+t,f)
        for side in ['left','right']:add(side+':rubber_pad',side+'_gripper_link',pad['vertices_m'],pad['faces'],'pad')
        box=trimesh.creation.box(table['size_xyz_m']);add('worktable',None,box.vertices+np.array(table['pose_xyz_m']),box.faces,'table',coal.Box(*table['size_xyz_m']),table['pose_xyz_m'])
        self.pairs=[]
        for i,a in enumerate(self.entries):
            for j,b in enumerate(self.entries[i+1:],i+1):
                cross={a['side'],b['side']}=={'left','right'}
                table_pair=a['kind']=='table' or b['kind']=='table'
                pad_jaw=(a['kind']=='pad' and b['link']==a['side']+'_moving_jaw_link') or (b['kind']=='pad' and a['link']==b['side']+'_moving_jaw_link')
                if a['side']==b['side']=='world':continue
                if not cross and not table_pair and not pad_jaw and (a['link']==b['link'] or frozenset((a['link'],b['link'])) in allowed):continue
                if table_pair and (a['side']=='world' and b['side']=='world'):continue
                threshold=.005 if cross else .00025 if table_pair or pad_jaw else .000001
                self.pairs.append((i,j,threshold,'interarm' if cross else 'table' if table_pair else 'self'))
        mounts=[e['name'] for e in self.entries if 'wrist_cam_mount' in e['name']]
        if not any(n.startswith('left_') for n in mounts) or not any(n.startswith('right_') for n in mounts):raise ValueError('both camera mounts required')
        self.mounts=mounts;self.request=coal.DistanceRequest();self.request.enable_nearest_points=True
    def state(self,q):
        q=np.asarray(q,dtype=float)
        if q.shape!=(12,) or not np.isfinite(q).all():raise ValueError('12 finite joint positions required')
        names=[side+'_'+name+'_joint' for side in ['left','right'] for name in ['base','shoulder','elbow','wrist_flex','wrist_roll','gripper']];pos=dict(zip(names,q));boxes=[]
        for e in self.entries:
            R,t=self.k._compose(e['chain'],pos);e['obj'].setTransform(coal.Transform3s(R,t+R@e['object_offset']));e['obj'].computeAABB();v=e['corners']@R.T+t;boxes.append((v.min(0),v.max(0)))
        minimum={};failures=[]
        for i,j,threshold,kind in self.pairs:
            broad=aabb_distance(boxes[i],boxes[j])
            # Retain a conservative lower bound when far apart. Exact query
            # near all acceptance boundaries and near the recorded minimum.
            if broad>max(.02,threshold*2):dist=broad;exact=False;points=None
            else:
                result=coal.DistanceResult();dist=float(coal.distance(self.entries[i]['obj'],self.entries[j]['obj'],self.request,result));exact=True;points=[np.asarray(result.getNearestPoint1()).tolist(),np.asarray(result.getNearestPoint2()).tolist()]
            item={'pair':[self.entries[i]['name'],self.entries[j]['name']],'distance_m':dist,'required_margin_m':threshold,'exact_mesh_distance':exact,'nearest_points_m':points}
            if kind not in minimum or dist<minimum[kind]['distance_m']:minimum[kind]=item
            # 0.1 nanometre tolerance handles roundoff at the analytic pad
            # stop. It cannot hide any meaningful reduction in clearance.
            if dist<threshold-1e-10:failures.append({'kind':kind,**item})
        return {'passed':not failures,'minimum':minimum,'failures':failures}
    def motion_bound(self,qa,qb):
        names=[side+'_'+name+'_joint' for side in ['left','right'] for name in ['base','shoulder','elbow','wrist_flex','wrist_roll','gripper']];delta=dict(zip(names,np.abs(np.asarray(qb)-qa)))
        bounds=[sum(r*delta.get(n,0.) for n,r in e['motion_radii'].items()) for e in self.entries]
        return max((bounds[i]+bounds[j] for i,j,_,_ in self.pairs),default=0.)
