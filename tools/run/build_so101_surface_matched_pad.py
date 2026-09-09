#!/usr/bin/env python3
"""Extrude the connected registered fixed-jaw contact face into the jaw gap."""
from pathlib import Path
import json
import sys
from hashlib import sha256
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def extrude_face(face, thickness):
    if not np.isfinite(thickness) or thickness <= 0:
        raise ValueError('positive finite pad thickness required')
    vertices = face.vertices.copy()
    vertices[:, 0] = -.0079
    count = len(vertices)
    outer = vertices + [thickness, 0, 0]
    faces = [*face.faces[:, ::-1].tolist(), *(face.faces + count).tolist()]
    # A positively oriented patch has its interior to the left of each boundary
    # edge when seen from +X. Build outward sidewalls without convexifying it.
    edges = np.concatenate([face.faces[:, [0,1]],face.faces[:, [1,2]],face.faces[:, [2,0]]])
    keys, inverse, occurrences = np.unique(np.sort(edges,axis=1),axis=0,return_inverse=True,return_counts=True)
    boundary = edges[occurrences[inverse] == 1]
    for a,b in boundary:
        faces.extend([[int(a),int(b),int(b+count)],[int(a),int(b+count),int(a+count)]])
    pad = trimesh.Trimesh(vertices=np.vstack([vertices,outer]), faces=faces, process=True)
    if not pad.is_watertight or not pad.is_winding_consistent or pad.volume <= 0:
        raise ValueError('extruded face must be a closed outward-oriented solid')
    return pad


def main():
    out = ROOT / 'artifacts/bimanual/planning/so101_surface_matched_pad_20260906'
    out.mkdir(parents=True,exist_ok=True)
    urdf = ROOT/'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf'
    tree = ET.parse(urdf)
    root = tree.getroot()
    components=[]
    for side in ('left','right'):
        link=root.find(f"link[@name='{side}_gripper_link']")
        collision=next(c for c in link.findall('collision') if 'wrist_cam_mount' in c.find('geometry/mesh').get('filename',''))
        source=ROOT/'ros2_ws/src/so101_description/meshes'/Path(collision.find('geometry/mesh').get('filename')).name
        m=trimesh.load_mesh(source)
        origin=collision.find('origin');T=np.eye(4)
        T[:3,:3]=Rotation.from_euler('xyz',[float(x) for x in origin.get('rpy').split()]).as_matrix()
        T[:3,3]=[float(x) for x in origin.get('xyz').split()];m.apply_transform(T)
        mask=(np.max(np.abs(m.triangles[:,:,0]+.0079),axis=1)<1e-7)&(m.face_normals[:,0]>.99999)
        patches=m.submesh([np.flatnonzero(mask)],append=True).split(only_watertight=False)
        tcp=[-.0079,-.000218121,-.0981274]
        face=min(patches,key=lambda patch:trimesh.proximity.closest_point(patch,[tcp])[1][0])
        distance=trimesh.proximity.closest_point(face,[tcp])[1][0]
        if distance>1e-7:
            raise ValueError('registered contact point is not on the selected flat face')
        components.append((face,source))
    if not np.allclose(components[0][0].bounds,components[1][0].bounds,atol=1e-8):
        raise ValueError('left/right local face geometry differs')
    face,source=components[1]
    pad=extrude_face(face,.0022)
    stl=out/'fixed_pad.stl';pad.export(stl)
    data={'schema_version':1,'record_kind':'so101_surface_matched_fixed_pad','motion_authorized':False,
          'status':'GEOMETRY_VERIFIED_CONTACT_REVALIDATION_REQUIRED',
          'source_mesh':str(source.relative_to(ROOT)),'source_mesh_sha256':sha256(source.read_bytes()).hexdigest(),
          'source_urdf_sha256':sha256(urdf.read_bytes()).hexdigest(),
          'mesh_path':str(stl.relative_to(ROOT)),'mesh_sha256':sha256(stl.read_bytes()).hexdigest(),
          'frame':'gripper_link','outward_normal':[1.,0.,0.],'thickness_m':.0022,
          'contact_center_parent_m':[-.0068,-.000218121,-.0981274],
          'inner_x_m':-.0079,'outer_x_m':-.0057,'vertices_m':pad.vertices.tolist(),'faces':pad.faces.tolist(),
          'source_patch_area_m2':float(face.area),'volume_m3':float(pad.volume),'bounds_m':pad.bounds.tolist(),
          'source_patch_triangle_count':len(face.faces),'watertight':bool(pad.is_watertight),
          'gripper_command_mapping_changed':False}
    (out/'geometry.json').write_text(json.dumps(data,indent=2)+'\n')
    # Plan-only URDF: exact same local STL on both arms. The historical URDF stays intact.
    for side in ('left','right'):
        link=root.find(f"link[@name='{side}_gripper_link']")
        for kind in ('collision','visual'):
            element=ET.SubElement(link,kind,{'name':f'{side}_surface_matched_fixed_pad'})
            ET.SubElement(element,'origin',{'xyz':'0 0 0','rpy':'0 0 0'})
            geometry=ET.SubElement(element,'geometry')
            ET.SubElement(geometry,'mesh',{'filename':str(stl.resolve())})
            if kind=='visual':
                material=ET.SubElement(element,'material',{'name':'surface_matched_rubber'})
                ET.SubElement(material,'color',{'rgba':'0.1 0.65 0.4 1'})
    ET.indent(tree,space='  ')
    tree.write(out/'plan_only_with_pad.urdf',encoding='utf-8',xml_declaration=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection, LineCollection
    fig, axes = plt.subplots(1,2,figsize=(11,5),constrained_layout=True)
    axes[0].add_collection(PolyCollection(face.triangles[:,:,1:]*1000,
                                         facecolor='#26a579',edgecolor='none'))
    axes[0].autoscale();axes[0].set_aspect('equal')
    axes[0].set(xlabel='Gripper-link Y (mm)',ylabel='Gripper-link Z (mm)',
                title='A | Exact attachment-face outline')
    for mesh_object,color,width in ((m,'#56616d',1.4),(pad,'#139a6a',2.2)):
        segments=trimesh.intersections.mesh_plane(mesh_object,[0,1,0],tcp)
        axes[1].add_collection(LineCollection(segments[:,:,[0,2]]*1000,
                                             colors=color,linewidths=width))
    axes[1].set(xlim=(-13,-3),ylim=(-106,-91),aspect='equal',xlabel='Gripper-link X (mm)',
                ylabel='Gripper-link Z (mm)',title='B | Uniform outward extrusion')
    axes[1].annotate('',xy=(-7.9,-92.5),xytext=(-5.7,-92.5),arrowprops={'arrowstyle':'<->'})
    axes[1].text(-6.8,-92,'2.2 mm',ha='center')
    for ax in axes:ax.grid(alpha=.15)
    fig.suptitle('Surface-matched rubber pad | green: new mesh | grey: original plastic')
    fig.savefig(out/'surface_matched_pad.png',dpi=180)
    print(json.dumps({k:v for k,v in data.items() if k not in ('vertices_m','faces')},indent=2))


if __name__=='__main__':
    main()
