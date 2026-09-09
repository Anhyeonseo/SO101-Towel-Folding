#!/usr/bin/env python3
"""Render registered STL sections and existing pad definitions; no simulation."""
from pathlib import Path
import sys
import json
import xml.etree.ElementTree as ET
from hashlib import sha256
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.run.audit_towel_s2_corner_contact import literal_assignment, unit, angle_deg


def transform(xyz, rpy):
    result = np.eye(4)
    result[:3,:3] = Rotation.from_euler('xyz', rpy).as_matrix()
    result[:3,3] = xyz
    return result


def values(element, name):
    return [float(x) for x in element.get(name, '0 0 0').split()]


def main():
    output = ROOT / 'artifacts/bimanual/planning/s2_contact_audit_20260906'
    output.mkdir(parents=True, exist_ok=True)
    urdf = ROOT / 'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf'
    root = ET.parse(urdf).getroot()
    mesh_root = ROOT / 'ros2_ws/src/so101_description/meshes'

    def mesh(link, filename):
        for col in root.find(f"link[@name='{link}']").findall('collision'):
            element = col.find('geometry/mesh')
            if element is not None and filename in element.get('filename'):
                path = mesh_root / Path(element.get('filename')).name
                result = trimesh.load_mesh(path)
                origin = col.find('origin')
                result.apply_transform(transform(values(origin, 'xyz'), values(origin, 'rpy')))
                return result, path
        raise ValueError('collision STL missing')

    fixed, fixed_path = mesh('right_gripper_link', 'wrist_cam_mount')
    moving, _ = mesh('right_moving_jaw_link', 'moving_jaw')
    joint = root.find("joint[@name='right_gripper_joint']")
    origin = joint.find('origin')
    config = json.loads((ROOT / 'config/so101_gripper_geometry.candidate.json').read_text())
    q0 = config['geometry']['detailed_stl_model_q_at_physical_q0_rad']
    joint_transform = transform(values(origin, 'xyz'), values(origin, 'rpy'))
    hinge = np.eye(4)
    hinge[:3,:3] = Rotation.from_rotvec(np.array(values(joint.find('axis'), 'xyz')) * q0).as_matrix()
    moving.apply_transform(joint_transform @ hinge)
    runner = ROOT / 'tools/setup/isaac/run_towel_s1_vertex_patch_lift.py'
    center = np.array(literal_assignment(runner, 'FIXED_JAW_PAD_CENTER_PARENT_M'))
    tcp = np.array(literal_assignment(runner, 'GRIPPER_FRAME_TRANSLATION_M'))
    normal = unit(literal_assignment(runner, 'JAW_PAD_NORMALS_PARENT')['right']['fixed'])
    axis = np.cross([1.,0,0], normal)
    pad_rotation = Rotation.from_rotvec(unit(axis)*np.arccos(normal[0])).as_matrix()
    size = [config['geometry']['fixed_jaw_rubber_pad']['thickness_mm']/1000, .006, .006]
    planner_box = trimesh.creation.box(size)
    planner_box.apply_translation(center)
    isaac_box = trimesh.creation.box(size)
    tr = np.eye(4);tr[:3,:3] = pad_rotation;tr[:3,3] = center
    isaac_box.apply_transform(tr)
    closest, distance, triangles = trimesh.proximity.closest_point(fixed, [tcp])
    surface_normal = fixed.face_normals[triangles[0]]
    palette = ['#555e68', '#999fa6', '#0879ce', '#e8871e']
    labels = ['Fixed jaw / camera STL', 'Moving jaw STL at physical Q0',
              'Planner normal + current centre*', 'Isaac collider (current)']
    meshes = [fixed, moving, planner_box, isaac_box]
    fig, axes = plt.subplots(1,3,figsize=(16,6.5))
    for ax, projection, plane_normal in [(axes[0], [0,2], [0,1,0]),
                                         (axes[1], [0,2], [0,1,0]),
                                         (axes[2], [0,1], [0,0,1])]:
        for m, color, label in zip(meshes, palette, labels):
            segments = trimesh.intersections.mesh_plane(m, plane_normal, tcp)
            ax.add_collection(LineCollection(segments[:,:,projection]*1000, colors=color,
                                            linewidths=2 if m in (planner_box,isaac_box) else 1.1,
                                            label=label))
        ax.autoscale();ax.set_aspect('equal');ax.grid(alpha=.15)
        ax.set_xlabel('Gripper-link X (mm)')
        ax.set_ylabel('Gripper-link '+('Z' if projection[1]==2 else 'Y')+' (mm)')
    axes[0].set_title('A | Whole jaw section (physical Q0)')
    axes[0].set_xlim(-40,45);axes[0].set_ylim(-110,25)
    axes[0].plot([-14,0,0,-14,-14],[-104,-104,-92,-92,-104],color='#be3540',ls='--')
    axes[1].set(xlim=(-14,0),ylim=(-104,-92),title='B | Contact face: X-Z section')
    axes[2].set(xlim=(-14,0),ylim=(-6,6),title='C | Contact face: X-Y section')
    for ax, coordinate in [(axes[1],2),(axes[2],1)]:
        ax.scatter(center[0]*1000,center[coordinate]*1000,c='black',s=25,zorder=8)
        ax.axvline(tcp[0]*1000,color='#555e68',ls=':',lw=1)
        ax.annotate('STL face X = -7.9',xy=(tcp[0]*1000,tcp[coordinate]*1000+2.5),
                    xytext=(-6,tcp[coordinate]*1000+4),fontsize=9,
                    arrowprops={'arrowstyle':'->','color':'#555e68'})
        ax.annotate('+X / jaw-gap side',xy=(-1,tcp[coordinate]*1000-4),
                    xytext=(-7,tcp[coordinate]*1000-4),fontsize=9,arrowprops={'arrowstyle':'->'})
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='lower center',bbox_to_anchor=(.5,.075),ncol=2,fontsize=10)
    fig.suptitle('Right fixed pad on registered STL | 2.2 mm thick, 6 x 6 mm model patch',fontsize=15)
    fig.supxlabel('*Blue: orientation comparison at the existing centre; not a separately authored planner solid.\nBlack dot: current centre X=-9.0 mm. No geometry changed. Dimensions of the real pad face are not verified.',fontsize=9)
    image_path = output / 'fixed_pad_stl_overlay.png'
    fig.subplots_adjust(left=.06,right=.98,top=.87,bottom=.25,wspace=.28)
    fig.savefig(image_path,dpi=180)
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig3 = plt.figure(figsize=(12, 6))
    for index, zoom in enumerate((False, True), 1):
        ax = fig3.add_subplot(1, 2, index, projection='3d')
        for mesh_object, color in ((fixed, '#687582'), (moving, '#a4aab1')):
            triangles3 = mesh_object.triangles
            if zoom:
                centers3 = triangles3.mean(axis=1)
                triangles3 = triangles3[(np.abs(centers3[:, 1]-tcp[1]) < .013)
                                       & (centers3[:, 2] < -.084)]
            ax.add_collection3d(Poly3DCollection(triangles3*1000, facecolor=color,
                                               edgecolor='none', alpha=.16))
        for box, color in ((planner_box,'#0879ce'), (isaac_box,'#e8871e')):
            ax.add_collection3d(Poly3DCollection(box.triangles*1000, facecolor=color,
                                               edgecolor=color, linewidth=.5, alpha=.18))
        # The actual local flat STL patch, marked for operator identification.
        face = np.array([[tcp[0],tcp[1]+y,tcp[2]+z] for y,z in
                         [(-.003,-.003),(.003,-.003),(.003,.003),(-.003,.003)]])
        ax.add_collection3d(Poly3DCollection([face*1000],facecolor='#21a179',alpha=.5))
        if zoom:
            ax.set(xlim=(-16,0), ylim=(-8,8), zlim=(-106,-90), title='B | Fixed-tip enlargement')
            ax.set_box_aspect((1,1,1))
        else:
            ax.set(xlim=(-40,40), ylim=(-90,35), zlim=(-110,25), title='A | Whole registered jaw + camera mount')
            ax.set_box_aspect((80,125,135))
        ax.view_init(elev=18, azim=-35)
        ax.set_xlabel('X (mm)');ax.set_ylabel('Y (mm)');ax.set_zlabel('Z (mm)')
    fig3.suptitle('Green = STL contact-plane patch | Orange = current Isaac pad | Blue = aligned at current centre')
    fig3.text(.5,.02,'Transparent CAD view. The green patch marks the face to confirm on the physical gripper; no model was changed.',ha='center',fontsize=10)
    fig3.subplots_adjust(left=.02,right=.97,bottom=.08,top=.9,wspace=.05)
    fig3.savefig(output/'fixed_pad_stl_3d.png',dpi=170)
    result = {'motion_authorized':False,'geometry_modified':False,
              'frame':'right_gripper_link','fixed_stl_sha256':sha256(fixed_path.read_bytes()).hexdigest(),
              'urdf_sha256':sha256(urdf.read_bytes()).hexdigest(),
              'runner_sha256':sha256(runner.read_bytes()).hexdigest(),
              'physical_q0_model_rad':q0,'pad_size_m':size,
              'stl_nearest_point_m':closest[0].tolist(),'tcp_to_stl_distance_m':float(distance[0]),
              'stl_face_normal':surface_normal.tolist(),'authored_pad_normal':normal.tolist(),
              'normal_disagreement_deg':angle_deg(surface_normal,normal),
              'existing_pad_center_m':center.tolist(),
              'center_signed_offset_along_stl_normal_m':float((center-closest[0])@surface_normal),
              'mesh_watertight':bool(fixed.is_watertight),
              'interpretation':'Local STL normal and sections place the current centre behind the contact plane. Non-watertight STL: no global solid-inside proof. Physical attachment side still requires visual confirmation.'}
    (output/'fixed_pad_stl_overlay.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
