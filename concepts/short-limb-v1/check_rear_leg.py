"""One-leg envelope/kinematics checks. Never calls mj_step; no walking claim.

Run with repository requirements-sim-lock.txt. Lengths internally in metres.
"""
from pathlib import Path
import argparse
import hashlib
import itertools
import json
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
OUT = HERE/'rear-leg'
C = json.loads((HERE/'design.json').read_text(encoding='utf-8'))
P = C['rear_leg_packaging']
REF = next(r for r in C['actuator_mass_references'] if r['id'] == P['motor_envelope_reference'])
JOINTS = list(P['axes_in_upright_mount_frame'])
MOTORS = ['hip_sway_motor', 'hip_pitch_motor', 'knee_motor', 'ankle_motor']


def vals(numbers):
    return ' '.join(f'{x:.12g}' for x in numbers)


def quaternion_y(degrees):
    a = math.radians(degrees)/2
    return [math.cos(a), 0, math.sin(a), 0]


def model_xml(length_mm):
    length = length_mm/1000
    r, h = REF['diameter_mm']/2000, REF['length_mm']/2000
    root = ET.Element('mujoco', model='short_limb_rear_leg_kinematic_proxy')
    ET.SubElement(root, 'compiler', angle='degree')
    ET.SubElement(root, 'option', gravity=vals([0, 0, -C['screening']['gravity_m_s2']]), timestep='0.001')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='1000', offheight='800')
    ET.SubElement(visual, 'headlight', ambient='.5 .5 .5', diffuse='.6 .6 .6')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'geom', contype='0', conaffinity='0')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -1 2', dir='0 0 -1')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='1 1 .01', rgba='.92 .91 .89 1')
    mount = ET.SubElement(world, 'body', name='mount', mocap='true', pos='0 0 .2')

    def motor(parent, name, center, axis='pitch'):
        # Cylinder primitive axis is z; orient its centre line onto x or y.
        q = [.707106781187, 0, .707106781187, 0] if axis == 'sway' else [.707106781187, .707106781187, 0, 0]
        color = '.33 .39 .44 1' if axis == 'sway' else '.22 .51 .52 1'
        ET.SubElement(parent, 'geom', name=name, type='cylinder', size=vals([r,h]),
                      pos=vals(center), quat=vals(q), mass=str(REF['mass_kg']), rgba=color)

    def joint(parent, name):
        ET.SubElement(parent, 'joint', name=name, type='hinge', axis=vals(P['axes_in_upright_mount_frame'][name]),
                      limited='true', range=vals(P['joint_limits_deg'][name]))

    def link(parent, name):
        ET.SubElement(parent, 'geom', name=name, type='capsule', fromto=vals([0,-h-.006,0,0,-h-.006,-length]),
                      size='.005', mass='.02', rgba='.79 .79 .77 1')

    motor(mount, MOTORS[0], [-P['hip_roll_housing_backward_center_mm']/1000,0,0], 'sway')
    sway = ET.SubElement(mount, 'body', name='sway')
    joint(sway, JOINTS[0])
    hip_offset = P['hip_pitch_forward_offset_mm']/1000
    motor(sway, MOTORS[1], [hip_offset,0,0])
    ET.SubElement(sway, 'geom', type='capsule', fromto=vals([0,0,0,hip_offset,0,0]), size='.006', mass='.02', rgba='.79 .79 .77 1')
    upper = ET.SubElement(sway, 'body', name='upper', pos=vals([hip_offset,0,0]))
    joint(upper, JOINTS[1])
    motor(upper, MOTORS[2], [0,0,-length])
    link(upper, 'upper_visual_link')
    lower = ET.SubElement(upper, 'body', name='lower', pos=vals([0,0,-length]))
    joint(lower, JOINTS[2])
    motor(lower, MOTORS[3], [0,0,-length])
    link(lower, 'lower_visual_link')
    foot = ET.SubElement(lower, 'body', name='foot', pos=vals([0,0,-length]))
    joint(foot, JOINTS[3])
    size = np.array(P['foot_dimensions_mm'])/2000
    height = P['ankle_axis_to_sole_mm']/1000
    forward = P['sole_center_forward_offset_mm']/1000
    ET.SubElement(foot, 'geom', name='sole', type='box', size=vals(size),
                  pos=vals([forward,0,-height+size[2]]), mass='.08', rgba='.81 .48 .6 1')
    ET.SubElement(foot, 'site', name='support_point', pos=vals([forward,0,-height]), size='.003', rgba='.9 .6 .1 1')
    ET.indent(root)
    return ET.tostring(root, encoding='unicode')+'\n'


def set_pose(model, data, pose):
    phi = pose['body_pitch_deg']
    a, k = pose['world_sagittal_hip_deg'], pose['knee_deg']
    q = [pose['hip_sway_deg'], a-phi, k, -a-k]
    data.qpos[:] = np.radians(q)
    data.mocap_quat[0] = quaternion_y(phi)
    data.mocap_pos[0] = [0,0,0]
    mujoco.mj_forward(model, data)
    corners = sole_corners(model, data)
    # Put the lowest foot corner on the floor. This is a prescribed transform,
    # not proof that the contact or the whole robot can sustain this posture.
    data.mocap_pos[0,2] = -corners[:,2].min()
    mujoco.mj_forward(model, data)
    return q


def sole_corners(model, data):
    g = model.geom('sole').id
    h = model.geom_size[g]
    local = np.array([[x*h[0],y*h[1],-h[2]] for x,y in itertools.product([-1,1],repeat=2)])
    return local@data.geom_xmat[g].reshape(3,3).T+data.geom_xpos[g]


def check_pose(model, data, pose, length_mm):
    q = set_pose(model, data, pose)
    names = MOTORS+['sole']
    distances = []
    for a,b in itertools.combinations(names,2):
        distance = mujoco.mj_geomDistance(model,data,model.geom(a).id,model.geom(b).id,1.0,None)
        distances.append({'a':a,'b':b,'gap_mm':float(distance*1000)})
    floor = model.geom('floor').id
    ground_clearance = min(mujoco.mj_geomDistance(model,data,floor,model.geom(n).id,1.0,None) for n in MOTORS)*1000
    corners = sole_corners(model,data)
    flat_error = float(np.ptp(corners[:,2])*1000)
    within_limits = all(P['joint_limits_deg'][name][0]-1e-9<=v<=P['joint_limits_deg'][name][1]+1e-9 for name,v in zip(JOINTS,q))
    worst = min(distances,key=lambda d:d['gap_mm'])
    # Equal-length, parallel pitch-axis cylinders give independent analytic checks.
    analytic = {
        ('hip_pitch_motor','knee_motor'):length_mm-REF['diameter_mm'],
        ('knee_motor','ankle_motor'):length_mm-REF['diameter_mm'],
        ('hip_pitch_motor','ankle_motor'):2*length_mm*math.cos(math.radians(pose['knee_deg'])/2)-REF['diameter_mm']}
    err = max(abs(next(d['gap_mm'] for d in distances if (d['a'],d['b'])==pair)-expected) for pair,expected in analytic.items())
    if err > .02:
        raise AssertionError(f'Cylinder distance disagreement: {err} mm')
    return {
        'pose':pose,'link_length_mm':length_mm,'joint_angles_deg':dict(zip(JOINTS,q)),
        'within_assumed_joint_limits':within_limits,'minimum_checked_gap_mm':worst['gap_mm'],
        'closest_pair':[worst['a'],worst['b']], 'pair_distances':distances,
        'minimum_motor_ground_clearance_mm':float(ground_clearance),
        'sole_corner_height_range_mm':flat_error,
        'motor_foot_envelope_clear':worst['gap_mm']>=P['minimum_envelope_gap_mm']-1e-6 and ground_clearance>=P['minimum_envelope_gap_mm']-1e-6,
        'sole_corners_coplanar_with_floor':flat_error<.1,
        'analytic_distance_max_error_mm':err,
        'mount_height_mm':float(data.mocap_pos[0,2]*1000)}


def force_examples(model,data,pose):
    set_pose(model,data,pose)
    jac=np.zeros((3,model.nv))
    mujoco.mj_jacSite(model,data,jac,None,model.site('support_point').id)
    q0=data.qpos.copy()
    fd=np.zeros_like(jac)
    eps=1e-7
    for i in range(model.nv):
        data.qpos[:]=q0;data.qpos[i]+=eps;mujoco.mj_forward(model,data)
        plus=data.site('support_point').xpos.copy()
        data.qpos[:]=q0;data.qpos[i]-=eps;mujoco.mj_forward(model,data)
        minus=data.site('support_point').xpos.copy()
        fd[:,i]=(plus-minus)/(2*eps)
    data.qpos[:]=q0;mujoco.mj_forward(model,data)
    error=float(np.max(np.abs(fd-jac)))
    if error>1e-8: raise AssertionError(f'Jacobian check failed: {error}')
    cases=[]
    for n in P['force_example_support_counts']:
        force=np.array([0,0,P['force_example_total_robot_mass_kg']*C['screening']['gravity_m_s2']/n])
        balancing=-jac.T@force
        cases.append({'supports':n,'vertical_force_n':float(force[2]),
                      'balancing_torque_contribution_nm':dict(zip(JOINTS,balancing.tolist()))})
    return {'pose_id':pose['id'],'finite_difference_max_error_m_per_rad':error,'cases':cases,
            'scope':'Only actuator contribution balancing the specified vertical force at sole centre; omits gravity of this leg, all inertia, horizontal forces and balance feasibility.'}


def render(model,data,pose,out):
    set_pose(model,data,pose)
    camera=mujoco.MjvCamera()
    camera.type=mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:]=[.015,0,.095]
    camera.distance=.47;camera.azimuth=125;camera.elevation=-18
    opt=mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT]=False
    with mujoco.Renderer(model,height=800,width=1000) as renderer:
        renderer.update_scene(data,camera,scene_option=opt)
        Image.fromarray(renderer.render()).save(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-render', action='store_true', help='Regenerate calculations/XML/HTML without opening a rendering context.')
    args = parser.parse_args()
    OUT.mkdir(exist_ok=True)
    models={}
    for length in P['upper_and_lower_length_candidates_mm']:
        xml=model_xml(length)
        (OUT/f'leg-{length}mm.xml').write_text(xml,encoding='utf-8',newline='\n')
        m=mujoco.MjModel.from_xml_string(xml)
        models[length]=(m,mujoco.MjData(m))
    sweep=[]
    s=P['sweep']
    for length,(m,d) in models.items():
        for phi,roll,a,k in itertools.product(s['body_pitch_deg'],s['hip_sway_deg'],s['world_sagittal_hip_deg'],s['knee_deg']):
            sweep.append(check_pose(m,d,dict(body_pitch_deg=phi,hip_sway_deg=roll,world_sagittal_hip_deg=a,knee_deg=k),length))
    length=P['display_link_length_mm'];m,d=models[length]
    poses=[check_pose(m,d,p,length) for p in P['poses']]
    forces=[force_examples(m,d,p) for p in P['poses'][:3]]
    summary=[]
    for l in models:
        subset=[r for r in sweep if r['link_length_mm']==l]
        summary.append({'link_length_mm':l,'sample_count':len(subset),
                        'within_limits':sum(r['within_assumed_joint_limits'] for r in subset),
                        'within_limits_and_clear':sum(r['within_assumed_joint_limits'] and r['motor_foot_envelope_clear'] for r in subset),
                        'within_limits_clear_and_flat':sum(r['within_assumed_joint_limits'] and r['motor_foot_envelope_clear'] and r['sole_corners_coplanar_with_floor'] for r in subset)})
    report={'status':'one-leg prescribed-pose kinematic/envelope screen; no timesteps or whole-body balance',
            'mujoco_version':mujoco.__version__,
            'input_sha256':hashlib.sha256((HERE/'design.json').read_text(encoding='utf-8').encode()).hexdigest(),
            'input_sha256_normalization':'UTF-8 text with LF newlines',
            'motor_envelope_reference':REF,'limits':P['limitations'],'sweep_summary':summary,
            'named_poses':poses,'force_examples':forces,'sweep':sweep}
    (OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
    page_data = {'config': C, 'report': {k:v for k,v in report.items() if k != 'sweep'}}
    template = (HERE/'rear-leg.template.html').read_text(encoding='utf-8')
    (OUT/'index.html').write_text(template.replace('__REAR_LEG_DATA__', json.dumps(page_data, ensure_ascii=False).replace('</','<\\/')), encoding='utf-8', newline='\n')
    if not args.no_render:
        for pose in P['poses']:render(m,d,pose,OUT/(pose['id']+'.png'))
    print(json.dumps({'summary':summary,'named_poses':[{k:r[k] for k in ['pose','minimum_checked_gap_mm','closest_pair','sole_corner_height_range_mm','within_assumed_joint_limits']} for r in poses], 'forces':forces},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
