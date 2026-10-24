"""Full-silhouette reach and arithmetic study; no dynamics, contacts or full CAD fit.

Run with requirements-sim-lock.txt. --render also creates geometric illustrations.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET

import numpy as np
from ankle_linkage import evaluate as evaluate_ankle
from ankle_linkage import solve as solve_ankle

HERE = Path(__file__).resolve().parent
C = json.loads((HERE/'design.json').read_text(encoding='utf-8'))
G, M, A = C['geometry_m'], C['motor_references'], C['mass_location_assumptions']


def rx(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])


def ry(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


def solve_limb(root, target, lengths, rotation):
    """Ideal intersecting roll/pitch hip, then knee. Positive knee branch only."""
    local=rotation.T@(target-root)
    roll=math.atan2(local[1],-local[2])
    # The same sagittal plane has two roll representations. Use the smaller
    # roll and let pitch represent reaching above the torso's local hip plane.
    if roll>math.pi/2:roll-=math.pi
    if roll<-math.pi/2:roll+=math.pi
    plane=rx(-roll)@local
    l1,l2=lengths
    radius=float(np.linalg.norm(local))
    c=(radius**2-l1*l1-l2*l2)/(2*l1*l2)
    if not -1-1e-10<=c<=1+1e-10:
        return {'reachable':False,'distance_m':radius,'max_reach_m':l1+l2}
    k=math.acos(np.clip(c,-1,1))
    p=math.atan2(-plane[0],-plane[2])-math.atan2(l2*math.sin(k),l1+l2*math.cos(k))
    knee=root+rotation@rx(roll)@ry(p)@np.array([0,0,-l1])
    end=knee+rotation@rx(roll)@ry(p+k)@np.array([0,0,-l2])
    error=float(np.linalg.norm(end-target))
    if error>1e-9:raise AssertionError('Inverse/forward endpoint mismatch')
    if abs(np.linalg.norm(knee-root)-l1)>1e-10 or abs(np.linalg.norm(end-knee)-l2)>1e-10:
        raise AssertionError('Link length mismatch')
    return {'reachable':True,'root':root,'middle':knee,'end':end,
            'angles_deg':np.degrees([roll,p,k]),'endpoint_error_m':error}


def allowed(name, value):
    a,b=C['joint_limit_assumptions_deg'][name]
    return a-1e-8<=value<=b+1e-8


def ellipsoid_lowest(center, radii, rotation):
    return float(center[2]-np.linalg.norm(rotation[2,:]*radii))


def pose(phi_deg, height, carried):
    phi=math.radians(phi_deg);R=ry(phi);base=np.array([0.,0.,height])
    neck=base+R@np.array(G['neck_pivot_xyz'])
    head=neck+np.array([0,0,G['head_center_above_neck']])
    torso=base+R@np.array(G['body_center_xyz'])
    result={'body_pitch_deg':phi_deg,'hip_height_m':height,'arms_carried':carried,
            'body_rotation':R,'base':base,'neck':neck,'head_center':head,'torso_center':torso,
            'head_ground_gap_m':ellipsoid_lowest(head,np.array(G['head_radii_xyz']),np.eye(3)),
            'body_ground_gap_m':ellipsoid_lowest(torso,np.array(G['body_radii_xyz']),R),
            'rear':[],'front':[],'limit_violations':[]}
    for side in [-1,1]:
        hip=base+R@np.array([0,side*G['hip_half_width'],0])
        ankle=np.array([G['rear_ankle_x'],side*G['hip_half_width'],G['ankle_height']])
        leg=solve_limb(hip,ankle,G['rear_links'],R)
        if leg['reachable']:
            roll,p,k=leg['angles_deg'];ap=-phi_deg-p-k
            # Rear targets have zero local lateral displacement in this screen.
            assert abs(roll)<1e-8
            leg['ankle_pitch_deg']=ap
            foot_rotation=R@rx(math.radians(roll))@ry(math.radians(p+k+ap))
            leg['foot_orientation_error']=float(np.max(np.abs(foot_rotation-np.eye(3))))
            assert leg['foot_orientation_error']<1e-9
            for key,v in [('rear_hip_pitch',p),('rear_knee',k),('rear_ankle_pitch',ap)]:
                if not allowed(key,v):result['limit_violations'].append(key)
        result['rear'].append(leg)
        sx,sy,sz=G['shoulder_xyz'];shoulder=base+R@np.array([sx,side*sy,sz])
        if carried:
            x,y,z=G['carried_paw_relative_xyz'];target=base+np.array([x,side*y,z])
        else:
            x,y=G['quad_fore_contact_xy'];target=np.array([x,side*y,G['paw_radius']])
        arm=solve_limb(shoulder,target,G['front_links'],R)
        if arm['reachable']:
            for key,v in zip(['front_shoulder_roll','front_shoulder_pitch','front_elbow'],arm['angles_deg']):
                if not allowed(key,v):result['limit_violations'].append(key)
        result['front'].append(arm)
    result['limb_targets_reachable']=all(r['reachable'] for r in result['rear']+result['front'])
    if not allowed('head_pitch',-phi_deg):result['limit_violations'].append('head_pitch')
    result['reach_and_assumed_ranges']=result['limb_targets_reachable'] and not result['limit_violations']
    if result['limb_targets_reachable']:
        result.update(mass_properties(result))
    return result


def mass_properties(p):
    masses=[]
    def add(name,m,where):masses.append({'name':name,'mass_kg':m,'position_m':where})
    O=C['other_mass_allocations_kg'];R=p['body_rotation']
    core=A['body_frame_share_kg']+sum(O[k] for k in ['body_shell','compute_power_electronics','wire_tether_attachment','unallocated_reserve'])
    add('body_core_allocation',core,p['base']+R@np.array(A['body_core_xyz']))
    add('neck_motor',M['head']['mass_kg'],p['neck'])
    add('light_head_shell',O['hollow_head_face'],p['head_center'])
    for i,leg in enumerate(p['rear']):
        h,k,f=leg['root'],leg['middle'],leg['end']
        add(f'hip_motors_{i}',2*M['large']['mass_kg']+M['medium']['mass_kg'],h)
        add(f'knee_motor_{i}',M['large']['mass_kg'],h+A['remote_knee_motor_upper_fraction']*(k-h))
        add(f'ankle_motor_pair_{i}',2*M['large']['mass_kg'],k+A['remote_ankle_pair_lower_fraction']*(f-k))
        add(f'thigh_structure_{i}',A['rear_frame_each_kg']/2,(h+k)/2)
        add(f'shank_structure_{i}',A['rear_frame_each_kg']/2,(k+f)/2)
        add(f'foot_{i}',A['rear_pad_each_kg'],f-np.array([0,0,G['ankle_height']/2]))
    for i,arm in enumerate(p['front']):
        h,k,f=arm['root'],arm['middle'],arm['end']
        add(f'shoulder_motors_{i}',M['large']['mass_kg']+M['medium']['mass_kg'],h)
        add(f'elbow_motor_{i}',M['large']['mass_kg'],h+A['remote_elbow_motor_upper_fraction']*(k-h))
        add(f'upper_arm_{i}',A['front_frame_each_kg']/2,(h+k)/2)
        add(f'forearm_{i}',A['front_frame_each_kg']/2,(k+f)/2)
        add(f'paw_{i}',A['front_pad_each_kg'],f)
    total=sum(m['mass_kg'] for m in masses)
    budget=sum(m['mass_kg']*m['count'] for m in M.values())+sum(O.values())
    assert abs(total-budget)<1e-10
    com=sum(m['mass_kg']*m['position_m'] for m in masses)/total
    rearx=G['rear_ankle_x'];foot_l,foot_w,_=G['rear_foot_length_width_height']
    if p['arms_carried']:
        support_x=[rearx-foot_l/2,rearx+foot_l/2]
        support_y=[-G['hip_half_width']-foot_w/2,G['hip_half_width']+foot_w/2]
        projection_in_two_foot_hull=support_x[0]<=com[0]<=support_x[1] and support_y[0]<=com[1]<=support_y[1]
    else:
        # Symmetric sagittal force distribution at nominal foot/paw centres.
        support_x=[rearx,G['quad_fore_contact_xy'][0]]
        projection_in_two_foot_hull=None
    return {'mass_points':masses,'allocated_mass_kg':total,'com_m':com,
            'nominal_support_x_m':support_x,'two_foot_com_projection_inside':projection_in_two_foot_hull,
            'quad_fore_vertical_load_fraction':None if p['arms_carried'] else float((com[0]-rearx)/(support_x[1]-rearx))}


def force_examples(total):
    out=[]
    for height in [G['upright_hip_height'],.22]:
        root=np.array([0.,G['hip_half_width'],height])
        end=np.array([G['rear_ankle_x'],G['hip_half_width'],G['ankle_height']])
        leg=solve_limb(root,end,G['rear_links'],np.eye(3))
        force=total*C['gravity_m_s2']
        # Sole force applied below ankle, with same x; vertical force only.
        out.append({'hip_height_m':height,'knee_deg':float(leg['angles_deg'][2]),
                    'vertical_force_n':force,'hip_pitch_contribution_nm':force*abs(end[0]-root[0]),
                    'knee_contribution_nm':force*abs(end[0]-leg['middle'][0]),
                    'scope':'One-leg vertical ground-force contribution in the shown geometry; not a balanced single-support posture, and leg gravity/inertia/horizontal forces omitted.'})
    return out


def recovery_reach():
    out=[]
    for name,R in [('side',rx(math.pi/2)),('back',ry(-math.pi/2))]:
        head=R@(np.array(G['neck_pivot_xyz'])+np.array([0,0,G['head_center_above_neck']]))
        body=R@np.array(G['body_center_xyz'])
        origin_height=-min(ellipsoid_lowest(head,np.array(G['head_radii_xyz']),R),ellipsoid_lowest(body,np.array(G['body_radii_xyz']),R))
        gaps=[]
        for side in [-1,1]:
            x,y,z=G['shoulder_xyz']
            shoulder_height=origin_height+(R@np.array([x,side*y,z]))[2]
            gaps.append({'side':side,'shoulder_to_floor_m':float(shoulder_height),
                         'unobstructed_max_reach_margin_m':float(sum(G['front_links'])+G['paw_radius']-shoulder_height)})
        out.append({'orientation':name,'shell_only_origin_height_m':origin_height,'arms':gaps,
                    'scope':'Necessary distance bound only, with head/torso as ellipsoid contact envelopes. Ignores actual folded limb contacts, reach path, joint limits, collisions, friction and restoring torque.'})
    return out


def render_geometry(p,filename,azimuth=135):
    import mujoco
    from PIL import Image
    def val(v):return ' '.join(str(float(x)) for x in v)
    root=ET.Element('mujoco',model='doro_static_geometry_illustration')
    visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='1200',offheight='1000')
    ET.SubElement(visual,'headlight',ambient='.65 .65 .65',diffuse='.6 .6 .6')
    asset=ET.SubElement(root,'asset');ET.SubElement(asset,'texture',type='skybox',builtin='gradient',rgb1='.95 .95 .94',rgb2='.95 .95 .94',width='64',height='64')
    world=ET.SubElement(root,'worldbody');ET.SubElement(world,'light',pos='0 -1 2',dir='0 0 -1',castshadow='false')
    ET.SubElement(world,'geom',type='plane',size='2 2 .01',rgba='.93 .93 .91 1')
    def geom(**args):ET.SubElement(world,'geom',contype='0',conaffinity='0',**args)
    phi=math.radians(p['body_pitch_deg'])
    geom(type='ellipsoid',size=val(G['body_radii_xyz']),pos=val(p['torso_center']),quat=val([math.cos(phi/2),0,math.sin(phi/2),0]),rgba='.65 .73 .73 .23')
    geom(type='ellipsoid',size=val(G['head_radii_xyz']),pos=val(p['head_center']),rgba='.87 .53 .68 .33')
    geom(type='capsule',fromto=val([*p['base'],*p['neck']]),size='.018',rgba='.22 .3 .32 1')
    geom(type='capsule',fromto=val([*p['neck'],*p['head_center']]),size='.012',rgba='.22 .3 .32 1')
    for limbs,color in [(p['rear'],'.17 .48 .5 1'),(p['front'],'.43 .53 .56 1')]:
        for limb in limbs:
            h,k,f=limb['root'],limb['middle'],limb['end']
            for a,b in [(h,k),(k,f)]:geom(type='capsule',fromto=val([*a,*b]),size='.013',rgba=color)
            for pt in [h,k,f]:geom(type='sphere',pos=val(pt),size='.023',rgba=color)
    for leg in p['rear']:
        x,y,_=leg['end'];l,w,h=G['rear_foot_length_width_height']
        geom(type='box',pos=val([x,y,h/2]),size=val([l/2,w/2,h/2]),rgba='.75 .43 .57 1')
        geom(type='capsule',fromto=val([*leg['end'],x,y,h/2]),size='.012',rgba='.17 .48 .5 1')
    for arm in p['front']:geom(type='sphere',pos=val(arm['end']),size=str(G['paw_radius']),rgba='.75 .43 .57 1')
    # Small marker is estimated centre of mass, not a physical part.
    geom(type='sphere',pos=val(p['com_m']),size='.014',rgba='1 .55 .05 1')
    xml=ET.tostring(root,encoding='unicode')+'\n'
    (HERE/(filename+'.xml')).write_text(xml,encoding='utf-8',newline='\n')
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    camera=mujoco.MjvCamera();camera.lookat[:]=[.06,0,.34];camera.distance=1.35;camera.azimuth=azimuth;camera.elevation=-13
    with mujoco.Renderer(m,height=1000,width=1200) as renderer:
        renderer.update_scene(d,camera)
        Image.fromarray(renderer.render()).save(HERE/(filename+'.png'))


def serial(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)


def render_ankle(result):
    import mujoco
    from PIL import Image
    def val(v):return ' '.join(str(float(x)) for x in v)
    root=ET.Element('mujoco',model='ankle_rod_crank_geometry_only')
    visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='1000',offheight='850')
    ET.SubElement(visual,'headlight',ambient='.65 .65 .65',diffuse='.6 .6 .6')
    asset=ET.SubElement(root,'asset');ET.SubElement(asset,'texture',type='skybox',builtin='gradient',rgb1='.95 .95 .94',rgb2='.95 .95 .94',width='64',height='64')
    world=ET.SubElement(root,'worldbody')
    def geom(**args):ET.SubElement(world,'geom',contype='0',conaffinity='0',**args)
    def link(a,b,r,color):geom(type='capsule',fromto=val([*a,*b]),size=str(r),rgba=color)
    foot_l,foot_w,foot_h=G['rear_foot_length_width_height']
    geom(type='plane',size='1 1 .01',pos=val([0,0,-G['ankle_height']]),rgba='.93 .93 .91 1')
    geom(type='box',size=val([foot_l/2,foot_w/2,foot_h/2]),pos=val([0,0,-G['ankle_height']+foot_h/2]),rgba='.75 .43 .57 1')
    P=C['ankle_linkage'];N=result['neutral'];ref=M['large']
    for center,axis,e,b in zip(P['motor_housing_centers_m'],P['crank_axis_centers_m'],N['crank_points_m'],N['foot_points_m']):
        geom(type='cylinder',size=val([ref['diameter_mm']/2000,ref['length_mm']/2000]),pos=val(center),quat='.707106781187 .707106781187 0 0',rgba='.26 .41 .44 1')
        link(center,axis,.004,'.48 .5 .52 1')
        link(axis,e,.007,'.65 .69 .67 1')
        link(e,b,P['rod_radius_m'],'.72 .31 .51 1')
        # Indicate the required attachment riser, without claiming a designed
        # bearing/rod-end bracket or including it in the clearance screen.
        link(b,[b[0],b[1],-G['ankle_height']+foot_h],.004,'.48 .5 .52 1')
        for point in [e,b]:geom(type='sphere',pos=val(point),size='.008',rgba='.22 .3 .32 1')
    geom(type='sphere',pos='0 0 0',size='.015',rgba='.18 .65 .62 1')
    link([0,0,0],[0,0,-G['ankle_height']+foot_h],.007,'.48 .5 .52 1')
    xml=ET.tostring(root,encoding='unicode')+'\n'
    (HERE/'ankle-neutral.xml').write_text(xml,encoding='utf-8',newline='\n')
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    camera=mujoco.MjvCamera();camera.lookat[:]=[0,0,.06];camera.distance=.53;camera.azimuth=130;camera.elevation=-18
    with mujoco.Renderer(m,height=850,width=1000) as renderer:
        renderer.update_scene(d,camera)
        Image.fromarray(renderer.render()).save(HERE/'ankle-neutral.png')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--render',action='store_true');args=parser.parse_args()
    poses={'biped':pose(0,G['upright_hip_height'],True),'quadruped':pose(G['quadruped_body_pitch_deg'],G['quadruped_hip_height'],False)}
    assert all(p['reach_and_assumed_ranges'] for p in poses.values())
    counts=sum(m['count'] for m in M.values());axes=2*C['axes']['rear_per_leg']+2*C['axes']['front_per_leg']+C['axes']['head_pitch'];assert counts==axes
    total=poses['biped']['allocated_mass_kg']
    sweep=[pose(phi,h,False) for phi in G['sweep_body_pitch_deg'] for h in G['sweep_hip_heights']]
    T=C['transmission_screen'];motor=M['large']
    transmissions=[{'additional_ratio':n,'assumed_rated_nm':motor['rated_torque_nm']*n*T['assumed_efficiency'],
                    'assumed_peak_nm':motor['peak_torque_nm']*n*T['assumed_efficiency'],
                    'no_load_rpm_divided_by_ratio':motor['no_load_rpm']/n} for n in T['knee_additional_ratios']]
    report={'status':'geometry, ideal-axis reach and arithmetic only; fabrication fit and locomotion unverified',
            'input_sha256':hashlib.sha256((HERE/'design.json').read_text(encoding='utf-8').encode()).hexdigest(),
            'hash_normalization':'UTF-8 text, LF newlines','axes_total':axes,
            'motor_mass_kg':sum(m['count']*m['mass_kg'] for m in M.values()),'total_mass_allocation_kg':total,
            'poses':poses,'all_four_feet_sweep':sweep,'force_examples':force_examples(total),
            'recovery_distance_bounds':recovery_reach(),'knee_ratio_examples':transmissions,
            'ankle_linkage':evaluate_ankle(C),
            'belt_tension_difference_examples':[{'torque_nm':t,'tight_minus_slack_force_n':t/T['output_pulley_pitch_radius_m']} for t in T['torque_examples_nm']],
            'single_support_from_symmetric_stance':{'minimum_lateral_com_shift_to_foot_inner_edge_m':G['hip_half_width']-G['rear_foot_length_width_height'][1]/2,
                                                    'scope':'Static projection only. Does not solve a feasible shifted pose or gait.'},
            'not_validated':C['not_validated']}
    report['ankle_at_body_reference_poses']={name:solve_ankle(C,C['ankle_linkage']['display_radius_m'],math.radians(p['rear'][0]['ankle_pitch_deg']),0) for name,p in poses.items()}
    assert all(r['reachable'] and not r.get('singular',False) and r['rod_to_motor_clearance_lower_bound_m']>0 for r in report['ankle_at_body_reference_poses'].values())
    (HERE/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,default=serial)+'\n',encoding='utf-8',newline='\n')
    template=HERE/'index.template.html'
    if template.exists():
        data=json.dumps({'config':C,'report':report},ensure_ascii=False,default=serial).replace('</','<\\/')
        (HERE/'index.html').write_text(template.read_text(encoding='utf-8').replace('__DATA__',data),encoding='utf-8',newline='\n')
    if args.render:
        render_geometry(poses['biped'],'biped');render_geometry(poses['quadruped'],'quadruped');render_geometry(poses['biped'],'front',0)
        render_ankle(report['ankle_linkage'])
    print(json.dumps({'mass':total,'axes':axes,'poses':{k:{x:p[x] for x in ['reach_and_assumed_ranges','head_ground_gap_m','body_ground_gap_m','com_m','quad_fore_vertical_load_fraction','two_foot_com_projection_inside']} for k,p in poses.items()},'forces':report['force_examples'],'four_feet_reachable_samples':sum(p['reach_and_assumed_ranges'] for p in sweep),'samples':len(sweep)},default=serial,indent=2))


if __name__=='__main__':main()
