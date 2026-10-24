"""Independent URDF kinematics, sampled collision diagnostics, model smoke test.

No controller is supplied. Passing these tests does not demonstrate balance.
"""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent


def rotation(axis, a):
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(a)*K + (1-math.cos(a))*(K@K)


def validate():
    model = json.loads((HERE/'model.json').read_text(encoding='utf8'))
    clips = json.loads((HERE/'motions.json').read_text(encoding='utf8'))
    ur = ET.parse(HERE/'robot.urdf').getroot()
    m = mujoco.MjModel.from_xml_path(str(HERE/'robot.xml')); d = mujoco.MjData(m)
    joints = model['joint_order']; links = model['links']
    uj = {j.attrib['name']: j for j in ur.findall('joint')}
    ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in joints]
    qa = m.jnt_qposadr[ids]
    assert (len(links), len(uj), m.nbody, m.njnt, m.nu, m.nq, m.nv) == (20,19,21,20,19,26,25)
    assert [mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,j) for j in range(1,20)] == joints
    assert np.all(m.body_inertia[1:] > 0)
    assert abs(sum(float(n.find('inertial/mass').attrib['value']) for n in ur.findall('link'))-model['mass_kg']) < 1e-9
    assert abs(sum(m.body_mass)-model['mass_kg']) < 1e-9
    for n in links:
        assert np.allclose(m.body_inertia[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n['name'])], n['inertia_diag'],atol=1e-10)
    maxima = {'urdf_mujoco_link_position_m':0., 'urdf_mujoco_rotation_matrix':0.,
              'site_position_m':0., 'sole_orientation_rad':0., 'loop_closure_rad':0.}
    frames_checked = 0; body_overlaps = {}; motor_overlaps = {}; floor_min = 0.
    motors = [mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,g['name']) for n in links for g in n['geoms'] if g['layer']=='motor' and g['collision']]
    motor_pairs = [(a,b) for i,a in enumerate(motors) for b in motors[i+1:]]
    assert len(motors) == 19
    def check_fk():
        # Independent parse of URDF origins, axis signs and tree, not MJ Jacobians.
        T = {'base':(d.xpos[1].copy(),d.xmat[1].reshape(3,3).copy())}
        for name in joints:
            j = uj[name]; parent = j.find('parent').attrib['link']; child = j.find('child').attrib['link']
            origin = np.fromstring(j.find('origin').attrib['xyz'],sep=' ')
            assert j.find('origin').attrib['rpy'] == '0 0 0'
            axis = np.fromstring(j.find('axis').attrib['xyz'],sep=' ')
            pp,pr = T[parent]; T[child] = (pp+pr@origin,pr@rotation(axis,d.qpos[qa[joints.index(name)]]))
        for name,(p,r) in T.items():
            bi = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,name)
            maxima['urdf_mujoco_link_position_m']=max(maxima['urdf_mujoco_link_position_m'],float(np.linalg.norm(p-d.xpos[bi])))
            maxima['urdf_mujoco_rotation_matrix']=max(maxima['urdf_mujoco_rotation_matrix'],float(np.max(np.abs(r-d.xmat[bi].reshape(3,3)))))
    sites = ['rear_R_sole','front_R_paw','rear_L_sole','front_L_paw']
    browser_samples = []
    for mode,clip in clips.items():
        q = np.array([f['q'] for f in clip['frames']])
        assert np.all(np.isfinite(q))
        assert np.all(q>=m.jnt_range[ids,0]-1e-9) and np.all(q<=m.jnt_range[ids,1]+1e-9)
        maxima['loop_closure_rad']=max(maxima['loop_closure_rad'],float(np.max(np.abs(q[0]-q[-1]))))
        for i,f in enumerate(clip['frames']):
            d.qpos[:] = f['base']+f['quat']+f['q']; mujoco.mj_forward(m,d); check_fk(); frames_checked += 1
            assert np.allclose(d.subtree_com[1], f['com'],atol=1e-9)
            for name,target in zip(sites,f['targets']):
                si=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,name)
                maxima['site_position_m']=max(maxima['site_position_m'],float(np.linalg.norm(d.site_xpos[si]-target)))
                if name.endswith('sole'):
                    a=math.acos(np.clip((np.trace(d.site_xmat[si].reshape(3,3))-1)/2,-1,1))
                    maxima['sole_orientation_rad']=max(maxima['sole_orientation_rad'],a)
            for contact in d.contact:
                if contact.dist >= -.001:continue
                names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(g)) for g in contact.geom]
                if 'floor' in names:floor_min=min(floor_min,float(contact.dist))
                else:
                    pair=' / '.join(sorted(names));body_overlaps[pair]=min(body_overlaps.get(pair,0),float(contact.dist))
            # Explicit queries include adjacent/same-body motor envelopes that
            # automatic contact filtering would otherwise omit.
            for a,b in motor_pairs:
                dist=mujoco.mj_geomDistance(m,d,a,b,.005,None)
                if dist<-.001:
                    pair=' / '.join(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g) for g in [a,b])
                    motor_overlaps[pair]=min(motor_overlaps.get(pair,0),float(dist))
            if i in {0,len(clip['frames'])//2}:
                browser_samples.append({'mode':mode,'time':i/clip['fps'],'points':{n['name']:d.xpos[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n['name'])].tolist() for n in links}})
    rng=np.random.default_rng(3)
    for _ in range(32):
        d.qpos[qa]=rng.uniform(m.jnt_range[ids,0],m.jnt_range[ids,1]);mujoco.mj_forward(m,d);check_fk()
    # Numerical loading/integration only: free base, zero input, no support claim.
    mujoco.mj_resetDataKeyframe(m,d,0)
    for _ in range(100):mujoco.mj_step(m,d)
    assert np.all(np.isfinite(d.qpos)) and abs(d.time-.2)<1e-10
    report={'mass_allocated_kg':model['mass_kg'],'joints':19,'links':20,'sampled_frames':frames_checked,
            'additional_random_fk_poses':32,'max_errors':maxima,'automatic_primitive_overlaps_below_1mm':body_overlaps,
            'all_motor_pair_overlaps_below_1mm':motor_overlaps,'floor_penetration_below_1mm_m':floor_min,
            'free_base_zero_input_smoke':{'seconds':float(d.time),'finite':True,'final_base_height_m':float(d.qpos[2]),'balance_test':False},
            'not_covered':['continuous paths between samples','complete shell, mounting and wiring clearances','joint-limit physical stops','balance, motor limits and self-righting']}
    (HERE/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8',newline='\n')
    (HERE/'browser_samples.json').write_text(json.dumps(browser_samples,separators=(',',':'))+'\n',encoding='utf8',newline='\n')
    print(json.dumps(report,indent=2))
    assert maxima['urdf_mujoco_link_position_m']<1e-9 and maxima['urdf_mujoco_rotation_matrix']<1e-9
    assert maxima['site_position_m']<.0002 and maxima['sole_orientation_rad']<.003
    assert maxima['loop_closure_rad']<.0002
    assert not body_overlaps and not motor_overlaps and floor_min==0
    return report


if __name__=='__main__':validate()
