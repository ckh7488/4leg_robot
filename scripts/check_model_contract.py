"""Cross-check independent URDF/MuJoCo kinematics, joint contracts and CAD masses."""
from pathlib import Path
import json, math, sys, xml.etree.ElementTree as ET
import mujoco
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
C = json.loads((ROOT/'design.json').read_text(encoding='utf-8'))
D = json.loads((ROOT/'models/dynamics.json').read_text(encoding='utf-8'))
J = json.loads((ROOT/'control/joint-order.json').read_text(encoding='utf-8'))
U = ET.parse(ROOT/'models/urdf/robot.urdf').getroot()
m = mujoco.MjModel.from_xml_path(str(ROOT/'models/mujoco/robot.xml'))
d = mujoco.MjData(m)
urdf_joints = U.findall('joint')
checks = {'joint_count_12': m.nu==12 and len(urdf_joints)==12,
          'joint_order': [j.attrib['name'] for j in urdf_joints]==J['joint_names']==[m.joint(i+1).name for i in range(12)],
          'mass_matches_CAD': abs(m.body_mass.sum()-D['total_mass_kg'])<1e-10,
          'mass_below_target': D['total_mass_kg']<=C['nominal_mass_target_kg'],
          'mass_budget_below_target': sum(C['mass_budget_kg'].values())<=C['nominal_mass_target_kg'],
          'positive_inertias': all(np.linalg.eigvalsh(b['inertia_kgm2']).min()>0 for b in D['bodies'].values())}
for joint in urdf_joints:
    name = joint.attrib['name']; idx = m.joint(name).id
    axis = np.fromstring(joint.find('axis').attrib['xyz'], sep=' ')
    checks[name+'_axis'] = bool(np.allclose(m.jnt_axis[idx], axis, atol=1e-10))
    limit = joint.find('limit').attrib
    checks[name+'_limits'] = bool(np.allclose(m.jnt_range[idx], [float(limit['lower']),float(limit['upper'])], atol=1e-10))
# Independent URDF chain forward kinematics at home and 32 deterministic random poses.
def rotate(axis, q):
    axis = axis/np.linalg.norm(axis); x,y,z=axis
    cross = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(q)*cross+(1-math.cos(q))*(cross@cross)
rng = np.random.default_rng(330)
errors = []
for sample in range(33):
    mujoco.mj_resetDataKeyframe(m,d,0)
    q = J['home_rad'] if sample==0 else [rng.uniform(*C['joint_limits_rad'][name.split('_')[1]]) for name in J['joint_names']]
    d.qpos[7:] = q; mujoco.mj_forward(m,d)
    frames = {'trunk': (d.xpos[m.body('trunk').id].copy(),np.eye(3))}
    for index,joint in enumerate(urdf_joints):
        parent=joint.find('parent').attrib['link']; child=joint.find('child').attrib['link']
        origin=joint.find('origin').attrib
        if origin.get('rpy','0 0 0')!='0 0 0': raise ValueError('Nonzero URDF origin rotation requires explicit implementation')
        pos,rot=frames[parent]; xyz=np.fromstring(origin['xyz'],sep=' ')
        axis=np.fromstring(joint.find('axis').attrib['xyz'],sep=' ')
        frames[child]=(pos+rot@xyz,rot@rotate(axis,q[index]))
    for leg in ['FL','FR','RL','RR']:
        pos,rot=frames[leg+'_calf']; side=1 if leg.endswith('L') else -1
        g=C['geometry_mm']; foot=pos+rot@np.array([0,side*g['foot_lateral_offset']/1000,-g['lower_foot_center_length']/1000])
        errors.append(float(np.linalg.norm(foot-d.geom(leg+'_foot').xpos)))
checks['URDF_MuJoCo_foot_positions'] = max(errors)<1e-8
checks = {key: bool(value) for key,value in checks.items()}
report={'scope':'33 poses; independent URDF chain vs MuJoCo, mass and joint consistency; no Isaac execution',
        'checks':checks,'max_foot_error_m':max(errors),'mass_kg':D['total_mass_kg'],
        'mass_target_kg':C['nominal_mass_target_kg'],'pass':all(checks.values())}
(ROOT/'reports/model-contract.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'pass':report['pass'],'max_foot_error_m':max(errors),'failed':[k for k,v in checks.items() if not v]}))
sys.exit(0 if report['pass'] else 1)
