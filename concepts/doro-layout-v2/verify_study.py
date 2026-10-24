"""Independent MuJoCo distance check for the selected ankle's 30 sampled poses."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import mujoco

HERE=Path(__file__).resolve().parent
C=json.loads((HERE/'design.json').read_text(encoding='utf-8'))
R=json.loads((HERE/'report.json').read_text(encoding='utf-8'))
expected=hashlib.sha256((HERE/'design.json').read_text(encoding='utf-8').encode()).hexdigest()
assert R['input_sha256']==expected,'Regenerate study first'
P=C['ankle_linkage'];M=C['motor_references']['large']
cases=[r for r in R['ankle_linkage']['cases'] if r['radius_m']==P['display_radius_m']]
cases+=list(R['ankle_at_body_reference_poses'].values())
gaps=[];slack=[]
for case in cases:
    assert case['reachable'] and not case.get('singular',False)
    root=ET.Element('mujoco');world=ET.SubElement(root,'worldbody')
    def vals(x):return ' '.join(str(v) for v in x)
    for i,center in enumerate(P['motor_housing_centers_m']):
        ET.SubElement(world,'geom',name=f'm{i}',type='cylinder',pos=vals(center),size=vals([M['diameter_mm']/2000,M['length_mm']/2000]),quat='.707106781187 .707106781187 0 0',contype='0',conaffinity='0')
    for i,(a,b) in enumerate(zip(case['crank_points_m'],case['foot_points_m'])):
        ET.SubElement(world,'geom',name=f'r{i}',type='capsule',fromto=vals([*a,*b]),size=str(P['rod_radius_m']),contype='0',conaffinity='0')
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    actual=min(mujoco.mj_geomDistance(m,d,m.geom(f'm{i}').id,m.geom(f'r{j}').id,1.0,None) for i in range(2) for j in range(2))
    lower=case['rod_to_motor_clearance_lower_bound_m']
    assert actual>=lower-1e-6,(actual,lower)
    assert actual>0
    gaps.append(actual);slack.append(actual-lower)
result={'status':'selected rod-capsule vs motor-cylinder pairs only; not full mechanical clearance',
        'input_sha256':expected,'mujoco_version':mujoco.__version__,'cases_checked':len(cases),
        'minimum_mujoco_gap_m':min(gaps),'maximum_gap_minus_conservative_bound_m':max(slack)}
(HERE/'verification.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n')
print(json.dumps(result,indent=2))
