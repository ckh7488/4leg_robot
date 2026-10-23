"""Reproducible CPU checks: inertias, joint contract and 5s stand with BAM.
This is not an RL gait, hardware test or runtime/thermal certification.
"""
from pathlib import Path
import sys,json,math
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from control.bam_controller import BamController
C=json.loads((ROOT/'design.json').read_text());D=json.loads((ROOT/'models/dynamics.json').read_text())
m=mujoco.MjModel.from_xml_path(str(ROOT/'models/mujoco/robot.xml'));d=mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
expected=json.loads((ROOT/'cad/home-kinematics.json').read_text())
foot_errors=[]
for k in expected['legs']:
 p=d.geom(k['name']+'_foot').xpos
 foot_errors.append(float(np.linalg.norm(p-np.array(k['foot_center'])/1000)))
eigs={k:np.linalg.eigvalsh(b['inertia_kgm2']).tolist() for k,b in D['bodies'].items()}
ctrl=BamController(m,d,C);home=np.array(C['home_rad']*4)
minz=float(d.qpos[2]);maxlean=0;peak=0
for i in range(5000):
 ctrl.update(home);mujoco.mj_step(m,d)
 minz=min(minz,float(d.qpos[2]))
 maxlean=max(maxlean,math.degrees(math.acos(min(1,max(-1,float(d.xmat[m.body('trunk').id].reshape(3,3)[2,2]))))))
 peak=max(peak,float(np.max(np.abs(d.qfrc_actuator[6:]))))
 if not np.all(np.isfinite(d.qpos)):raise RuntimeError('non-finite simulation state')
feet=set()
for con in d.contact:
 for gi in [con.geom1,con.geom2]:
  n=m.geom(gi).name
  if n and n.endswith('_foot'):feet.add(n)
homeheight=expected['hip_height_mm']/1000
report={'scope':'5 seconds deterministic standing, BAM M6 + transport delay + PWM slew, no RL or walking',
 'mujoco_version':mujoco.__version__,'joint_count':m.nu,'mass_kg':float(m.body_mass.sum()),
 'cad_foot_max_error_m':max(foot_errors),'inertia_eigenvalues_positive':all(min(x)>0 for x in eigs.values()),
 'min_base_height_m':minz,'end_base_height_m':float(d.qpos[2]),'max_base_lean_deg':maxlean,
 'peak_actuator_torque_nm':peak,'feet_contacting_at_end':sorted(feet),
 'motor_voltage_v':C['motor']['supply_v'],'command_delay_model_s':ctrl.delay_seconds,
 'pwm_slope_register':C['control']['initial_pwm_slope'],
 'pass':m.nu==12 and all(min(x)>0 for x in eigs.values()) and abs(float(m.body_mass.sum())-D['total_mass_kg'])<1e-10 and float(m.body_mass.sum())<=C['nominal_mass_target_kg'] and max(foot_errors)<1e-8 and minz>.70*homeheight and maxlean<10 and len(feet)==4,
 'limitations':['No hardware calibration at 5V.','No RL gait, disturbances or terrain trial.','Collision geometry is a documented primitive approximation, not the whole CAD mesh.','Supply droop, backlash, temperature and continuous torque require further identification.']}
(ROOT/'reports/simulation-stand.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2),flush=True)
if not report['pass']:sys.exit(1)
