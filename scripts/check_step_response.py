"""Deterministic predicted step response; never hardware latency evidence."""
from pathlib import Path
import json,math,sys,argparse
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from control.bam_controller import BamController
C=json.loads((ROOT/'design.json').read_text())
p=argparse.ArgumentParser();p.add_argument('--kp',type=int,default=400);p.add_argument('--damping',type=float,default=0);p.add_argument('--suffix',default='baseline');args=p.parse_args()
C['control']['position_p_gain_initial']=args.kp;C['control']['velocity_feedback_s']=args.damping
rows=[]
for torque in [0,.05,.10]:
 mass=max(1e-5,torque/(9.81*.05))
 xml=f"""<mujoco><option timestep=".001" integrator="implicitfast"/><worldbody><body>
 <joint name="hinge" axis="0 1 0"/><inertial mass="{mass}" pos=".05 0 0" diaginertia=".0001 .0001 .0001"/>
 <geom type="capsule" fromto="0 0 0 .05 0 0" size=".003" contype="0" conaffinity="0"/>
 </body></worldbody><actuator><motor name="hinge_act" joint="hinge" ctrllimited="true" ctrlrange="-.65 .65"/></actuator></mujoco>"""
 m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
 ctrl=BamController(m,d,C);step=math.radians(10);samples=[]
 for i in range(2000):
  ctrl.update([0 if i<1000 else step]);mujoco.mj_step(m,d);samples.append(float(d.qpos[0]))
 baseline=float(np.mean(samples[900:1000]));final=float(np.mean(samples[-100:]));a=np.array(samples[1000:]);amplitude=final-baseline
 threshold=max(abs(amplitude)*.01,math.radians(.05))
 changed=np.flatnonzero(abs(a-baseline)>threshold)
 tolerance=max(abs(amplitude)*.02,math.radians(.1));settle=None
 for i in range(len(a)-100):
  if np.all(abs(a[i:]-final)<=tolerance):settle=i;break
 norm=(a-baseline)/(amplitude if abs(amplitude)>1e-8 else 1);rise=np.flatnonzero(norm>=.9)
 rows.append({'time_to_90percent_ms':None if not len(rise) else int(rise[0]),'nominal_gravity_load_nm':torque,'first_motion_ms':None if not len(changed) else int(changed[0]),
 'settle_to_final_band_ms':settle,'static_goal_error_deg':math.degrees(final-step),'actual_step_deg':math.degrees(amplitude),
 'definition':'1% or 0.05deg initial movement; 2% or 0.1deg of final steady state, not zero position error'})
out={'status':'simulation prediction only; no hardware sample and no p95 claim','voltage_v':5,'firmware_kp':C['control']['position_p_gain_initial'],
 'velocity_feedback_s':args.damping,'pwm_slope':C['control']['initial_pwm_slope'],'transport_delay_reference_s':C['motor']['fit_delay_s'],'results':rows,
 'limitations':['BAM fit was not identified on our 5V actuator.','No gear backlash or controller sampling jitter.','The gravity load changes with angle.','Firmware slew wrapper is a documented approximation needing measured comparison.']}
(ROOT/'reports'/('predicted-step-response-'+args.suffix+'.json')).write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
