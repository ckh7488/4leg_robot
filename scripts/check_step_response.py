"""Deterministic bidirectional step screening. No hardware sample or p95 claim."""
from pathlib import Path
import argparse, json, math, re, sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import mujoco
import numpy as np
from control.bam_controller import BamController
from control.response_metrics import step_metrics

def main():
    C=json.loads((ROOT/'design.json').read_text(encoding='utf-8'))
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kp',type=int)
    parser.add_argument('--damping',type=float)
    parser.add_argument('--suffix',default='')
    parser.add_argument('--require-targets',action='store_true',help='Exit 1 if any deterministic case fails the numerical targets')
    args=parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]*',args.suffix):parser.error('Invalid suffix')
    if args.kp is not None:C['control']['position_p_gain_initial']=args.kp
    if args.damping is not None:C['control']['velocity_feedback_s']=args.damping
    rows=[];dt=.001;pre_steps=2000;post_steps=2000
    targets=C['response_acceptance_targets']
    for torque in [0,.05,.10]:
        for goal_deg in [-10,10]:
            mass=max(1e-5,torque/(9.81*.05))
            xml=f'''<mujoco><option timestep="{dt}" integrator="implicitfast"/><worldbody><body>
            <joint name="hinge" axis="0 1 0"/><inertial mass="{mass}" pos=".05 0 0" diaginertia=".0001 .0001 .0001"/>
            <geom type="capsule" fromto="0 0 0 .05 0 0" size=".003" contype="0" conaffinity="0"/>
            </body></worldbody><actuator><motor name="hinge_act" joint="hinge" ctrllimited="true" ctrlrange="-.65 .65"/></actuator></mujoco>'''
            m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
            ctrl=BamController(m,d,C);pre=[]
            for _ in range(pre_steps):
                ctrl.update([0]);mujoco.mj_step(m,d);pre.append(math.degrees(float(d.qpos[0])))
            baseline=float(d.qpos[0])*180/math.pi
            baseline_span=float(np.ptp(pre[-100:]));baseline_speed=abs(math.degrees(float(d.qvel[0])))
            command_time=float(d.time);times=[];positions=[];peak=0.0
            for _ in range(post_steps):
                ctrl.update([math.radians(goal_deg)]);mujoco.mj_step(m,d)
                times.append(float(d.time)-command_time);positions.append(math.degrees(float(d.qpos[0])))
                peak=max(peak,abs(float(d.qfrc_actuator[0])))
            metrics=step_metrics(times,positions,baseline,goal_deg,goal_deg,
                                 tolerance_deg=targets['position_tolerance_deg'],
                                 min_hold_s=targets['settle_observed_hold_ms']/1000)
            pre_stable=baseline_span<=.1 and baseline_speed<=1.0
            onset=metrics['first_motion_ms'];settle=metrics['settle_to_goal_band_ms']
            checks={'pre_step_stable':pre_stable,
                    'onset_within_design_target':onset is not None and onset<=targets['command_to_first_motion_p95_ms'],
                    'goal_settle_within_design_target':settle is not None and settle<=targets['loaded_10degree_step_settle_p95_ms']}
            rows.append({'nominal_gravity_load_nm_at_zero':torque,'command_goal_deg':goal_deg,
                         'pre_step_actual_deg':baseline,'pre_step_span_100ms_deg':baseline_span,
                         'pre_step_speed_deg_s':baseline_speed,'peak_predicted_torque_nm':peak,
                         **metrics,'numerical_target_checks':checks,'single_trial_targets_met':all(checks.values())})
    out={'status':'Deterministic simulation screening only; no hardware sample or p95 verification',
         'voltage_v':C['motor']['supply_v'],'firmware_kp':C['control']['position_p_gain_initial'],
         'velocity_feedback_s':C['control'].get('velocity_feedback_s',0.0),
         'pwm_slope':C['control']['initial_pwm_slope'],'transport_delay_reference_s':C['motor']['fit_delay_s'],
         'transport_delay_quantized_ms':ctrl.delay_steps*dt*1000,'command_bus_hz':C['control']['mcu_joint_bus_hz'],'numerical_design_targets':targets,
         'fixture':{'arm_length_m':.05,'inertia_at_com_kgm2':[.0001]*3,'pre_hold_s':pre_steps*dt,'post_observation_s':post_steps*dt,'timestep_s':dt},
         'definitions':{'goal_settle':'Within configured position tolerance of commanded absolute angle for all remaining samples, with configured minimum observed hold',
                        'first_motion':'Toward goal, more than 1% commanded step or 0.05 degree from actual pre-step position',
                        'final_band':'Diagnostic only: settling near terminal 100ms mean; never a substitute for commanded angle'},
         'results':rows,'all_single_trial_targets_met':all(r['single_trial_targets_met'] for r in rows),
         'statistical_p95_verified':False,
         'limitations':['BAM fit not identified on our 5V actuator; reference prediction only.',
                        'Pendulum test load is not the assembled robot leg.',
                        'No supply droop, backlash, temperature or sampling jitter identification.',
                        'Gravity torque changes with angle; the zero-load fixture retains a 1e-5kg minimum mass.',
                        'Firmware PWM slew is an approximation; direct measurement is needed.']}
    name='step-response'+('-'+args.suffix if args.suffix else '')+'.json'
    (ROOT/'reports'/name).write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'report':'reports/'+name,'cases':len(rows),'single_trial_targets_met':sum(r['single_trial_targets_met'] for r in rows),'statistical_p95_verified':False}))
    return 1 if args.require_targets and not out['all_single_trial_targets_met'] else 0

if __name__=='__main__':sys.exit(main())
