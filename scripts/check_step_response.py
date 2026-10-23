"""Deterministic bidirectional step screening. No hardware sample or p95 claim.

Speed (first motion, rise to 90% of the reached displacement) is judged against design.json.
Accuracy under load is characterized as static compliance (deg per N*m) without pass/fail.
Commanded-angle goal-band settling is kept as a diagnostic.
"""
from pathlib import Path
import argparse, copy, json, math, re, sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import mujoco
import numpy as np
from control.bam_controller import BamController
from control.response_metrics import response_assessment, step_metrics

LOADS_NM=[0,.05,.10]
GOALS_DEG=[-10,10]
DT=.001;PRE_STEPS=2000;POST_STEPS=2000;ARM_M=.05

def run_cases(C):
    """Run the pendulum fixture for every load and direction with the gains in C."""
    targets=C['response_acceptance_targets'];rows=[]
    for torque in LOADS_NM:
        for goal_deg in GOALS_DEG:
            mass=max(1e-5,torque/(9.81*ARM_M))
            xml=f'''<mujoco><option timestep="{DT}" integrator="implicitfast"/><worldbody><body>
            <joint name="hinge" axis="0 1 0"/><inertial mass="{mass}" pos="{ARM_M} 0 0" diaginertia=".0001 .0001 .0001"/>
            <geom type="capsule" fromto="0 0 0 {ARM_M} 0 0" size=".003" contype="0" conaffinity="0"/>
            </body></worldbody><actuator><motor name="hinge_act" joint="hinge" ctrllimited="true" ctrlrange="-.65 .65"/></actuator></mujoco>'''
            m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
            ctrl=BamController(m,d,C);pre=[]
            for _ in range(PRE_STEPS):
                ctrl.update([0]);mujoco.mj_step(m,d);pre.append(math.degrees(float(d.qpos[0])))
            baseline=float(d.qpos[0])*180/math.pi
            baseline_span=float(np.ptp(pre[-100:]));baseline_speed=abs(math.degrees(float(d.qvel[0])))
            command_time=float(d.time);times=[];positions=[];peak=0.0
            for _ in range(POST_STEPS):
                ctrl.update([math.radians(goal_deg)]);mujoco.mj_step(m,d)
                times.append(float(d.time)-command_time);positions.append(math.degrees(float(d.qpos[0])))
                peak=max(peak,abs(float(d.qfrc_actuator[0])))
            metrics=step_metrics(times,positions,baseline,goal_deg,goal_deg,
                                 tolerance_deg=targets['diagnostic_goal_band_deg'],
                                 min_hold_s=targets['diagnostic_settle_hold_ms']/1000,
                                 max_terminal_span_deg=targets['terminal_stability_max_span_deg'],
                                 max_terminal_drift_deg_s=targets['terminal_stability_max_drift_deg_s'])
            pre_stable=baseline_span<=.1 and baseline_speed<=1.0
            rows.append({'nominal_gravity_load_nm_at_zero':torque,'command_goal_deg':goal_deg,
                         'pre_step_actual_deg':baseline,'pre_step_span_100ms_deg':baseline_span,
                         'pre_step_speed_deg_s':baseline_speed,'peak_predicted_torque_nm':peak,
                         **metrics,**response_assessment(metrics,targets,pre_stable),
                         'diagnostic_goal_settle_within_150ms':metrics['settle_to_goal_band_ms'] is not None and metrics['settle_to_goal_band_ms']<=150})
    return rows

def compliance(rows):
    """Static hold offset per unit load, from the pre-step hold at command 0."""
    free=np.mean([r['pre_step_actual_deg'] for r in rows if r['nominal_gravity_load_nm_at_zero']==0])
    out=[]
    for load in LOADS_NM:
        if load==0:continue
        hold=np.mean([r['pre_step_actual_deg'] for r in rows if r['nominal_gravity_load_nm_at_zero']==load])
        out.append({'load_nm':load,'hold_offset_deg':float(hold-free),'compliance_deg_per_nm':float((hold-free)/load)})
    return out

def main():
    C=json.loads((ROOT/'design.json').read_text(encoding='utf-8'))
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kp',type=int)
    parser.add_argument('--damping',type=float)
    parser.add_argument('--suffix',default='')
    parser.add_argument('--require-targets',action='store_true',help='Exit 1 if any deterministic case fails the speed targets')
    args=parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]*',args.suffix):parser.error('Invalid suffix')
    C=copy.deepcopy(C)
    if args.kp is not None:C['control']['position_p_gain_initial']=args.kp
    if args.damping is not None:C['control']['velocity_feedback_s']=args.damping
    rows=run_cases(C);targets=C['response_acceptance_targets']
    out={'status':'Deterministic simulation screening only; no hardware sample or p95 verification',
         'voltage_v':C['motor']['supply_v'],'firmware_kp':C['control']['position_p_gain_initial'],
         'velocity_feedback_s':C['control'].get('velocity_feedback_s',0.0),
         'pwm_slope':C['control']['initial_pwm_slope'],'transport_delay_reference_s':C['motor']['fit_delay_s'],
         'transport_delay_quantized_ms':math.ceil(C['motor']['fit_delay_s']/DT)*DT*1000,'command_bus_hz':C['control']['mcu_joint_bus_hz'],'numerical_design_targets':targets,
         'fixture':{'arm_length_m':ARM_M,'inertia_at_com_kgm2':[.0001]*3,'pre_hold_s':PRE_STEPS*DT,'post_observation_s':POST_STEPS*DT,'timestep_s':DT},
         'definitions':{'first_motion':'Toward goal, more than 1% commanded step or 0.05 degree from actual pre-step position',
                        'rise_to_90percent_of_final':'Command-to-90% latency including delay, not conventional 10%-90% rise time. Uses the terminal-window mean; undefined unless the complete window meets span/drift limits and motion toward the goal exceeds the motion threshold.',
                        'terminal_estimate_stable':'Finite-window screening only: full diagnostic hold duration, span and least-squares drift within design.json limits; does not prove asymptotic stability.',
                        'actual_to_command_step_ratio':'Actual terminal displacement from baseline divided by the commanded step. Report with absolute goal error; a small ratio can still meet speed-only targets.',
                        'static_compliance':'Accuracy: pre-step hold offset under load relative to the unloaded fixture, divided by the nominal load; characterization only',
                        'goal_settle':'Diagnostic: within the diagnostic band of the commanded angle for all remaining samples with the diagnostic hold; with P-only control under load this is not expected',
                        'final_band':'Diagnostic only: settling near terminal 100ms mean'},
         'results':rows,'static_compliance':compliance(rows),
         'all_single_trial_targets_met':all(r['single_trial_targets_met'] for r in rows),
         'accuracy_target_met':None,'overall_response_accepted':None,
         'acceptance_status':'pending_accuracy_budget_and_hardware_validation',
         'statistical_p95_verified':False,
         'limitations':['BAM fit not identified on our 5V actuator; reference prediction only.',
                        'Pendulum test load is not the assembled robot leg.',
                        'No supply droop, backlash, temperature or sampling jitter identification.',
                        'Gravity torque changes with angle; the zero-load fixture retains a 1e-5kg minimum mass.',
                        'Firmware PWM slew is an approximation; direct measurement is needed.']}
    name='step-response'+('-'+args.suffix if args.suffix else '')+'.json'
    (ROOT/'reports'/name).write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'report':'reports/'+name,'cases':len(rows),'single_trial_targets_met':sum(r['single_trial_targets_met'] for r in rows),
                      'compliance_deg_per_nm':[round(c['compliance_deg_per_nm'],1) for c in out['static_compliance']],'statistical_p95_verified':False}))
    return 1 if args.require_targets and not out['all_single_trial_targets_met'] else 0

if __name__=='__main__':sys.exit(main())
