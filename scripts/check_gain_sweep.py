"""What-if position P gain sweep on the step-response fixture.

Shows how speed and static compliance scale with the firmware P gain. The design gain in
design.json is NOT changed by this script. BAM is a reference fit, not our 5V unit, and it
does not predict high-gain chatter, heating or backlash, so no gain is recommended here.
"""
from pathlib import Path
import copy, json, sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.check_step_response import run_cases, compliance

GAINS=[400,800,1600,3200]

def complete_max(rows, field):
    """A missing metric must not disappear into a best-case summary."""
    values=[r[field] for r in rows]
    return max(values) if values and all(v is not None for v in values) else None

def main():
    C=json.loads((ROOT/'design.json').read_text(encoding='utf-8'))
    targets=C['response_acceptance_targets'];rows=[]
    gains=sorted(set(GAINS+[C['control']['position_p_gain_initial']]))
    for kp in gains:
        c=copy.deepcopy(C);c['control']['position_p_gain_initial']=kp
        cases=run_cases(c);free=[r for r in cases if r['nominal_gravity_load_nm_at_zero']==0]
        rows.append({'firmware_kp':kp,'is_design_gain':kp==C['control']['position_p_gain_initial'],
                     'first_motion_ms_max':complete_max(cases,'first_motion_ms'),
                     'rise_to_90percent_of_final_ms_max':complete_max(cases,'rise_to_90percent_of_final_ms'),
                     'unloaded_rise_to_90percent_ms':complete_max(free,'rise_to_90percent_of_final_ms'),
                     'invalid_speed_case_count':sum(not r['speed_metric_valid'] for r in cases),
                     'unloaded_terminal_goal_error_deg_max':max(abs(r['terminal_goal_error_deg']) for r in free),
                     'loaded_terminal_goal_error_deg_max':max(abs(r['terminal_goal_error_deg']) for r in cases if r['nominal_gravity_load_nm_at_zero']>0),
                     'actual_to_command_step_ratio_min':min(r['actual_to_command_step_ratio'] for r in cases),
                     'static_compliance':compliance(cases),
                     'speed_targets_met_all_cases':all(r['single_trial_targets_met'] for r in cases),
                     'accuracy_target_met':None,'overall_response_accepted':None})
    design=C['control']['position_p_gain_initial']
    base=next(r for r in rows if r['firmware_kp']==design)
    worst=max(c['compliance_deg_per_nm'] for c in base['static_compliance'])
    out={'status':'What-if simulation sweep only; design gain unchanged; no hardware sample',
         'design_kp':design,'gains':gains,'targets':targets,'results':rows,
         'observation':{'compliance_times_kp_at_0.1nm':{r['firmware_kp']:round(r['firmware_kp']*r['static_compliance'][-1]['compliance_deg_per_nm']) for r in rows},
                        'note':'Roughly constant product means compliance is roughly inverse to P gain in this model.',
                        'kp_needed_for_0.2deg_at_0.1nm_if_inverse_scaling_from_design_gain_holds':round(design*worst*0.1/0.2),
                        'extrapolation_note':'Approximate static offset scaling only; not a tested required gain, settling guarantee, or proof of infeasibility.',
                        'register_max':16383},
         'limitations':['BAM M6 reference fit, not identified on our 5V actuator.',
                        'High gains may chatter, heat or excite backlash on hardware; not modeled.',
                        'Pendulum fixture, not the assembled leg.']}
    (ROOT/'reports/gain-sweep.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'report':'reports/gain-sweep.json','rows':[{k:r[k] for k in ['firmware_kp','unloaded_rise_to_90percent_ms','rise_to_90percent_of_final_ms_max','speed_targets_met_all_cases']}|{'compliance_deg_per_nm':[round(c['compliance_deg_per_nm'],1) for c in r['static_compliance']]} for r in rows]},indent=1))
    return 0

if __name__=='__main__':sys.exit(main())
