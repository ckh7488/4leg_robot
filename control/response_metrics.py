"""Step metrics; times are measured after the command in seconds.

Speed (first motion, rise to 90% of the reached displacement) and accuracy (terminal error
against the commanded angle) are reported separately. Goal-band settling is diagnostic.
"""
import math

def step_metrics(times_s, positions_deg, baseline_deg, goal_deg, command_step_deg,
                 tolerance_deg=0.2, min_hold_s=0.1,
                 max_terminal_span_deg=0.1, max_terminal_drift_deg_s=1.0):
    times = list(map(float, times_s)); values = list(map(float, positions_deg))
    if len(times)!=len(values) or not times:
        raise ValueError('Equal nonempty time and position arrays required')
    if not all(math.isfinite(x) for x in [*times,*values,baseline_deg,goal_deg,command_step_deg,tolerance_deg,min_hold_s,
                                        max_terminal_span_deg,max_terminal_drift_deg_s]):
        raise ValueError('Metrics require finite values')
    if times[0]<0 or any(b<=a for a,b in zip(times,times[1:])):
        raise ValueError('Times must be nonnegative and strictly increasing')
    if min(tolerance_deg,min_hold_s,max_terminal_span_deg,max_terminal_drift_deg_s)<=0:
        raise ValueError('Positive tolerance, hold duration and stability limits required')
    if abs(goal_deg-baseline_deg)<1e-12 or abs(command_step_deg)<1e-12:
        raise ValueError('Nonzero step required')
    direction = 1 if goal_deg>baseline_deg else -1
    motion_band = max(abs(command_step_deg)*0.01, 0.05)
    def first(predicate):
        return next((round(t*1000,6) for t,q in zip(times,values) if predicate(q)), None)
    def settled(target):
        outside=[i for i,q in enumerate(values) if abs(q-target)>tolerance_deg]
        index=outside[-1]+1 if outside else 0
        if index>=len(values) or times[-1]-times[index]<min_hold_s-1e-12:
            return None
        return round(times[index]*1000,6)
    terminal_pairs=[(t,q) for t,q in zip(times,values) if t>=times[-1]-min_hold_s-1e-12]
    terminal=[q for t,q in terminal_pairs]
    final=sum(terminal)/len(terminal)
    terminal_duration=terminal_pairs[-1][0]-terminal_pairs[0][0]
    terminal_span=max(terminal)-min(terminal)
    mean_t=sum(t for t,q in terminal_pairs)/len(terminal_pairs)
    variance_t=sum((t-mean_t)**2 for t,q in terminal_pairs)
    drift=(sum((t-mean_t)*(q-final) for t,q in terminal_pairs)/variance_t) if variance_t else None
    terminal_stable=(terminal_duration>=min_hold_s-1e-12 and terminal_span<=max_terminal_span_deg
                     and drift is not None and abs(drift)<=max_terminal_drift_deg_s)
    # A finite-window mean is a usable final-position estimate only after a stability check.
    # Speed relative to this estimate does not establish command tracking accuracy.
    reached=direction*(final-baseline_deg)
    rise_valid=terminal_stable and reached>motion_band
    rise=first(lambda q:direction*(q-baseline_deg)>=0.9*reached) if rise_valid else None
    return {'first_motion_ms':first(lambda q:direction*(q-baseline_deg)>motion_band),
            'time_to_90percent_of_goal_ms':first(lambda q:direction*(q-baseline_deg)>=abs(goal_deg-baseline_deg)*0.9),
            'rise_to_90percent_of_final_ms':rise,
            'settle_to_goal_band_ms':settled(goal_deg),
            'settle_to_final_band_ms_diagnostic':settled(final),
            'terminal_mean_deg':final,'terminal_goal_error_deg':final-goal_deg,
            'terminal_estimate_stable':terminal_stable,'terminal_span_deg':terminal_span,
            'terminal_drift_deg_s':drift,'terminal_window_ms':round(terminal_duration*1000,6),
            'speed_metric_valid':rise_valid,
            'actual_to_command_step_ratio':(final-baseline_deg)/command_step_deg,
            'actual_step_deg':final-baseline_deg,'goal_band_deg':tolerance_deg,
            'required_hold_ms':min_hold_s*1000,'observation_ms':round(times[-1]*1000,6)}

def response_assessment(metrics, targets, pre_stable=True):
    """Keep deterministic speed screening separate from unbudgeted accuracy acceptance."""
    onset=metrics['first_motion_ms'];rise=metrics['rise_to_90percent_of_final_ms']
    checks={'pre_step_stable':bool(pre_stable),
            'terminal_estimate_stable':metrics['terminal_estimate_stable'],
            'speed_metric_valid':metrics['speed_metric_valid'],
            'onset_within_design_target':onset is not None and onset<=targets['command_to_first_motion_p95_ms'],
            'rise_within_design_target':rise is not None and rise<=targets['step_rise_to_90pct_of_reached_p95_ms']}
    return {'numerical_target_checks':checks,'single_trial_targets_met':all(checks.values()),
            'accuracy_target_met':None,'overall_response_accepted':None,
            'acceptance_status':'pending_accuracy_budget_and_hardware_validation'}
