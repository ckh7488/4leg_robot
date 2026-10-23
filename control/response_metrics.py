"""Command-referenced step metrics; times are measured after the command in seconds."""
import math

def step_metrics(times_s, positions_deg, baseline_deg, goal_deg, command_step_deg,
                 tolerance_deg=0.2, min_hold_s=0.1):
    times = list(map(float, times_s)); values = list(map(float, positions_deg))
    if len(times)!=len(values) or not times:
        raise ValueError('Equal nonempty time and position arrays required')
    if not all(math.isfinite(x) for x in [*times,*values,baseline_deg,goal_deg,command_step_deg,tolerance_deg,min_hold_s]):
        raise ValueError('Metrics require finite values')
    if times[0]<0 or any(b<=a for a,b in zip(times,times[1:])):
        raise ValueError('Times must be nonnegative and strictly increasing')
    if tolerance_deg<=0 or min_hold_s<=0:
        raise ValueError('Positive tolerance and hold duration required')
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
    terminal=[q for t,q in zip(times,values) if t>=times[-1]-min_hold_s-1e-12]
    final=sum(terminal)/len(terminal)
    return {'first_motion_ms':first(lambda q:direction*(q-baseline_deg)>motion_band),
            'time_to_90percent_of_goal_ms':first(lambda q:direction*(q-baseline_deg)>=abs(goal_deg-baseline_deg)*0.9),
            'settle_to_goal_band_ms':settled(goal_deg),
            'settle_to_final_band_ms_diagnostic':settled(final),
            'terminal_mean_deg':final,'terminal_goal_error_deg':final-goal_deg,
            'actual_step_deg':final-baseline_deg,'goal_band_deg':tolerance_deg,
            'required_hold_ms':min_hold_s*1000,'observation_ms':round(times[-1]*1000,6)}
