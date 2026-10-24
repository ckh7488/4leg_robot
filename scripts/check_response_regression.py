"""Regression tests for biased final position, false settling, sample timestamps and speed/accuracy separation."""
from pathlib import Path
import sys, unittest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from control.response_metrics import response_assessment, step_metrics

class MetricsTest(unittest.TestCase):
    targets={'command_to_first_motion_p95_ms':40,'step_rise_to_90pct_of_reached_p95_ms':150}
    def trace(self, values, goal=10):
        return step_metrics([(i+1)*.01 for i in range(len(values))],values,0,goal,goal)
    def test_biased_final_is_not_goal_settling(self):
        r=self.trace([0]*3+[12.5]*30)
        self.assertIsNone(r['settle_to_goal_band_ms'])
        self.assertEqual(r['settle_to_final_band_ms_diagnostic'],40)
        self.assertAlmostEqual(r['terminal_goal_error_deg'],2.5)
    def test_transient_goal_crossing_is_not_settling(self):
        self.assertIsNone(self.trace([0]*3+[10]*20+[12]*20)['settle_to_goal_band_ms'])
    def test_negative_step_and_timestamps(self):
        r=self.trace([0,0,-10]+[-10]*20,goal=-10)
        self.assertEqual(r['first_motion_ms'],30)
        self.assertEqual(r['settle_to_goal_band_ms'],30)
    def test_insufficient_hold_is_not_settling(self):
        self.assertIsNone(self.trace([0]*20+[10]*5)['settle_to_goal_band_ms'])
    def test_no_motion(self):
        r=self.trace([0]*30)
        self.assertIsNone(r['first_motion_ms'])
        self.assertIsNone(r['settle_to_goal_band_ms'])
    def test_rise_uses_reached_displacement_not_goal(self):
        # Joint reaches only 8 of 10 degrees (load offset); rise is judged on the 8 degrees.
        r=self.trace([0,0,4,7.3,7.9]+[8]*20)
        self.assertEqual(r['rise_to_90percent_of_final_ms'],40)
        self.assertIsNone(r['time_to_90percent_of_goal_ms'])
        self.assertAlmostEqual(r['terminal_goal_error_deg'],-2)
    def test_rise_absent_without_motion_toward_goal(self):
        self.assertIsNone(self.trace([0]*5+[-1]*20)['rise_to_90percent_of_final_ms'])
        self.assertIsNone(self.trace([0]*30)['rise_to_90percent_of_final_ms'])
    def test_negative_rise(self):
        self.assertEqual(self.trace([0,-5,-9.5]+[-10]*20,goal=-10)['rise_to_90percent_of_final_ms'],30)
    def test_continuous_oscillation_cannot_pass_speed(self):
        r=self.trace([0,0]+[5,15]*14)
        self.assertEqual(r['time_to_90percent_of_goal_ms'],40)
        self.assertFalse(r['terminal_estimate_stable'])
        self.assertIsNone(r['rise_to_90percent_of_final_ms'])
        self.assertFalse(response_assessment(r,self.targets)['single_trial_targets_met'])
    def test_short_record_cannot_estimate_final_position(self):
        r=self.trace([0,5,10])
        self.assertFalse(r['terminal_estimate_stable'])
        self.assertIsNone(r['rise_to_90percent_of_final_ms'])
    def test_terminal_drift_rejected_even_with_wide_span_limit(self):
        values=[2*i*.01 for i in range(30)]
        r=step_metrics([(i+1)*.01 for i in range(30)],values,0,10,10,
                       max_terminal_span_deg=1.0)
        self.assertAlmostEqual(r['terminal_drift_deg_s'],2)
        self.assertFalse(r['terminal_estimate_stable'])
        self.assertIsNone(r['rise_to_90percent_of_final_ms'])
    def test_tiny_fast_motion_fails_speed_screening(self):
        for reached in [.11,.2,4.99,-.11,-.2,-4.99]:
            r=self.trace([0,0]+[reached]*28,goal=10 if reached>0 else -10)
            a=response_assessment(r,self.targets)
            self.assertFalse(a['single_trial_targets_met'])
            self.assertFalse(r['command_displacement_sufficient'])
            self.assertIsNone(r['rise_to_90percent_of_final_ms'])
            self.assertIsNone(a['accuracy_target_met'])
            self.assertIsNone(a['overall_response_accepted'])
        r=self.trace([0,0]+[.2]*28)
        self.assertAlmostEqual(r['actual_to_command_step_ratio'],.02)
        self.assertAlmostEqual(r['terminal_goal_error_deg'],-9.8)
    def test_displacement_floor_is_configurable_and_not_accuracy(self):
        times=[(i+1)*.01 for i in range(30)]
        for goal in [-10,10]:
            values=[0,0]+[goal*.5]*28
            r=step_metrics(times,values,0,goal,goal,min_reached_command_fraction=.5)
            self.assertTrue(response_assessment(r,self.targets)['single_trial_targets_met'])
            self.assertIsNone(response_assessment(r,self.targets)['overall_response_accepted'])
            self.assertIsNone(step_metrics(times,values,0,goal,goal,
                                          min_reached_command_fraction=.6)['rise_to_90percent_of_final_ms'])
        for invalid in [0,1.01,float('nan')]:
            with self.assertRaises(ValueError):
                step_metrics(times,values,0,goal,goal,min_reached_command_fraction=invalid)
    def test_stable_window_with_non_aligned_samples(self):
        r=step_metrics([(i+1)*.007 for i in range(30)],[0,0]+[10]*28,0,10,10)
        self.assertTrue(r['terminal_estimate_stable'])
        self.assertGreaterEqual(r['terminal_window_ms'],100)
        self.assertEqual(r['rise_to_90percent_of_final_ms'],21)
    def test_motion_at_detection_threshold_is_not_valid_speed(self):
        r=self.trace([0,0]+[.1]*28)
        self.assertIsNone(r['first_motion_ms'])
        self.assertFalse(r['speed_metric_valid'])
    def test_unstable_pre_step_rejects_otherwise_fast_response(self):
        r=self.trace([0,0]+[10]*28)
        self.assertFalse(response_assessment(r,self.targets,pre_stable=False)['single_trial_targets_met'])
    def test_stability_limits_are_configurable(self):
        values=[0,0]+[9.95,10.05]*14
        times=[(i+1)*.01 for i in range(len(values))]
        r=step_metrics(times,values,0,10,10,max_terminal_span_deg=.05)
        self.assertIsNone(r['rise_to_90percent_of_final_ms'])
        with self.assertRaises(ValueError):
            step_metrics(times,values,0,10,10,max_terminal_span_deg=0)
    def test_nonfinite_and_nonmonotonic_samples_rejected(self):
        for ts,qs in [([.01,.02],[0,float('nan')]),([.01,.01],[0,10]),([] ,[])]:
            with self.assertRaises(ValueError):step_metrics(ts,qs,0,10,10)

if __name__=='__main__':unittest.main()
