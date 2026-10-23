"""Regression tests for biased final position, false settling, sample timestamps and speed/accuracy separation."""
from pathlib import Path
import sys, unittest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from control.response_metrics import step_metrics

class MetricsTest(unittest.TestCase):
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
    def test_nonfinite_and_nonmonotonic_samples_rejected(self):
        for ts,qs in [([.01,.02],[0,float('nan')]),([.01,.01],[0,10]),([] ,[])]:
            with self.assertRaises(ValueError):step_metrics(ts,qs,0,10,10)

if __name__=='__main__':unittest.main()
