"""Regression tests for biased final position, false settling and sample timestamps."""
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
    def test_nonfinite_and_nonmonotonic_samples_rejected(self):
        for ts,qs in [([.01,.02],[0,float('nan')]),([.01,.01],[0,10]),([] ,[])]:
            with self.assertRaises(ValueError):step_metrics(ts,qs,0,10,10)

if __name__=='__main__':unittest.main()
