"""Regression: a coincident-cylinder degeneracy must not pass clearance checks."""
import unittest
import mujoco
from validate import HERE, motor_distance


class MotorClearance(unittest.TestCase):
    def setUp(self):
        self.m=mujoco.MjModel.from_xml_path(str(HERE/'robot.xml'))
        self.d=mujoco.MjData(self.m)
        mujoco.mj_resetDataKeyframe(self.m,self.d,0)
        self.a=mujoco.mj_name2id(self.m,mujoco.mjtObj.mjOBJ_GEOM,'rear_R_pitch_motor')
        self.b=mujoco.mj_name2id(self.m,mujoco.mjtObj.mjOBJ_GEOM,'rear_R_pitch_knee_drive')

    def distance(self, offset):
        self.m.geom_pos[self.b]=offset
        mujoco.mj_forward(self.m,self.d)
        return motor_distance(self.m,self.d,self.a,self.b)

    def test_exact_coincidence_rejected(self):
        self.assertLess(self.distance([0,0,0]),-.001)

    def test_partial_overlap_rejected(self):
        self.assertLess(self.distance([.06,0,0]),-.001)

    def test_separated_cylinders_allowed(self):
        self.assertGreater(self.distance([.1,0,0]),0)


if __name__=='__main__':unittest.main()
