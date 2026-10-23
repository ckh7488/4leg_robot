"""BAM 1.0.2 M6 + explicit transport delay and firmware PWM voltage slew.
Only CPU MuJoCo integration is verified here. Fitted parameters are reference data.
"""
from pathlib import Path
from collections import deque
import math,numpy as np
from bam.model import load_model
from bam.mujoco import MujocoController
class BamController:
 def __init__(self,m,d,config):
  self.config=config;self.m=m;self.d=d
  self.names=[m.actuator(i).name for i in range(m.nu)]
  self.fit=load_model(motor_name='xl330',model='m6')
  self.fit.actuator.vin=config['motor']['supply_v']
  self.fit.actuator.kp=config['control']['position_p_gain_initial']
  self.controller=MujocoController(self.fit,self.names,m,d)
  self.delay_seconds=config['motor']['fit_delay_s']
  self.delay_steps=math.ceil(self.delay_seconds/m.opt.timestep)
  self.home=d.qpos[self.controller.qpos_indexes].copy()
  self.history=deque([self.home.copy() for _ in range(self.delay_steps+1)],maxlen=self.delay_steps+1)
  self.slew_state=np.zeros(m.nu)
  self.velocity_history=deque([np.zeros(m.nu) for _ in range(3)],maxlen=3)
  self.outer_target=self.home.copy();self.tick=0
  self.bus_steps=max(1,round(1/(config['control']['mcu_joint_bus_hz']*m.opt.timestep)))
  original=self.fit.actuator.compute_control
  def limited(qt,q,dq,dt):
   raw=original(qt,q,dq,dt)
   # Register 62 uses 1.977 mV/ms per count; numerically V/s.
   dv=config['control']['initial_pwm_slope']*1.977*max(0.0,float(dt))
   self.slew_state=np.clip(raw,self.slew_state-dv,self.slew_state+dv)
   return self.slew_state.copy()
  self.fit.actuator.compute_control=limited
 def update(self,target):
  self.velocity_history.append(self.d.qvel[self.controller.dof_indexes].copy())
  if self.tick%self.bus_steps==0:
   correction=self.config['control'].get('velocity_feedback_s',0.0)*self.velocity_history[0]
   self.outer_target=np.asarray(target,dtype=float)-np.clip(correction,-.25,.25)
  self.tick+=1
  self.history.append(self.outer_target.copy())
  self.controller.q_target=self.history[0]
  self.controller.update()
