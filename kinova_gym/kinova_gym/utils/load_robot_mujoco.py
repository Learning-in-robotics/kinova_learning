# load the robot using mujoco

import os
import numpy as np
import time

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('models/urdf/kinova_gen3.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    # set all joint positions to 0
    num_joints = 7
    for j in range(1, num_joints):
        d.qpos[j] = 0
        d.qvel[j] = 0
        d.qacc[j] = 0
    # d.qvel[0] = 10
    # set first joint torque to 10
    d.ctrl[0] = 0.05
    
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)