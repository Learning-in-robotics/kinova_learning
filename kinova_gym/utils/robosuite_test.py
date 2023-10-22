# test the robosuite environment with kinova robot
from robosuite.environments.manipulation.single_arm_env import SingleArmEnv
from robosuite.models.arenas import FloorArena
from robosuite.models.tasks.task import Task

import numpy as np

import mujoco
import mujoco.viewer
import time

# create the environment
class SimpleEnv(SingleArmEnv):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _load_model(self):
        super()._load_model()
        
        mujoco_arena = FloorArena()

        mujoco_arena.set_origin([0, 0, 0])

        # set robot offset
        self.robots[0].robot_model.set_base_xpos([0, 0, 0.85])

        self.model = Task(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=[],
        )

    def reward(self, action):
        return 0

# create the environment
env = SimpleEnv(
    robots="Kinova3",
    mount_types="TableMount",
    has_renderer=True,
    has_offscreen_renderer=True,
    use_camera_obs=False,
)

env.reset()

# # get the mujoco model and data
m = env.sim.model._model
d = env.sim.data._data

# run mujoco simulation
with mujoco.viewer.launch_passive(m, d) as viewer:

    # set viewer cam properties
    viewer.cam.distance = 3.35
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -45
    viewer.cam.lookat[0] = -0.5
    viewer.cam.lookat[1] = 0.0
    viewer.cam.lookat[2] = 1.015

    while viewer.is_running() and True:

        num_joints = 7
        for j in range(1, num_joints):
            d.qpos[j] = 0
            d.qvel[j] = 0
            d.qacc[j] = 0
        # d.qvel[0] = 10
        # set first joint torque to 10
        d.ctrl[0] = 2.5
        
        mujoco.mj_step(m, d)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

# for i in range(1000):
#     action = np.random.randn(env.robots[0].dof) # sample random action
#     obs, reward, done, info = env.step(np.array([5, 0, 0, 0, 0, 0, 0, 0]))  # take action in the environment
#     # get ee position
#     ee_pos = env._eef_xpos
#     print(ee_pos)
#     env.render()  # render on display