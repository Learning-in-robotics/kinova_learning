# test the robosuite environment with kinova robot
from robosuite.environments.manipulation.single_arm_env import SingleArmEnv
from robosuite.models.arenas import Arena
from robosuite.models.tasks.task import Task
from robosuite.utils.mjcf_utils import array_to_string, xml_path_completion

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
        
        mujoco_arena = Arena(xml_path_completion("arenas/empty_arena.xml"))
        mujoco_arena.set_origin([0, 0, 0])

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
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    camera_heights=1024,
    camera_widths=1024,
)

env.reset()

# get the mujoco model and data
m = env.sim.model._model
d = env.sim.data._data

# run mujoco simulation
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
        d.ctrl[0] = 5
        
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

# for i in range(1000):
#     action = np.random.randn(env.robots[0].dof) # sample random action
#     obs, reward, done, info = env.step(np.array([5, 0, 0, 0, 0, 0, 0, 0]))  # take action in the environment
#     env.render()  # render on display