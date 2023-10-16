# test the robosuite environment with kinova robot
from robosuite.environments.manipulation.single_arm_env import SingleArmEnv
from robosuite.models.arenas import Arena
from robosuite.models.tasks.task import Task
from robosuite.utils.mjcf_utils import array_to_string, xml_path_completion

import numpy as np

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
)

env.reset()

for i in range(1000):
    action = np.random.randn(env.robots[0].dof) # sample random action
    obs, reward, done, info = env.step(action)  # take action in the environment
    env.render()  # render on display