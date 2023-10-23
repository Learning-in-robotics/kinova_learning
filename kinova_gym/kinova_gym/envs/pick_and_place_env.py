# test the robosuite environment with kinova robot
from robosuite.environments.manipulation.single_arm_env import SingleArmEnv
from robosuite.models.arenas import FloorArena
from robosuite.models.tasks.task import Task
from robosuite.models.objects import BoxObject
from robosuite.utils.placement_samplers import UniformRandomSampler
from robosuite.wrappers import GymWrapper 

import numpy as np

import mujoco
import mujoco.viewer
import time

# create the environment
class SimpleEnv(SingleArmEnv):
    def __init__(self,
        robots,
        env_configuration="default",
        controller_configs=None,
        gripper_types="default",
        mount_types="default",
        initialization_noise="default",
        use_camera_obs=False,
        use_object_obs=False,
        reward_scale=1.0,
        reward_shaping=False,
        placement_initializer=None,
        has_renderer=False,
        has_offscreen_renderer=True,
        render_camera="frontview",
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=20,
        horizon=1000,
        ignore_done=False,
        hard_reset=True,
        camera_names="agentview",
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
        camera_segmentations=None,  # {None, instance, class, element}
        renderer="mujoco",
        renderer_config=None,
    ):
        
        self.table_offset = np.array((0, 0, 1.1))

        self.placement_initializer = placement_initializer

        self.reward_scale = reward_scale
        self.reward_shaping = reward_shaping
        self.use_camera_obs = use_camera_obs
        self.use_object_obs = use_object_obs

        super().__init__(
            robots=robots,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            mount_types=mount_types,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
            has_renderer=has_renderer,
            has_offscreen_renderer=has_offscreen_renderer,
            render_camera=render_camera,
            render_collision_mesh=render_collision_mesh,
            render_visual_mesh=render_visual_mesh,
            render_gpu_device_id=render_gpu_device_id,
            control_freq=control_freq,
            horizon=horizon,
            ignore_done=ignore_done,
            hard_reset=hard_reset,
            camera_names=camera_names,
            camera_heights=camera_heights,
            camera_widths=camera_widths,
            camera_depths=camera_depths,
            camera_segmentations=camera_segmentations,
            renderer=renderer,
            renderer_config=renderer_config,
        )

    def _load_model(self):
        super()._load_model()
        
        mujoco_arena = FloorArena()

        mujoco_arena.set_origin([0, 0, 0])

        # set robot offset
        self.robots[0].robot_model.set_base_xpos([0, 0, 0.85])

        # create objects
        box_red = BoxObject(
            name="box_red",
            size=[0.025, 0.025, 0.025],
            rgba=[1, 0, 0, 1],
        )

        box_green = BoxObject(
            name="box_green",
            size=[0.025, 0.025, 0.025],
            rgba=[0, 1, 0, 1],
        )

        box_blue = BoxObject(
            name="box_blue",
            size=[0.025, 0.025, 0.025],
            rgba=[0, 0, 1, 1],
        )

        boxes = [box_red, box_green, box_blue]

        # Create placement initializer
        if self.placement_initializer is not None:
            self.placement_initializer.reset()
            self.placement_initializer.add_objects(boxes)
        else:
            self.placement_initializer = UniformRandomSampler(
                name="ObjectSampler",
                mujoco_objects=boxes,
                x_range=[-1.0, 0.15],
                y_range=[-0.50, 0.0],
                rotation=None,
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=True,
                reference_pos=self.table_offset,
                z_offset=0.01,
            )

        # box_red.get_obj().set("pos", "0 0 1.1")

        self.model = Task(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=boxes,
        )

    def reward(self, action):
        return 0
    
    def reset(self, **kwargs):
        """
        Extends the superclass reset function to add the placement initializer reset functionality.
        """
        return super().reset(**kwargs)
    
    def _reset_internal(self):
        """
        Resets simulation internal configurations.
        """
        super()._reset_internal()

        # Reset all object positions using initializer sampler if we're not directly loading from an xml
        if not self.deterministic_reset:

            # Sample from the placement initializer for all objects
            object_placements = self.placement_initializer.sample()

            # Loop through all objects and reset their positions
            for obj_pos, obj_quat, obj in object_placements.values():
                self.sim.data.set_joint_qpos(obj.joints[0], np.concatenate([np.array(obj_pos), np.array(obj_quat)]))

    def init_renderer(self):
        m = self.sim.model._model
        d = self.sim.data._data

        # run mujoco simulation
        self.viewer = mujoco.viewer.launch_passive(m, d)
        
        # set viewer cam properties
        self.viewer.cam.distance = 3.35
        self.viewer.cam.azimuth = 90
        self.viewer.cam.elevation = -45
        self.viewer.cam.lookat[0] = -0.5
        self.viewer.cam.lookat[1] = 0.0
        self.viewer.cam.lookat[2] = 1.015

    def render(self):
        self.viewer.sync()


if __name__ == "__main__":

  # create the environment
  env = SimpleEnv(
      robots="Kinova3",
      mount_types="TableMount",
      has_renderer=True,
      has_offscreen_renderer=True,
      use_camera_obs=False,
      render_collision_mesh=False,
      render_visual_mesh=True,
      ignore_done=True,
  )

  env = GymWrapper(env)

  env.reset(seed=0)

  env.env.init_renderer()

  # run the simulation
  for i in range(1000):
      env.step([1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
      env.render()
      time.sleep(0.01)