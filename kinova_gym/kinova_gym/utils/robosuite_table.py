import mujoco
import mujoco.viewer
import time

from robosuite.models import MujocoWorldBase
from robosuite.models.robots import Kinova3
from robosuite.models.grippers import gripper_factory
from robosuite.models.arenas import TableArena
from robosuite.models.objects import BallObject, BoxObject
from robosuite.utils.mjcf_utils import new_joint

world = MujocoWorldBase()
mujoco_robot = Kinova3()
gripper = gripper_factory('Robotiq85Gripper')
mujoco_robot.add_gripper(gripper)
mujoco_robot.set_base_xpos([0.45, 0, 0.8])
world.merge(mujoco_robot)
mujoco_arena = TableArena()
mujoco_arena.set_origin([0.8, 0, 0])
world.merge(mujoco_arena)

# #add blue ball
# sphere = BallObject(
#     name="sphere",
#     size=[0.04],
#     rgba=[0, 0.5, 0.5, 1]).get_obj()
# sphere.set('pos', '1.0 0 1.0')
# world.worldbody.append(sphere)

#add red cube
cube_red = BoxObject(
    name="cube_red",
    size=[0.03,0.03,0.03],
    rgba=[1, 0, 0, 0]).get_obj()
cube_red.set('pos', '0.8 0 1.0')
world.worldbody.append(cube_red)

#add green cube
cube_green = BoxObject(
    name="cube_green",
    size=[0.03,0.03,0.03],
    rgba=[0, 1, 0, 0]).get_obj()
cube_green.set('pos', '1.0 0 1.0')
world.worldbody.append(cube_green)

m = world.get_model(mode="mujoco")

d = mujoco.MjData(m)

# run mujoco simulation
with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 3000:
        step_start = time.time()

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        # set all joint positions to 0
        # num_joints = 7
        # for j in range(1, num_joints):
        #     d.qpos[j] = 0
        #     d.qvel[j] = 0
        #     d.qacc[j] = 0
        # d.qvel[0] = 10
        # set first joint torque to 10
        # d.ctrl[0] = 5
        
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