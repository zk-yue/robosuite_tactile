from robosuite.models import MujocoWorldBase

world = MujocoWorldBase()
world.save_model("./demo/scene_gen/xml/world.xml")

from robosuite.models.robots import UR5e

mujoco_robot = UR5e()
mujoco_robot.save_model("./demo/scene_gen/xml/ur5e.xml")

from robosuite.models.grippers import gripper_factory

gripper = gripper_factory('Robotiq85Gripper')
mujoco_robot.save_model("./demo/scene_gen/xml/Robotiq85Gripper.xml")
mujoco_robot.add_gripper(gripper)
mujoco_robot.save_model("./demo/scene_gen/xml/ur5e_with_Robotiq85Gripper.xml")

mujoco_robot.set_base_xpos([0, 0, 0])
world.merge(mujoco_robot)
world.save_model("./demo/scene_gen/xml/world_merged.xml")

from robosuite.models.arenas import TableArena

mujoco_arena = TableArena()
mujoco_arena.set_origin([0.8, 0, 0])
world.merge(mujoco_arena)

from robosuite.models.objects import BallObject
from robosuite.utils.mjcf_utils import new_joint

sphere = BallObject(
    name="sphere",
    size=[0.04],
    rgba=[0, 0.5, 0.5, 1]).get_obj()
sphere.set('pos', '1.0 0 1.0')
world.worldbody.append(sphere)
world.save_model("./demo/scene_gen/xml/world_final.xml")

import mujoco
import mujoco.viewer
import time
# model = world.get_model(mode="mujoco")
model = mujoco.MjModel.from_xml_path('./demo/scene_gen/xml/world_final.xml')

data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(model, data)

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

# # while data.time < 1:
# #     mujoco.mj_step(model, data)
# # 渲染
