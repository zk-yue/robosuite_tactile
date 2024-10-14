import numpy as np
import robosuite as suite
import cv2

from robosuite.devices import Keyboard
from robosuite.utils.input_utils import input2action

from FOTS.tactile_render import get_simapproach

if __name__ == "__main__":
    # create environment instance
    env = suite.make(
        env_name="Lift", # try with other tasks like "Stack" and "Door"
        robots="UR5e",  # try with other robots like "Sawyer" and "Jaco"
        gripper_types="Robotiq85GripperTactile",  # try with other grippers like "85" and "140"
        controller_configs=suite.load_controller_config(default_controller="OSC_POSE"),  # try with other controllers like "JOINT_POSITION" and "OSC_POSITION"
        has_renderer=True,
        render_camera="frontview",
        # camera_names = 'robot0_eye_in_hand',
        camera_names = 'gripper0_digit1_camera',
        camera_heights = 320,
        camera_widths = 240,
        camera_depths=True,
    )

    env.reset()
    env.viewer.set_camera(camera_id=0)
    # Get action limits
    low, high = env.action_spec

    device = Keyboard(pos_sensitivity=1.0, rot_sensitivity=1.0)
    device.start_control()

    # for i in range(1000):
    while True:
        active_robot = env.robots[0]
        # Get the newest action
        action, grasp = input2action(
            device=device, robot=active_robot, active_arm="right", env_configuration="Lift"
        )
        print(action)

        # action = np.random.randn(env.robots.dof) # sample random action
        # action = np.random.uniform(low, high)
        obs, reward, done, info = env.step(action)  # take action in the environment

        # np.save("./FOTS/utils/utils_data/ini_depth_extent.npy", obs['gripper0_digit1_camera_depth'].reshape(320, 240))
        # tactile
        ini_depth = np.load("./FOTS/utils/utils_data/ini_depth_extent.npy")

        ini_depth_normalized = cv2.normalize(ini_depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # depth_frame_normalized = cv2.normalize(obs['robotws0_eye_in_hand_depth'], None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        ini_depth_color = cv2.applyColorMap(ini_depth_normalized, cv2.COLORMAP_JET)
        cv2.imshow('ini_depth', cv2.normalize(ini_depth_color, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U))
        depth = obs['gripper0_digit1_camera_depth'].reshape(320, 240)
        depth_diff = ini_depth - depth
        mask = depth_diff > 0.0
        # render depth to tactile img
        simulation = get_simapproach()
        tact_img = simulation.generate(depth)
        cv2.imshow('tact_img', tact_img)
        depth_frame_normalized = cv2.normalize(obs['gripper0_digit1_camera_depth'], None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # depth_frame_normalized = cv2.normalize(obs['robot0_eye_in_hand_depth'], None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_frame_color = cv2.applyColorMap(depth_frame_normalized, cv2.COLORMAP_JET)
        combined_frame = np.hstack((obs['gripper0_digit1_camera_image'], depth_frame_color))
        # combined_frame = np.hstack((obs['robot0_eye_in_hand_image'], depth_frame_color))
        cv2.imshow('RGB and Depth', combined_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        env.render()  # render on display
    cv2.destroyAllWindows()