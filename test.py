import numpy as np
import robosuite as suite
import cv2

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
        camera_depths=True,
    )

    env.reset()
    env.viewer.set_camera(camera_id=0)
    # Get action limits
    low, high = env.action_spec

    for i in range(1000):
        # action = np.random.randn(env.robots.dof) # sample random action
        action = np.random.uniform(low, high)
        obs, reward, done, info = env.step(action)  # take action in the environment
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