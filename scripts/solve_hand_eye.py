import numpy as np
import cv2

# Load data
R_gripper2base = []
t_gripper2base = []
R_target2cam = []
t_target2cam = []

robot_poses = np.load("data/robot_poses.npy", allow_pickle=True)
camera_poses = np.load("data/camera_poses.npy", allow_pickle=True)

# Compute relative motions
for i in range(len(robot_poses) - 1):
    A = np.linalg.inv(robot_poses[i]) @ robot_poses[i + 1]
    B = camera_poses[i] @ np.linalg.inv(camera_poses[i + 1])

    R_gripper2base.append(A[:3, :3])
    t_gripper2base.append(A[:3, 3])
    R_target2cam.append(B[:3, :3])
    t_target2cam.append(B[:3, 3])

# Solve hand-eye
retval, R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,
    R_target2cam, t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI
)

print("🎯 Camera→EndEffector Transform:")
print("Rotation:\n", R_cam2gripper)
print("Translation:\n", t_cam2gripper)
np.save("data/T_cam_to_hand.npy", np.hstack([R_cam2gripper, t_cam2gripper.reshape(3,1)]))
