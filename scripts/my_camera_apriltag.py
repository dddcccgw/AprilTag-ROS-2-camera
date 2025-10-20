import cv2
import sys
from argparse import ArgumentParser
import numpy as np
from dt_apriltags import Detector
import pyrealsense2 as rs

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Get color camera intrinsic parameters
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

# RealSense D435 camera intrinsics
camera_params = [intr.fx, intr.fy, intr.ppx, intr.ppy]
camera_matrix = np.array([
    [camera_params[0], 0, camera_params[2]],
    [0, camera_params[1], camera_params[3]],
    [0, 0, 1]
])

# Tag 的實際大小（單位：米）
tag_size = 0.1  # 10cm
# ------------------------------

# Print camera parameters
print(f"📷 Camera Intrinsics:")
print(f"   fx: {camera_params[0]}, fy: {camera_params[1]}")
print(f"   cx: {camera_params[2]}, cy: {camera_params[3]}")

# Initialize AprilTag detector
detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

print("✅ AprilTag 偵測器啟動成功！按 Q 鍵離開")

while True:
    try:
        # Wait for a coherent color frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            print("❌ No color frame received")
            continue

        # Convert images to numpy arrays
        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    except Exception as e:
        print(f"❌ Error: {e}")
        break
    
    # 偵測 AprilTag（需要傳入 camera_params 和 tag_size 才能計算位姿）
    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=tag_size
    )

    for det in detections:
        # 繪製偵測框
        corners = det.corners.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)

        # 標記 ID
        tag_id = det.tag_id
        center = tuple(det.center.astype(int))
        cv2.putText(frame, f"ID: {tag_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 顯示位姿資訊
        pose_R = det.pose_R
        pose_t = det.pose_t
        if pose_R is not None and pose_t is not None:
            # 轉換為歐拉角（更容易理解）
            from scipy.spatial.transform import Rotation
            r = Rotation.from_matrix(pose_R)
            euler = r.as_euler('xyz', degrees=True)
            
            print(f"\n📍 Tag {tag_id}")
            print(f"   Position (x,y,z): {pose_t.flatten()}")
            print(f"   Rotation (deg): Roll={euler[0]:.1f}, Pitch={euler[1]:.1f}, Yaw={euler[2]:.1f}")

    # 顯示結果
    cv2.imshow("AprilTag Detection - Press Q to quit", frame)

    # 按下 q 鍵離開
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop streaming
pipeline.stop()
cv2.destroyAllWindows()