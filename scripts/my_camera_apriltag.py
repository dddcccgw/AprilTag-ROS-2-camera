import cv2
import sys
from argparse import ArgumentParser
import numpy as np
from dt_apriltags import Detector
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation
import threading
from collections import deque
import time

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream

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

# Distortion coefficients (assuming no distortion for RealSense)
dist_coeffs = np.zeros((5, 1))

# Tag parameters
tag_size = 0.1  # Tag size in meters (10cm)
tag_spacing = 0.2  # Spacing between tags in meters (if using multiple tags)

def draw_pose_axes(frame, rvec, tvec, camera_matrix, dist_coeffs, length=0.1):
    """Draw 3D axes on the detected tag for pose visualization"""
    axis_points = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
    imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    imgpts = imgpts.astype(int)
    
    # Draw axes
    origin = tuple(imgpts[0].ravel())
    frame = cv2.line(frame, origin, tuple(imgpts[1].ravel()), (0,0,255), 3)  # X-axis (red)
    frame = cv2.line(frame, origin, tuple(imgpts[2].ravel()), (0,255,0), 3)  # Y-axis (green)
    frame = cv2.line(frame, origin, tuple(imgpts[3].ravel()), (255,0,0), 3)  # Z-axis (blue)
    
    return frame

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

print("✅ AprilTag detector initialized! Press Q to quit")

# Create alignment object to align depth frames to color frame
align = rs.align(rs.stream.color)

while True:
    try:
        # Wait for coherent frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            print("❌ No frames received")
            continue

        # Convert images to numpy arrays
        frame = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size
        )

        for det in detections:
            # Draw detection boundaries
            corners = det.corners.astype(int)
            cv2.polylines(frame, [corners.reshape((-1,1,2))], True, (0,255,0), 2)
            
            # Get tag information
            tag_id = det.tag_id
            center = tuple(det.center.astype(int))
            
            # Get depth at tag center
            depth = depth_frame.get_distance(int(center[0]), int(center[1]))
            
            # Get pose information
            pose_R = det.pose_R
            pose_t = det.pose_t
            
            if pose_R is not None and pose_t is not None:
                # Convert rotation matrix to rotation vector
                rvec, _ = cv2.Rodrigues(pose_R)
                tvec = pose_t
                
                # Draw 3D axes
                frame = draw_pose_axes(frame, rvec, tvec, camera_matrix, dist_coeffs)
                
                # Convert to Euler angles for display
                from scipy.spatial.transform import Rotation
                r = Rotation.from_matrix(pose_R)
                euler = r.as_euler('xyz', degrees=True)
                
                # Display pose information
                pos_text = f"Pos (m): X={pose_t[0,0]:.2f} Y={pose_t[1,0]:.2f} Z={pose_t[2,0]:.2f}"
                rot_text = f"Rot (deg): R={euler[0]:.1f} P={euler[1]:.1f} Y={euler[2]:.1f}"
                depth_text = f"Depth: {depth:.3f}m"
                
                # Draw text with background for better visibility
                y_offset = 60
                for i, text in enumerate([f"Tag ID: {tag_id}", pos_text, rot_text, depth_text]):
                    pos = (10, y_offset + i*30)
                    cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3)
                    cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 1)

        # Display Results
        cv2.imshow("AprilTag Pose Estimation - Press Q to quit", frame)

        # Break on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"❌ Error: {e}")
        break

# Stop streaming
pipeline.stop()
cv2.destroyAllWindows()

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

# Distortion coefficients (assuming no distortion for RealSense)
dist_coeffs = np.zeros((5, 1))

# Tag parameters
tag_size = 0.1  # Tag size in meters (10cm)
tag_spacing = 0.2  # Spacing between tags in meters (if using multiple tags)

def draw_pose_axes(frame, rvec, tvec, camera_matrix, dist_coeffs, length=0.1):
    """Draw 3D axes on the detected tag for pose visualization"""
    axis_points = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
    imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    imgpts = imgpts.astype(int)
    
    # Draw axes
    origin = tuple(imgpts[0].ravel())
    frame = cv2.line(frame, origin, tuple(imgpts[1].ravel()), (0,0,255), 3)  # X-axis (red)
    frame = cv2.line(frame, origin, tuple(imgpts[2].ravel()), (0,255,0), 3)  # Y-axis (green)
    frame = cv2.line(frame, origin, tuple(imgpts[3].ravel()), (255,0,0), 3)  # Z-axis (blue)
    
    return frame

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

print("✅ AprilTag detector initialized! Press Q to quit")

# Create alignment object to align depth frames to color frame
align = rs.align(rs.stream.color)

while True:
    try:
        # Wait for coherent frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            print("❌ No frames received")
            continue

        # Convert images to numpy arrays
        frame = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size
        )

        for det in detections:
            # Draw detection boundaries
            corners = det.corners.astype(int)
            cv2.polylines(frame, [corners.reshape((-1,1,2))], True, (0,255,0), 2)
            
            # Get tag information
            tag_id = det.tag_id
            center = tuple(det.center.astype(int))
            
            # Get depth at tag center
            depth = depth_frame.get_distance(int(center[0]), int(center[1]))
            
            # Get pose information
            pose_R = det.pose_R
            pose_t = det.pose_t
            
            if pose_R is not None and pose_t is not None:
                # Convert rotation matrix to rotation vector
                rvec, _ = cv2.Rodrigues(pose_R)
                tvec = pose_t
                
                # Draw 3D axes
                frame = draw_pose_axes(frame, rvec, tvec, camera_matrix, dist_coeffs)
                
                # Convert to Euler angles for display
                r = Rotation.from_matrix(pose_R)
                euler = r.as_euler('xyz', degrees=True)
                
                # Display pose information
                pos_text = f"Pos (m): X={pose_t[0,0]:.2f} Y={pose_t[1,0]:.2f} Z={pose_t[2,0]:.2f}"
                rot_text = f"Rot (deg): R={euler[0]:.1f} P={euler[1]:.1f} Y={euler[2]:.1f}"
                depth_text = f"Depth: {depth:.3f}m"
                
                # Draw text with background for better visibility
                y_offset = 60
                for i, text in enumerate([f"Tag ID: {tag_id}", pos_text, rot_text, depth_text]):
                    pos = (10, y_offset + i*30)
                    cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3)
                    cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 1)

        # Display Results
        cv2.imshow("AprilTag Pose Estimation - Press Q to quit", frame)

        # Break on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"❌ Error: {e}")
        break

# Stop streaming
pipeline.stop()
cv2.destroyAllWindows()