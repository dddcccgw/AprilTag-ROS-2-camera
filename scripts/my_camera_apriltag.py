import cv2
import numpy as np
import pyrealsense2 as rs
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation
import time
import traceback
import json

# ====== Configuration ======
TAG_SIZE = 0.0625  # meters (6.25 cm)
FPS = 30
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
TARGET_TAG_IDS = [0, 1, 2]  # 三個目標標籤
MAP_ORIGIN_TAG_ID = 0  # Tag 0 作為 Map 座標系原點

# ====== Initialize RealSense ======
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)

profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# Get camera intrinsics
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
camera_params = [intr.fx, intr.fy, intr.ppx, intr.ppy]
camera_matrix = np.array([[intr.fx, 0, intr.ppx],
                          [0, intr.fy, intr.ppy],
                          [0, 0, 1]])
dist_coeffs = np.zeros((5, 1))

print(f"📷 Camera Intrinsics:\n   fx: {intr.fx}, fy: {intr.fy}, cx: {intr.ppx}, cy: {intr.ppy}")

# ====== Initialize AprilTag Detector ======
detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=3.0,      # 降低解析度提升穩定性
    quad_sigma=1.0,         # 增加模糊減少誤判
    refine_edges=1,         # 邊緣細化
    decode_sharpening=0.1   # 降低銳化
)
print("✅ AprilTag detector initialized! Press Q to quit")
print(f"🗺️  Map Origin: Tag {MAP_ORIGIN_TAG_ID}")

# ====== Helper Functions ======
def draw_pose_axes(frame, rvec, tvec, camera_matrix, dist_coeffs, length=0.05):
    """繪製 3D 座標軸"""
    axis_points = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
    imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    imgpts = imgpts.astype(int)
    origin = tuple(imgpts[0].ravel())
    frame = cv2.line(frame, origin, tuple(imgpts[1].ravel()), (0,0,255), 3)  # X軸 紅色
    frame = cv2.line(frame, origin, tuple(imgpts[2].ravel()), (0,255,0), 3)  # Y軸 綠色
    frame = cv2.line(frame, origin, tuple(imgpts[3].ravel()), (255,0,0), 3)  # Z軸 藍色
    return frame

def transform_to_map_frame(tag_pose_R, tag_pose_t, map_origin_R, map_origin_t):
    """
    將 Tag 姿態從相機座標系轉換到 Map 座標系
    
    參數:
        tag_pose_R: 目標 Tag 的旋轉矩陣 (相對相機)
        tag_pose_t: 目標 Tag 的位移向量 (相對相機)
        map_origin_R: Map 原點 Tag 的旋轉矩陣 (相對相機)
        map_origin_t: Map 原點 Tag 的位移向量 (相對相機)
    
    返回:
        (R_map, t_map): 目標 Tag 相對於 Map 原點的姿態
    """
    # 計算 Map 原點相對於相機的變換矩陣
    T_cam_to_map = np.eye(4)
    T_cam_to_map[:3, :3] = map_origin_R
    T_cam_to_map[:3, 3] = map_origin_t.flatten()
    
    # 計算逆變換（Map 原點 → 相機）
    T_map_to_cam = np.linalg.inv(T_cam_to_map)
    
    # 計算目標 Tag 相對於相機的變換矩陣
    T_cam_to_tag = np.eye(4)
    T_cam_to_tag[:3, :3] = tag_pose_R
    T_cam_to_tag[:3, 3] = tag_pose_t.flatten()
    
    # 計算目標 Tag 相對於 Map 的變換矩陣
    T_map_to_tag = T_map_to_cam @ T_cam_to_tag
    
    # 提取旋轉和位移
    R_map = T_map_to_tag[:3, :3]
    t_map = T_map_to_tag[:3, 3].reshape(3, 1)
    
    return R_map, t_map

def draw_map_info(frame, tag_data, map_origin_id):
    """在畫面上繪製 Map 資訊"""
    y_offset = 30
    
    # 標題
    cv2.putText(frame, f"=== Map Frame (Origin: Tag {map_origin_id}) ===", 
                (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
    y_offset += 30
    
    # 顯示每個 Tag 在 Map 中的位置
    for tag_id in sorted(tag_data.keys()):
        data = tag_data[tag_id]
        
        if tag_id == map_origin_id:
            text = f"Tag {tag_id}: [ORIGIN] X=0.00 Y=0.00 Z=0.00"
            color = (0, 255, 0)  # 綠色
        else:
            pos = data['map_position']
            text = f"Tag {tag_id}: X={pos[0]:.3f} Y={pos[1]:.3f} Z={pos[2]:.3f}m"
            color = (255, 255, 255)
        
        cv2.putText(frame, text, (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 3)
        cv2.putText(frame, text, (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        y_offset += 25
    
    return frame

# ====== 儲存 Tag 資料的字典 ======
tag_data_storage = {}

# ====== Main Loop ======
try:
    frame_count = 0
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 影像預處理（改善偵測穩定性）
        gray = cv2.equalizeHist(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        # AprilTag Detection with error handling
        try:
            detections = detector.detect(gray, estimate_tag_pose=True,
                                         camera_params=camera_params, tag_size=TAG_SIZE)
        except Exception as e:
            if frame_count % 30 == 0:  # 每秒顯示一次錯誤
                print(f"⚠️ Detection error: {e}")
            detections = []
            cv2.putText(frame, "Detection Error!", (10, FRAME_HEIGHT - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # 暫存當前幀的 Tag 資料
        current_frame_tags = {}
        map_origin_data = None

        # 第一輪：收集所有 Tag 資料（相對相機）
        for det in detections:
            if det.tag_id not in TARGET_TAG_IDS:
                continue
            
            # 繪製標籤邊框
            corners = det.corners.astype(int)
            color = (0, 255, 0) if det.tag_id == MAP_ORIGIN_TAG_ID else (255, 0, 0)
            cv2.polylines(frame, [corners.reshape((-1,1,2))], True, color, 2)
            
            # 繪製 Tag ID
            center = tuple(det.center.astype(int))
            cv2.putText(frame, f"ID:{det.tag_id}", (center[0]-20, center[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
            
            # 獲取深度
            depth = depth_frame.get_distance(int(center[0]), int(center[1]))
            
            # 檢查姿態有效性
            pose_R, pose_t = det.pose_R, det.pose_t
            if pose_R is not None and pose_t is not None:
                # 繪製相機座標系的座標軸
                rvec, _ = cv2.Rodrigues(pose_R)
                frame = draw_pose_axes(frame, rvec, pose_t, camera_matrix, dist_coeffs)
                
                # 儲存 Tag 資料
                current_frame_tags[det.tag_id] = {
                    'pose_R': pose_R,
                    'pose_t': pose_t,
                    'depth': depth,
                    'center': center
                }
                
                # 記錄 Map 原點
                if det.tag_id == MAP_ORIGIN_TAG_ID:
                    map_origin_data = current_frame_tags[det.tag_id]

        # 第二輪：如果找到 Map 原點，計算其他 Tag 在 Map 中的位置
        if map_origin_data is not None:
            map_origin_R = map_origin_data['pose_R']
            map_origin_t = map_origin_data['pose_t']
            
            for tag_id, data in current_frame_tags.items():
                if tag_id == MAP_ORIGIN_TAG_ID:
                    # 原點在 Map 中的位置就是 (0, 0, 0)
                    data['map_position'] = np.array([0.0, 0.0, 0.0])
                    data['map_rotation'] = np.eye(3)
                else:
                    # 轉換到 Map 座標系
                    R_map, t_map = transform_to_map_frame(
                        data['pose_R'], data['pose_t'],
                        map_origin_R, map_origin_t
                    )
                    data['map_position'] = t_map.flatten()
                    data['map_rotation'] = R_map
                
                # 更新儲存
                tag_data_storage[tag_id] = data
            
            # 繪製 Map 資訊
            frame = draw_map_info(frame, tag_data_storage, MAP_ORIGIN_TAG_ID)
            
            # 終端輸出（每 30 幀輸出一次）
            if frame_count % 30 == 0:
                print(f"\n{'='*60}")
                print(f"🗺️  Map Frame Status (Origin: Tag {MAP_ORIGIN_TAG_ID})")
                print(f"{'='*60}")
                for tag_id in sorted(tag_data_storage.keys()):
                    data = tag_data_storage[tag_id]
                    pos = data['map_position']
                    euler = Rotation.from_matrix(data['map_rotation']).as_euler('xyz', degrees=True)
                    
                    if tag_id == MAP_ORIGIN_TAG_ID:
                        print(f"Tag {tag_id} [ORIGIN]: (0.000, 0.000, 0.000) m")
                    else:
                        print(f"Tag {tag_id}: ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) m")
                        print(f"         Rotation: R={euler[0]:6.1f}° P={euler[1]:6.1f}° Y={euler[2]:6.1f}°")
        else:
            # 未找到 Map 原點
            cv2.putText(frame, f"Searching for Map Origin (Tag {MAP_ORIGIN_TAG_ID})...", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,165,255), 2)

        # 顯示偵測到的 Tag 數量
        cv2.putText(frame, f"Detected: {len(current_frame_tags)}/{len(TARGET_TAG_IDS)} tags", 
                    (10, FRAME_HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

        # Display Results
        cv2.imshow("AprilTag Map Frame - Press Q to quit", frame)

        # Break on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        frame_count += 1

except Exception as e:
    print("❌ Error:")
    traceback.print_exc()

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("\n🛑 Program terminated")
    
    # 最終輸出並儲存 Map 資料
    if tag_data_storage:
        print(f"\n{'='*60}")
        print("📊 Final Map Data")
        print(f"{'='*60}")
        
        # 準備儲存的資料
        map_data = {}
        
        for tag_id in sorted(tag_data_storage.keys()):
            data = tag_data_storage[tag_id]
            pos = data['map_position']
            rot = data['map_rotation']
            euler = Rotation.from_matrix(rot).as_euler('xyz', degrees=True)
            
            print(f"Tag {tag_id}: Position = [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m")
            print(f"         Rotation = [R={euler[0]:.2f}°, P={euler[1]:.2f}°, Y={euler[2]:.2f}°]")
            
            # 儲存到字典
            map_data[str(tag_id)] = {
                'position': pos.tolist(),
                'rotation_matrix': rot.tolist(),
                'rotation_euler': euler.tolist()
            }
        
        # 儲存為 JSON 檔案
        try:
            with open('apriltag_map.json', 'w') as f:
                json.dump(map_data, f, indent=2)
            print("\n💾 Map data saved to: apriltag_map.json")
        except Exception as e:
            print(f"\n⚠️ Failed to save map data: {e}")