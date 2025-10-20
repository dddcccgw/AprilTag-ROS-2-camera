import numpy as np
from collections import deque
import threading
from scipy.spatial.transform import Rotation

class MapFrameBuilder:
    def __init__(self, tag_size=0.1, tag_spacing=0.2, window_size=30):
        # Define the expected tag IDs and their positions in the map frame
        self.MAP_TAGS = {
            1: {'pos': np.array([0.0, 0.0, 0.0]),        # Origin tag
                'rot': np.eye(3)},                        # Identity rotation
            2: {'pos': np.array([tag_spacing, 0.0, 0.0]), # Tag along X-axis
                'rot': np.eye(3)},
            3: {'pos': np.array([0.0, tag_spacing, 0.0]), # Tag along Y-axis
                'rot': np.eye(3)}
        }
        
        self.tag_poses = {tag_id: deque(maxlen=window_size) for tag_id in self.MAP_TAGS.keys()}
        self.map_frame = None
        self.lock = threading.Lock()
        
    def update_tag_pose(self, tag_id, R, t):
        """Update the pose of a tag in the map frame"""
        if tag_id in self.tag_poses:
            with self.lock:
                self.tag_poses[tag_id].append((R, t))
    
    def compute_map_frame(self):
        """Compute the map frame from multiple tag observations"""
        with self.lock:
            # Check if we have enough observations
            valid_tags = [tag_id for tag_id, poses in self.tag_poses.items() if len(poses) > 0]
            if len(valid_tags) < 2:  # Need at least 2 tags for reliable frame
                return None, None
            
            # Use tag 1 as reference if available, otherwise use the first valid tag
            ref_id = 1 if 1 in valid_tags else valid_tags[0]
            ref_R, ref_t = self.tag_poses[ref_id][-1]  # Use most recent observation
            
            # Transform from camera frame to map frame
            R_cam_to_map = ref_R.T  # Inverse rotation
            t_cam_to_map = -R_cam_to_map @ ref_t
            
            return R_cam_to_map, t_cam_to_map
    
    def get_camera_pose(self):
        """Get the camera pose in the map frame"""
        R_cam_to_map, t_cam_to_map = self.compute_map_frame()
        if R_cam_to_map is None:
            return None, None
            
        # Convert to euler angles for display
        r = Rotation.from_matrix(R_cam_to_map)
        euler = r.as_euler('xyz', degrees=True)
        
        return euler, t_cam_to_map
    
    def get_valid_tag_count(self):
        """Get the number of valid tags currently being tracked"""
        return sum(1 for poses in self.tag_poses.values() if len(poses) > 0)
