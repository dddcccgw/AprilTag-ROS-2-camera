# 🧭 AprilTag Pose Estimation using Intel RealSense D435

This repository provides a **standalone Python implementation** for real-time **AprilTag detection and 6DoF pose estimation** using an **Intel RealSense D435** RGB-D camera.

It is **based on and inspired by** the original [Tinker-Twins/AprilTag-ROS-2](https://github.com/Tinker-Twins/AprilTag-ROS-2) project — a ROS 2-based AprilTag detection package.  
This fork adapts their work into a **non-ROS**, lightweight Python pipeline for easier use in computer vision or robotics projects.

---

## 🚀 Key Contributions

### 🔧 What’s New in This Version
- ✅ **Standalone Python script** (no ROS2 dependency)  
- ✅ Direct **Intel RealSense D435** integration using `pyrealsense2`  
- ✅ Real-time **pose visualization** with 3D axes and text overlays  
- ✅ Displays **depth, translation, and Euler rotation**  
- ✅ Simplified structure for research and prototyping use

### 🔗 Credit
Original AprilTag detection logic and structure inspired by:  
👉 [Tinker-Twins/AprilTag-ROS-2](https://github.com/Tinker-Twins/AprilTag-ROS-2)

All credit to the Tinker-Twins team for their foundational ROS2 implementation.

---

## 📦 Requirements

### Python 3.8+
Install dependencies via pip:

```bash
pip install opencv-python pyrealsense2 numpy dt-apriltags scipy
💡 For Intel RealSense cameras, make sure Intel RealSense SDK 2.0 is installed:
https://www.intelrealsense.com/sdk-2/

⚙️ Configuration
Parameter	Description	Default
TAG_SIZE	AprilTag physical size (meters)	0.1
FRAME_WIDTH, FRAME_HEIGHT	Frame resolution	640x480
FPS	Frame rate	30
families	AprilTag family	"tag36h11"

▶️ Usage
Connect your Intel RealSense D435.

Run the script:

bash
Copy code
python apriltag_realsense_pose.py
View the live feed with detected AprilTags and pose information.

Press Q to quit.

🧩 Example Output
yaml
Copy code
Tag ID: 2
Pos (m): X=0.03 Y=-0.02 Z=0.45
Rot (deg): R=1.2 P=-3.5 Y=88.7
Depth: 0.448m
Axes:

Red → X-axis

Green → Y-axis

Blue → Z-axis

🧪 Tested Setup
Component	Version
Intel RealSense	D435
RealSense SDK	2.55+
Python	3.10
OpenCV	4.10+
dt-apriltags	1.0.4+

📄 License
This project is released under the MIT License.
Please also refer to the license in Tinker-Twins/AprilTag-ROS-2 for their original work.
