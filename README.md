 
# 🧭 AprilTag Pose Estimation using Intel RealSense D435

This project provides a **standalone Python implementation** for **AprilTag detection and 6DoF pose estimation** using an **Intel RealSense D435** RGB-D camera.

It is **based on and inspired by** the original [Tinker-Twins/AprilTag-ROS-2](https://github.com/Tinker-Twins/AprilTag-ROS-2) repository — a ROS 2-based AprilTag detection package.  
This fork adapts their system into a **non-ROS**, lightweight Python tool for direct use in vision or robotics projects.

---

## 🚀 My Contributions

✅ Converted ROS 2 node into a **pure Python script**  
✅ Added **Intel RealSense D435** integration via `pyrealsense2`  
✅ Real-time **pose visualization** (3D axes + text overlay)  
✅ Displays **depth, translation, and Euler rotation**  
✅ Minimal dependencies — runs anywhere, no ROS setup required  

---

## 📦 Requirements

### 🐍 Python 3.8 or newer
Install dependencies:

```bash
pip install opencv-python pyrealsense2 numpy dt-apriltags scipy
````

> 💡 On Windows, install [Intel RealSense SDK 2.0](https://www.intelrealsense.com/sdk-2/) before running.

---

## ⚙️ Configuration

You can change parameters directly in `my_camera_apriltag.py`:

| Parameter                     | Description                        | Default      |
| ----------------------------- | ---------------------------------- | ------------ |
| `TAG_SIZE`                    | Physical size of AprilTag (meters) | `0.1`        |
| `FRAME_WIDTH`, `FRAME_HEIGHT` | Image resolution                   | `640x480`    |
| `FPS`                         | Frame rate                         | `30`         |
| `families`                    | AprilTag family                    | `"tag36h11"` |

---

## ▶️ Running the Script

1. Connect your **Intel RealSense D435** camera.
2. From your project directory, run:

```bash
python3 my_camera_apriltag.py
```

### 🧾 Example Terminal Output

```
david@hrclab-System-Product-Name:~/AprilTag-ROS-2-camera/scripts$ python3 my_camera_apriltag.py
📷 Camera Intrinsics:
   fx: 603.59423828125, fy: 603.0836791992188, cx: 329.98138427734375, cy: 246.50851440429688
 AprilTag detector initialized! Press Q to quit
```

When the window opens, you’ll see live camera video with AprilTags outlined and annotated.

Press **Q** to exit.

---

## 🧩 Example On-Screen Output

```
Tag ID: 2
Pos (m): X=0.03 Y=-0.02 Z=0.45
Rot (deg): R=1.2 P=-3.5 Y=88.7
Depth: 0.448m
```

Axes drawn on each tag:

* **Red** → X-axis
* **Green** → Y-axis
* **Blue** → Z-axis

---

## 🧪 Tested Setup

| Component       | Version |
| --------------- | ------- |
| Intel RealSense | D435    |
| RealSense SDK   | 2.55+   |
| Python          | 3.10    |
| OpenCV          | 4.10+   |
| dt-apriltags    | 1.0.4+  |

---

## 🙏 Credits

* Original AprilTag ROS 2 framework: [Tinker-Twins/AprilTag-ROS-2](https://github.com/Tinker-Twins/AprilTag-ROS-2)
* Adapted and extended for standalone RealSense use by **dddcccgw**

---

## 📄 License

Released under the **MIT License**.
Please also respect the license of the original [Tinker-Twins/AprilTag-ROS-2](https://github.com/Tinker-Twins/AprilTag-ROS-2) project.

---

## 👨‍💻 Author

**Your Name**
📧 [[gwchen24@gmail.com](mailto:your.email@example.com)]
💼 [https://github.com/dddcccgw]

---

⭐ If this project helps you, please star both:

* [Tinker-Twins/AprilTag-ROS-2](https://github.com/Tinker-Twins/AprilTag-ROS-2)
* [This repository](https://github.com/dddcccgw/AprilTag-ROS-2-camera/tree/feature/my_submodule)

