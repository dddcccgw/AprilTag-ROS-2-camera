# MySubmodule - ROS 2 Custom Component

這是一個 **ROS 2 Package**，為 AprilTag-ROS-2-camera 項目提供自訂功能的模板。

## 🎯 功能特性

✅ **ROS 2 Humble/Jazzy** 完全相容  
✅ **Component-based 架構** - 支援 ROS 2 composition  
✅ **訂閱相機影像** - 接收 `sensor_msgs/Image`  
✅ **發佈處理結果** - 輸出 `std_msgs/String`  
✅ **YAML 參數配置** - 動態調整設定  
✅ **Launch 檔案支援** - 快速啟動  
✅ **定時回調** - 100ms 循環處理  

---

## 📋 前置需求

- **ROS 2** (Humble 或更新版本)
- **colcon** 構建工具
- **ament_cmake** 相關依賴

安裝基本 ROS 2 工具：
```bash
sudo apt-get install ros-humble-ros-core ros-humble-ros-base
```

---

## 🔨 構建與安裝

### 1. 構建 Package

從工作區根目錄執行：

```bash
cd ~/AprilTag-ROS-2-camera
colcon build --packages-select my_submodule
```

### 2. Source 環境

```bash
source install/setup.bash
```

### 3. 驗證安裝

```bash
ros2 pkg list | grep my_submodule
```

---

## ▶️ 運行

### 方式 1：使用 Launch 檔案（推薦）

```bash
ros2 launch my_submodule my_submodule.launch.py
```

### 方式 2：直接執行 Node

```bash
ros2 run my_submodule my_submodule_node
```

### 方式 3：在 Composition 中載入

```bash
ros2 component load /ComponentManager my_submodule MySubmodule
```

---

## ⚙️ 配置

編輯 `config/my_submodule.yaml` 來自訂參數：

```yaml
/**:
  ros__parameters:
    # 調試模式
    debug: false
    
    # Node 名稱
    node_name: "my_submodule"
    
    # 定時器週期（毫秒）
    timer_period_ms: 100
    
    # 輸入/輸出主題
    input_topic: "image_raw"
    output_topic: "output"
```

### 參數說明

| 參數 | 類型 | 預設值 | 說明 |
|------|------|--------|------|
| `debug` | bool | `false` | 啟用調試日誌 |
| `node_name` | string | `my_submodule` | Node 識別名稱 |
| `timer_period_ms` | int | `100` | 定時器週期（毫秒） |
| `input_topic` | string | `image_raw` | 輸入影像主題 |
| `output_topic` | string | `output` | 輸出結果主題 |

---

## 📡 ROS 主題

### 訂閱的主題

| 主題 | 類型 | 說明 |
|------|------|------|
| `image_raw` | `sensor_msgs/Image` | 輸入相機影像 |

### 發佈的主題

| 主題 | 類型 | 說明 |
|------|------|------|
| `output` | `std_msgs/String` | 處理結果字串 |

---

## 📁 項目結構

```
my_submodule/
├── CMakeLists.txt                # CMake 構建配置
├── package.xml                   # ROS 2 Package 描述
├── README.md                     # 本檔案（使用說明）
├── include/my_submodule/
│   └── my_submodule.hpp         # 頭文件（宣告）
├── src/
│   └── my_submodule.cpp         # 實現文件
├── launch/
│   └── my_submodule.launch.py   # Launch 配置檔案
└── config/
    └── my_submodule.yaml        # YAML 參數配置
```

---

## 🔧 開發指南

### 修改功能

編輯 `src/my_submodule.cpp` 實現你的邏輯：

1. **修改定時器回調**
   ```cpp
   void MySubmodule::timer_callback()
   {
     // 你的定時處理邏輯
   }
   ```

2. **修改影像訂閱回調**
   ```cpp
   void MySubmodule::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
   {
     // 處理輸入影像
   }
   ```

3. **添加新的發佈者/訂閱者**
   - 在 `.hpp` 檔案中宣告成員
   - 在建構子中初始化
   - 編譯並重新構建

### 修改主題名稱

編輯 `launch/my_submodule.launch.py` 中的 `remappings`：

```python
remappings=[
    ('image_raw', 'your_camera_topic'),
    ('output', 'your_output_topic'),
]
```

---

## 🐛 除錯

### 啟用詳細日誌

```bash
# 設定日誌級別為 DEBUG
ros2 run my_submodule my_submodule_node --log-level debug
```

### 監聽輸出主題

```bash
ros2 topic echo /my_submodule/output
```

### 檢查 Node 狀態

```bash
ros2 node info /my_submodule_node
```

---

## 📚 相關資源

- [ROS 2 官方文檔](https://docs.ros.org/en/humble/)
- [ROS 2 Component 指南](https://docs.ros.org/en/humble/Concepts/Advanced/Composition.html)
- [AprilTag-ROS-2 源專案](https://github.com/Tinker-Twins/AprilTag-ROS-2)

---

## 📝 License

**BSD License**

---

## 📞 支援

如有問題或建議，請提出 Issue 或 Pull Request。

---

**最後更新**: 2025-11-12
