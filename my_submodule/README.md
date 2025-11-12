# MySubmodule - ROS 2 Custom Component# MySubmodule - ROS 2 Custom Component# MySubmodule - ROS 2 Custom Component



這是一個 **ROS 2 Package**，為 AprilTag-ROS-2-camera 項目提供自訂功能的模板。作為 `ros-package-node` 分支的核心組件，此 package 展示了如何在 ROS 2 生態系統中構建模塊化的影像處理節點。



## 🎯 功能特性這是一個 **ROS 2 Package**，為 AprilTag-ROS-2-camera 項目提供自訂功能的模板。作為 `feature/my_submodule` 分支的核心組件，此 package 展示了如何在 ROS 2 生態系統中構建模塊化的影像處理節點。這是一個 **ROS 2 Package**，為 AprilTag-ROS-2-camera 項目提供自訂功能的模板。作為 `feature/my_submodule` 分支的核心組件，此 package 展示了如何在 ROS 2 生態系統中構建模塊化的影像處理節點。



✅ **ROS 2 Humble/Jazzy** 完全相容  

✅ **Component-based 架構** - 支援 ROS 2 composition  

✅ **訂閱相機影像** - 接收 `sensor_msgs/Image`  ## 🎯 功能特性## 🎯 功能特性

✅ **發佈處理結果** - 輸出 `std_msgs/String`  

✅ **YAML 參數配置** - 動態調整設定  

✅ **Launch 檔案支援** - 快速啟動  

✅ **定時回調** - 100ms 循環處理  ✅ **ROS 2 Humble/Jazzy** 完全相容  ✅ **ROS 2 Humble/Jazzy** 完全相容  

✅ **與主項目集成** - 無縫整合到 AprilTag-ROS-2-camera 工作區

✅ **Component-based 架構** - 支援 ROS 2 composition  ✅ **Component-based 架構** - 支援 ROS 2 composition  

---

✅ **訂閱相機影像** - 接收 `sensor_msgs/Image`  ✅ **訂閱相機影像** - 接收 `sensor_msgs/Image`  

## 📊 分支對比：main vs ros-package-node

✅ **發佈處理結果** - 輸出 `std_msgs/String`  ✅ **發佈處理結果** - 輸出 `std_msgs/String`  

此分支擴展了主專案，添加了 ROS 2 Package 支援：

✅ **YAML 參數配置** - 動態調整設定  ✅ **YAML 參數配置** - 動態調整設定  

| 功能 | main 分支 | ros-package-node |

|------|-----------|------------------|✅ **Launch 檔案支援** - 快速啟動  ✅ **Launch 檔案支援** - 快速啟動  

| **AprilTag 檢測** | ✅ Python 純實現 | ✅ Python + ROS 2 |

| **相機驅動** | Intel RealSense D435 | Intel RealSense D435 |✅ **定時回調** - 100ms 循環處理  ✅ **定時回調** - 100ms 循環處理  

| **依賴** | 最小化 (pip) | ROS 2 完整堆棧 |

| **運行方式** | Python 直接執行 | ROS 2 node/launch |✅ **與主項目集成** - 無縫整合到 AprilTag-ROS-2-camera 工作區✅ **與主項目集成** - 無縫整合到 AprilTag-ROS-2-camera 工作區  

| **節點架構** | 單一腳本 | 模塊化 component |

| **參數配置** | 硬編碼/命令行 | YAML + ROS 參數伺服器 |

| **訂閱/發佈** | N/A | 完整 ROS 主題系統 |

| **Composition** | ❌ | ✅ 支援 |------



### 何時選擇哪個版本？



- **使用 main 分支** 如果你需要：## 📊 分支對比：main vs feature/my_submodule## � 分支對比：main vs feature/my_submodule

  - 輕量級、無依賴的獨立 Python 工具

  - 快速原型開發

  - 最小的系統開銷

此分支擴展了主專案，添加了 ROS 2 Package 支援：此分支擴展了主專案，添加了 ROS 2 Package 支援：

- **使用 ros-package-node** 如果你需要：

  - ROS 2 生態系統集成

  - 多節點協作

  - 完整的中間件功能（參數、發現、日誌等）| 功能 | main 分支 | feature/my_submodule || 功能 | main 分支 | feature/my_submodule |

  - 可擴展的模塊化架構

|------|-----------|----------------------||------|-----------|----------------------|

---

| **AprilTag 檢測** | ✅ Python 純實現 | ✅ Python + ROS 2 || **AprilTag 檢測** | ✅ Python 純實現 | ✅ Python + ROS 2 |

## 📋 前置需求

| **相機驅動** | Intel RealSense D435 | Intel RealSense D435 || **相機驅動** | Intel RealSense D435 | Intel RealSense D435 |

### ROS 2 環境

| **依賴** | 最小化 (pip) | ROS 2 完整堆棧 || **依賴** | 最小化 (pip) | ROS 2 完整堆棧 |

- **ROS 2** (Humble 或更新版本)

  ```bash| **運行方式** | Python 直接執行 | ROS 2 node/launch || **運行方式** | Python 直接執行 | ROS 2 node/launch |

  # Ubuntu 22.04 安裝 ROS 2 Humble

  sudo apt install ros-humble-desktop| **節點架構** | 單一腳本 | 模塊化 component || **節點架構** | 單一腳本 | 模塊化 component |

  ```

| **參數配置** | 硬編碼/命令行 | YAML + ROS 參數伺服器 || **參數配置** | 硬編碼/命令行 | YAML + ROS 參數伺服器 |

- **colcon** 構建工具

  ```bash| **訂閱/發佈** | N/A | 完整 ROS 主題系統 || **訂閱/發佈** | N/A | 完整 ROS 主題系統 |

  sudo apt install python3-colcon-common-extensions

  ```| **Composition** | ❌ | ✅ 支援 || **Composition** | ❌ | ✅ 支援 |



- **必要的 ROS 2 依賴**

  ```bash

  sudo apt install ros-humble-sensor-msgs ros-humble-std-msgs \### 何時選擇哪個版本？### 選擇哪個版本？

    ros-humble-rclcpp ros-humble-rclcpp-components

  ```



### Python 依賴- **使用 main 分支** 如果你需要：- **使用 main 分支** 如果你需要：



```bash  - 輕量級、無依賴的獨立 Python 工具  - 輕量級、無依賴的獨立 Python 工具

pip install opencv-python pyrealsense2 numpy dt-apriltags scipy

```  - 快速原型開發  - 快速原型開發



---  - 最小的系統開銷  - 最小的系統開銷



## 🔨 構建與安裝



### 方法 1：在工作區構建（推薦）- **使用 feature/my_submodule** 如果你需要：- **使用 feature/my_submodule** 如果你需要：



```bash  - ROS 2 生態系統集成  - ROS 2 生態系統集成

# 進入工作區根目錄

cd ~/AprilTag-ROS-2-camera  - 多節點協作  - 多節點協作



# 只構建 my_submodule  - 完整的中間件功能（參數、發現、日誌等）  - 完整的中間件功能（參數、發現、日誌等）

colcon build --packages-select my_submodule

  - 可擴展的模塊化架構  - 可擴展的模塊化架構

# 或構建所有 ROS 2 packages

colcon build



# Source 環境------

source install/setup.bash

```



### 方法 2：開發模式構建## 📋 前置需求## �📋 前置需求



適合頻繁修改源代碼的情況：



```bash### ROS 2 環境### ROS 2 環境

cd ~/AprilTag-ROS-2-camera

colcon build --packages-select my_submodule --symlink-install

source install/setup.bash

```- **ROS 2** (Humble 或更新版本)- **ROS 2** (Humble 或更新版本)



這樣修改源代碼後無需重新構建。  ```bash  ```bash



### 驗證安裝  # Ubuntu 22.04 安裝 ROS 2 Humble  # Ubuntu 22.04 安裝 ROS 2 Humble



```bash  sudo apt install ros-humble-desktop  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 列出所有 ROS 2 packages

ros2 pkg list | grep my_submodule  ```  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null



# 查看 package 信息  sudo apt update

ros2 pkg info my_submodule

- **colcon** 構建工具  sudo apt install ros-humble-desktop

# 列出可執行文件

ros2 pkg executables my_submodule  ```bash  ```

```

  sudo apt install python3-colcon-common-extensions

---

  ```- **colcon** 構建工具

## ▶️ 運行 Node

  ```bash

在運行任何命令前，請確保已 source ROS 2 環境：

- **必要的 ROS 2 依賴**  sudo apt install python3-colcon-common-extensions

```bash

source ~/AprilTag-ROS-2-camera/install/setup.bash  ```bash  ```

```

  sudo apt install ros-humble-sensor-msgs ros-humble-std-msgs \

### 方式 1：使用 Launch 檔案（推薦 ⭐）

    ros-humble-rclcpp ros-humble-rclcpp-components- **必要的 ROS 2 依賴**

這是最簡單且推薦的方式。Launch 檔案自動配置所有參數和主題映射。

  ```  ```bash

```bash

ros2 launch my_submodule my_submodule.launch.py  sudo apt install ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-rclcpp ros-humble-rclcpp-components ros-humble-apriltag-msgs

```

### Python 依賴  ```

**預期輸出**：

```

[INFO] [my_submodule_node]: MySubmodule initializing...

[INFO] [my_submodule_node]: MySubmodule initialized successfully```bash### Python 依賴

[DEBUG] [my_submodule_node]: Received image: 640x480

```pip install opencv-python pyrealsense2 numpy dt-apriltags scipy



#### 自訂 Launch 參數``````bash



傳遞自訂參數到 launch 檔案：pip install opencv-python pyrealsense2 numpy dt-apriltags scipy



```bash---```

# 設定調試模式

ros2 launch my_submodule my_submodule.launch.py debug:=true



# 修改定時器週期（毫秒）## 🔨 構建與安裝---

ros2 launch my_submodule my_submodule.launch.py timer_period_ms:=200

```



### 方式 2：直接執行 Node### 方法 1：在工作區構建（推薦）## 🔨 構建與安裝



不使用 launch 檔案直接運行 node：



```bash```bash### 方法 1：在工作區構建（推薦）

ros2 run my_submodule my_submodule_node

```# 進入工作區根目錄



#### 啟用調試日誌cd ~/AprilTag-ROS-2-camera```bash



```bash# 進入工作區根目錄

ros2 run my_submodule my_submodule_node --log-level debug

```# 只構建 my_submodulecd ~/AprilTag-ROS-2-camera



#### 重新映射主題colcon build --packages-select my_submodule



```bash# 只構建 my_submodule

ros2 run my_submodule my_submodule_node \

  --ros-args \# 或構建所有 ROS 2 packagescolcon build --packages-select my_submodule

  -r image_raw:=/camera/image_raw \

  -r output:=/processing/outputcolcon build

```

# 或構建所有 ROS 2 packages

### 方式 3：使用 ROS 2 Composition（進階）

# Source 環境colcon build

利用 ROS 2 的 component 機制，將 node 動態載入到 ComponentManager：

source install/setup.bash

```bash

# 終端 1：啟動 ComponentManager```# Source 環境

ros2 run rclcpp_components component_container

source install/setup.bash

# 終端 2：載入 component

ros2 component load /ComponentManager my_submodule MySubmodule### 方法 2：開發模式構建```



# 檢查已載入的 components

ros2 component list

```適合頻繁修改源代碼的情況：### 方法 2：開發模式構建



### 方式 4：完整工作流（包含相機）



#### 終端 1：啟動相機發佈者```bash```bash

```bash

ros2 launch realsense2_camera rs_launch.pycd ~/AprilTag-ROS-2-cameracd ~/AprilTag-ROS-2-camera

```

colcon build --packages-select my_submodule --symlink-installcolcon build --packages-select my_submodule --symlink-install

#### 終端 2：啟動 my_submodule

```bashsource install/setup.bashsource install/setup.bash

ros2 launch my_submodule my_submodule.launch.py

`````````



#### 終端 3：監聽輸出

```bash

ros2 topic echo /my_submodule/output這樣修改源代碼後無需重新構建。這樣修改源代碼後無需重新構建。

```



---

### 驗證安裝### 驗證安裝

## 📡 ROS 主題與通信



### 訂閱的主題

```bash```bash

| 主題名 | 消息類型 | 預設來源 | 說明 |

|--------|---------|---------|------|# 列出所有 ROS 2 packagesros2 pkg list | grep my_submodule

| `image_raw` | `sensor_msgs/Image` | `/camera/image_raw` | 輸入相機影像 |

ros2 pkg list | grep my_submoduleros2 pkg info my_submodule

### 發佈的主題

```

| 主題名 | 消息類型 | 預設輸出 | 說明 |

|--------|---------|---------|------|# 查看 package 信息

| `output` | `std_msgs/String` | `/my_submodule/output` | 處理結果字串 |

ros2 pkg info my_submodule---

### 實時監聽主題



```bash

# 監聽處理結果# 列出可執行文件## ▶️ 運行 Node

ros2 topic echo /my_submodule/output

ros2 pkg executables my_submodule

# 監聽相機圖像（需要額外工具）

ros2 run image_view image_view image:=/camera/image_raw```### 方式 1：使用 Launch 檔案（推薦）



# 查看所有活躍主題

ros2 topic list

---```bash

# 查看特定主題信息和統計

ros2 topic info /my_submodule/outputros2 launch my_submodule my_submodule.launch.py

ros2 topic hz /my_submodule/output

```## ▶️ 運行 Node```



---



## ⚙️ 配置在運行任何命令前，請確保已 source ROS 2 環境：### 方式 2：直接執行 Node



### YAML 參數配置



編輯 `config/my_submodule.yaml` 來自訂運行時參數：```bash```bash



```yamlsource ~/AprilTag-ROS-2-camera/install/setup.bashros2 run my_submodule my_submodule_node

/**:

  ros__parameters:``````

    # 調試模式（啟用詳細日誌）

    debug: false

    

    # Node 名稱### 方式 1：使用 Launch 檔案（推薦 ⭐）### 方式 3：在 Composition 中載入

    node_name: "my_submodule"

    

    # 定時器週期（毫秒）

    timer_period_ms: 100這是最簡單且推薦的方式。Launch 檔案自動配置所有參數和主題映射。```bash

    

    # 輸入/輸出主題ros2 component load /ComponentManager my_submodule MySubmodule

    input_topic: "image_raw"

    output_topic: "output"```bash```

```

ros2 launch my_submodule my_submodule.launch.py

### 參數說明

```---

| 參數 | 類型 | 預設值 | 說明 |

|------|------|--------|------|

| `debug` | bool | `false` | 啟用調試日誌，顯示詳細信息 |

| `node_name` | string | `my_submodule` | Node 的識別名稱 |**預期輸出**：## ⚙️ 配置

| `timer_period_ms` | int | `100` | 定時器週期（毫秒），值越小處理越頻繁 |

| `input_topic` | string | `image_raw` | 訂閱的影像主題 |```

| `output_topic` | string | `output` | 發佈結果的主題 |

[INFO] [my_submodule_node]: MySubmodule initializing...編輯 `config/my_submodule.yaml` 來自訂參數：

### 在運行時修改參數

[INFO] [my_submodule_node]: MySubmodule initialized successfully

使用 ROS 2 參數 API 動態修改參數：

[DEBUG] [my_submodule_node]: Received image: 640x480```yaml

```bash

# 設定參數```/**:

ros2 param set /my_submodule debug true

  ros__parameters:

# 獲取參數值

ros2 param get /my_submodule timer_period_ms#### 自訂 Launch 參數    # 調試模式



# 列出所有參數    debug: false

ros2 param list /my_submodule

```傳遞自訂參數到 launch 檔案：    



---    # Node 名稱



## 📁 項目結構```bash    node_name: "my_submodule"



```# 設定調試模式    

my_submodule/

├── CMakeLists.txt                     # CMake 構建配置ros2 launch my_submodule my_submodule.launch.py debug:=true    # 定時器週期（毫秒）

├── package.xml                        # ROS 2 Package 定義

├── .gitignore                         # Git 忽略規則    timer_period_ms: 100

├── README.md                          # 本檔案（使用說明）

│# 修改定時器週期（毫秒）    

├── include/my_submodule/

│   └── my_submodule.hpp              # Component 類宣告ros2 launch my_submodule my_submodule.launch.py timer_period_ms:=200    # 輸入/輸出主題

│

├── src/```    input_topic: "image_raw"

│   └── my_submodule.cpp              # Component 實現

│    output_topic: "output"

├── launch/

│   └── my_submodule.launch.py        # Python Launch 配置### 方式 2：直接執行 Node```

│

└── config/

    └── my_submodule.yaml             # YAML 參數配置

```不使用 launch 檔案直接運行 node：### 參數說明



---



## 🔧 開發與修改```bash| 參數 | 類型 | 預設值 | 說明 |



### 修改功能邏輯ros2 run my_submodule my_submodule_node|------|------|--------|------|



編輯 `src/my_submodule.cpp` 實現你的功能：```| `debug` | bool | `false` | 啟用調試日誌 |



#### 1. 定時器回調（每 100ms 執行一次）| `node_name` | string | `my_submodule` | Node 識別名稱 |

```cpp

void MySubmodule::timer_callback()#### 啟用調試日誌| `timer_period_ms` | int | `100` | 定時器週期（毫秒） |

{

  // 你的定時處理邏輯| `input_topic` | string | `image_raw` | 輸入影像主題 |

  auto message = std_msgs::msg::String();

  message.data = "Hello from MySubmodule";```bash| `output_topic` | string | `output` | 輸出結果主題 |

  publisher_->publish(message);

}ros2 run my_submodule my_submodule_node --log-level debug

```

```---

#### 2. 影像訂閱回調（接收相機影像）

```cpp

void MySubmodule::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)

{#### 重新映射主題## 📡 ROS 主題

  // 處理輸入影像

  RCLCPP_DEBUG(

    this->get_logger(),

    "Received image: %dx%d",```bash### 訂閱的主題

    msg->width,

    msg->heightros2 run my_submodule my_submodule_node \

  );

    --ros-args \| 主題 | 類型 | 說明 |

  // 你的影像處理代碼

}  -r image_raw:=/camera/image_raw \|------|------|------|

```

  -r output:=/processing/output| `image_raw` | `sensor_msgs/Image` | 輸入相機影像 |

### 添加新的發佈者/訂閱者

```

1. **在頭文件中宣告** (`include/my_submodule/my_submodule.hpp`)：

   ```cpp### 發佈的主題

   rclcpp::Publisher<your_msgs::msg::Type>::SharedPtr new_publisher_;

   rclcpp::Subscription<your_msgs::msg::Type>::SharedPtr new_subscription_;### 方式 3：使用 ROS 2 Composition（進階）

   ```

| 主題 | 類型 | 說明 |

2. **在建構子中初始化** (`src/my_submodule.cpp`)：

   ```cpp利用 ROS 2 的 component 機制，將 node 動態載入到 ComponentManager：|------|------|------|

   new_publisher_ = this->create_publisher<your_msgs::msg::Type>("topic_name", 10);

   new_subscription_ = this->create_subscription<your_msgs::msg::Type>(| `output` | `std_msgs/String` | 處理結果字串 |

     "input_topic",

     10,```bash

     std::bind(&MySubmodule::callback, this, std::placeholders::_1)

   );# 終端 1：啟動 ComponentManager---

   ```

ros2 run rclcpp_components component_container

3. **重新構建**：

   ```bash## 📁 項目結構

   colcon build --packages-select my_submodule

   source install/setup.bash# 終端 2：載入 component

   ```

ros2 component load /ComponentManager my_submodule MySubmodule```

### 修改主題名稱

my_submodule/

編輯 `launch/my_submodule.launch.py` 中的 `remappings` 部分：

# 檢查已載入的 components├── CMakeLists.txt                # CMake 構建配置

```python

remappings=[ros2 component list├── package.xml                   # ROS 2 Package 描述

    ('image_raw', 'your_camera_topic'),

    ('output', 'your_output_topic'),```├── README.md                     # 本檔案（使用說明）

]

```├── include/my_submodule/



### 修改定時器週期### 方式 4：完整工作流（包含相機）│   └── my_submodule.hpp         # 頭文件（宣告）



在 `config/my_submodule.yaml` 中修改 `timer_period_ms`，或在 launch 時傳遞參數：├── src/



```bash#### 終端 1：啟動相機發佈者│   └── my_submodule.cpp         # 實現文件

ros2 launch my_submodule my_submodule.launch.py timer_period_ms:=50

``````bash├── launch/



---ros2 launch realsense2_camera rs_launch.py│   └── my_submodule.launch.py   # Launch 配置檔案



## 🐛 除錯與故障排除```└── config/



### 1. 啟用詳細日誌    └── my_submodule.yaml        # YAML 參數配置



```bash#### 終端 2：啟動 my_submodule```

# 方法 A：通過 launch 檔案

ros2 launch my_submodule my_submodule.launch.py debug:=true```bash



# 方法 B：通過命令行ros2 launch my_submodule my_submodule.launch.py---

ros2 run my_submodule my_submodule_node --log-level debug

```

# 方法 C：通過環境變量

RCL_LOG_LEVEL=DEBUG ros2 run my_submodule my_submodule_node## 🔧 開發指南

```

#### 終端 3：監聽輸出

### 2. 檢查 Node 狀態

```bash### 修改功能

```bash

# 查看 node 信息ros2 topic echo /my_submodule/output

ros2 node info /my_submodule_node

```編輯 `src/my_submodule.cpp` 實現你的邏輯：

# 查看 node 發佈的主題

ros2 node info /my_submodule_node | grep Publishers



# 查看 node 訂閱的主題---1. **修改定時器回調**

ros2 node info /my_submodule_node | grep Subscriptions

```   ```cpp



### 3. 監聽主題內容## 📡 ROS 主題與通信   void MySubmodule::timer_callback()



```bash   {

# 實時監聽輸出

ros2 topic echo /my_submodule/output### 訂閱的主題     // 你的定時處理邏輯



# 只打印前 10 條消息   }

ros2 topic echo /my_submodule/output --limit 10

| 主題名 | 消息類型 | 預設來源 | 說明 |   ```

# 監聽相機影像（需要 image_view）

ros2 run image_view image_view image:=/camera/image_raw|--------|---------|---------|------|

```

| `image_raw` | `sensor_msgs/Image` | `/camera/image_raw` | 輸入相機影像 |2. **修改影像訂閱回調**

### 4. 常見問題

   ```cpp

#### 找不到 package

```bash### 發佈的主題   void MySubmodule::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)

# 原因：沒有 source 環境

# 解決：   {

source ~/AprilTag-ROS-2-camera/install/setup.bash

| 主題名 | 消息類型 | 預設輸出 | 說明 |     // 處理輸入影像

# 或檢查構建是否成功

colcon build --packages-select my_submodule|--------|---------|---------|------|   }

```

| `output` | `std_msgs/String` | `/my_submodule/output` | 處理結果字串 |   ```

#### Node 無法訂閱影像

```bash

# 檢查相機是否在發佈

ros2 topic list | grep image### 實時監聽主題3. **添加新的發佈者/訂閱者**



# 如果沒有，先啟動相機驅動   - 在 `.hpp` 檔案中宣告成員

ros2 launch realsense2_camera rs_launch.py

``````bash   - 在建構子中初始化



#### 參數未被應用# 監聽處理結果   - 編譯並重新構建

```bash

# 檢查參數是否存在ros2 topic echo /my_submodule/output

ros2 param list /my_submodule

### 修改主題名稱

# 查看參數值

ros2 param get /my_submodule timer_period_ms# 監聽相機圖像（需要額外工具）



# 使用 launch 參數重寫ros2 run image_view image_view image:=/camera/image_raw編輯 `launch/my_submodule.launch.py` 中的 `remappings`：

ros2 launch my_submodule my_submodule.launch.py timer_period_ms:=200

```



---# 查看所有活躍主題```python



## 📊 性能監控ros2 topic listremappings=[



### 測量 Node 延遲    ('image_raw', 'your_camera_topic'),



```bash# 查看特定主題信息和統計    ('output', 'your_output_topic'),

# 訂閱主題並測量頻率

ros2 topic hz /my_submodule/outputros2 topic info /my_submodule/output]



# 預期輸出ros2 topic hz /my_submodule/output```

# average rate: 10.02 Hz

# min: 95.120 ms max: 107.080 ms std dev: 3.43 ms```

```

---

### CPU 與記憶體使用

---

```bash

# 使用 ros2_monitor（需要安裝）## 🐛 除錯

ros2 monitor

## ⚙️ 配置

# 或使用系統工具

top -p $(pgrep -f my_submodule_node)### 啟用詳細日誌

```

### YAML 參數配置

---

```bash

## 📚 相關資源

編輯 `config/my_submodule.yaml` 來自訂運行時參數：# 設定日誌級別為 DEBUG

- **ROS 2 官方文檔**: https://docs.ros.org/en/humble/

- **ROS 2 Component 指南**: https://docs.ros.org/en/humble/Concepts/Advanced/Composition.htmlros2 run my_submodule my_submodule_node --log-level debug

- **ROS 2 Launch 教程**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html

- **AprilTag-ROS-2 源專案**: https://github.com/Tinker-Twins/AprilTag-ROS-2```yaml```

- **Python RCL API**: https://docs.ros.org/en/humble/Concepts/About-ROS-2/DDS-and-ROS-concept-mapping.html

/**:

---

  ros__parameters:### 監聽輸出主題

## 📝 License

    # 調試模式（啟用詳細日誌）

**BSD License**

    debug: false```bash

---

    ros2 topic echo /my_submodule/output

## 📞 支援與反饋

    # Node 名稱```

如有問題、建議或改進意見，歡迎：

- 提出 GitHub Issue    node_name: "my_submodule"

- 發起 Pull Request

- 提供反饋    ### 檢查 Node 狀態



---    # 定時器週期（毫秒）



## 🔄 版本歷史    timer_period_ms: 100```bash



- **v0.1.0** (2025-11-12): 初始發佈    ros2 node info /my_submodule_node

  - 基本 ROS 2 component 實現

  - Launch 檔案支援    # 輸入/輸出主題```

  - YAML 參數配置

  - 詳細文檔    input_topic: "image_raw"



---    output_topic: "output"---



**最後更新**: 2025-11-12  ```

**分支**: `ros-package-node`  

**維護者**: AprilTag-ROS-2-camera 開發團隊## 📚 相關資源


### 參數說明

- [ROS 2 官方文檔](https://docs.ros.org/en/humble/)

| 參數 | 類型 | 預設值 | 說明 |- [ROS 2 Component 指南](https://docs.ros.org/en/humble/Concepts/Advanced/Composition.html)

|------|------|--------|------|- [AprilTag-ROS-2 源專案](https://github.com/Tinker-Twins/AprilTag-ROS-2)

| `debug` | bool | `false` | 啟用調試日誌，顯示詳細信息 |

| `node_name` | string | `my_submodule` | Node 的識別名稱 |---

| `timer_period_ms` | int | `100` | 定時器週期（毫秒），值越小處理越頻繁 |

| `input_topic` | string | `image_raw` | 訂閱的影像主題 |## 📝 License

| `output_topic` | string | `output` | 發佈結果的主題 |

**BSD License**

### 在運行時修改參數

---

使用 ROS 2 參數 API 動態修改參數：

## 📞 支援

```bash

# 設定參數如有問題或建議，請提出 Issue 或 Pull Request。

ros2 param set /my_submodule debug true

---

# 獲取參數值

ros2 param get /my_submodule timer_period_ms**最後更新**: 2025-11-12


# 列出所有參數
ros2 param list /my_submodule
```

---

## 📁 項目結構

```
my_submodule/
├── CMakeLists.txt                     # CMake 構建配置
├── package.xml                        # ROS 2 Package 描述
├── .gitignore                         # Git 忽略文件規則
├── README.md                          # 本檔案（使用說明）
│
├── include/my_submodule/
│   └── my_submodule.hpp              # 頭文件（類宣告）
│
├── src/
│   └── my_submodule.cpp              # 實現文件（邏輯實現）
│
├── launch/
│   └── my_submodule.launch.py        # Python Launch 配置
│
└── config/
    └── my_submodule.yaml             # YAML 參數配置
```

---

## 🔧 開發與修改

### 修改功能邏輯

編輯 `src/my_submodule.cpp` 實現你的功能：

#### 1. 定時器回調（每 100ms 執行一次）
```cpp
void MySubmodule::timer_callback()
{
  // 你的定時處理邏輯
  auto message = std_msgs::msg::String();
  message.data = "Hello from MySubmodule";
  publisher_->publish(message);
}
```

#### 2. 影像訂閱回調（接收相機影像）
```cpp
void MySubmodule::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // 處理輸入影像
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received image: %dx%d",
    msg->width,
    msg->height
  );
  
  // 你的影像處理代碼
}
```

### 添加新的發佈者/訂閱者

1. **在頭文件中宣告** (`include/my_submodule/my_submodule.hpp`)：
   ```cpp
   rclcpp::Publisher<your_msgs::msg::Type>::SharedPtr new_publisher_;
   rclcpp::Subscription<your_msgs::msg::Type>::SharedPtr new_subscription_;
   ```

2. **在建構子中初始化** (`src/my_submodule.cpp`)：
   ```cpp
   new_publisher_ = this->create_publisher<your_msgs::msg::Type>("topic_name", 10);
   new_subscription_ = this->create_subscription<your_msgs::msg::Type>(
     "input_topic",
     10,
     std::bind(&MySubmodule::callback, this, std::placeholders::_1)
   );
   ```

3. **重新構建**：
   ```bash
   colcon build --packages-select my_submodule
   source install/setup.bash
   ```

### 修改主題名稱

編輯 `launch/my_submodule.launch.py` 中的 `remappings` 部分：

```python
remappings=[
    ('image_raw', 'your_camera_topic'),
    ('output', 'your_output_topic'),
]
```

### 修改定時器週期

在 `config/my_submodule.yaml` 中修改 `timer_period_ms`，或在 launch 時傳遞參數：

```bash
ros2 launch my_submodule my_submodule.launch.py timer_period_ms:=50
```

---

## 🐛 除錯與故障排除

### 1. 啟用詳細日誌

```bash
# 方法 A：通過 launch 檔案
ros2 launch my_submodule my_submodule.launch.py debug:=true

# 方法 B：通過命令行
ros2 run my_submodule my_submodule_node --log-level debug

# 方法 C：通過環境變量
RCL_LOG_LEVEL=DEBUG ros2 run my_submodule my_submodule_node
```

### 2. 檢查 Node 狀態

```bash
# 查看 node 信息
ros2 node info /my_submodule_node

# 查看 node 發佈的主題
ros2 node info /my_submodule_node | grep Publishers

# 查看 node 訂閱的主題
ros2 node info /my_submodule_node | grep Subscriptions
```

### 3. 監聽主題內容

```bash
# 實時監聽輸出
ros2 topic echo /my_submodule/output

# 只打印前 10 條消息
ros2 topic echo /my_submodule/output --limit 10

# 監聽相機影像（需要 image_view）
ros2 run image_view image_view image:=/camera/image_raw
```

### 4. 常見問題

#### 找不到 package
```bash
# 原因：沒有 source 環境
# 解決：
source ~/AprilTag-ROS-2-camera/install/setup.bash

# 或檢查構建是否成功
colcon build --packages-select my_submodule
```

#### Node 無法訂閱影像
```bash
# 檢查相機是否在發佈
ros2 topic list | grep image

# 如果沒有，先啟動相機驅動
ros2 launch realsense2_camera rs_launch.py
```

#### 參數未被應用
```bash
# 檢查參數是否存在
ros2 param list /my_submodule

# 查看參數值
ros2 param get /my_submodule timer_period_ms

# 使用 launch 參數重寫
ros2 launch my_submodule my_submodule.launch.py timer_period_ms:=200
```

---

## 📊 性能監控

### 測量 Node 延遲

```bash
# 訂閱主題並測量頻率
ros2 topic hz /my_submodule/output

# 預期輸出
# average rate: 10.02 Hz
# min: 95.120 ms max: 107.080 ms std dev: 3.43 ms
```

### CPU 與記憶體使用

```bash
# 使用 ros2_monitor（需要安裝）
ros2 monitor

# 或使用系統工具
top -p $(pgrep -f my_submodule_node)
```

---

## 📚 相關資源

- **ROS 2 官方文檔**: https://docs.ros.org/en/humble/
- **ROS 2 Component 指南**: https://docs.ros.org/en/humble/Concepts/Advanced/Composition.html
- **ROS 2 Launch 教程**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html
- **AprilTag-ROS-2 源專案**: https://github.com/Tinker-Twins/AprilTag-ROS-2
- **Python RCL API**: https://docs.ros.org/en/humble/Concepts/About-ROS-2/DDS-and-ROS-concept-mapping.html

---

## 📝 License

**BSD License**

---

## 📞 支援與反饋

如有問題、建議或改進意見，歡迎：
- 提出 GitHub Issue
- 發起 Pull Request
- 提供反饋

---

## 🔄 版本歷史

- **v0.1.0** (2025-11-12): 初始發佈
  - 基本 ROS 2 component 實現
  - Launch 檔案支援
  - YAML 參數配置
  - 詳細文檔

---

**最後更新**: 2025-11-12  
**分支**: `feature/my_submodule`  
**維護者**: AprilTag-ROS-2-camera 開發團隊
