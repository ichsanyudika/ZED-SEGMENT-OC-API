## 🎥 ZED Camera Open Capture (CPU Only)

Real-time color object detection and depth estimation — all on CPU!
No GPU, no CUDA — just pure processing with OpenCV and ZED Open Capture.
Perfect for lightweight systems, educational projects, and CPU-only simulations.

🔗 Source: https://github.com/stereolabs/zed-open-capture

### ✨ Key Features

✅ Access ZED camera frames via ZED Open Capture API
🎨 Perform color-based object detection using HSV filtering
📏 Estimate depth at the center of detected object
💻 Fully optimized for CPU-only execution

### ⚙️ Tech Stack

  - Camera: ZED (tested on ZED 2)

  - SDK: ZED Open Capture

  - Image Processing: OpenCV (segmentation, contour, drawing)

  - Programming Language: C++

  - Execution Mode: CPU only (no CUDA)

### 🔧 Prerequisites

Ensure the following dependencies are installed:

  - OpenCV (version 4.x or newer)

  - ZED Open Capture SDK

  - CMake (version 3.10 or newer)

### 📦 Clone & Build

    git clone https://github.com/ichsanyudika/ZED-SEGMENT-OC-API.git
    cd ZED-SEGMENT-OC-API
    mkdir build && cd build
    cmake ..
    make

### ▶️ Run the Program

Ensure your ZED camera is connected and accessible, then run:

    ./testing
