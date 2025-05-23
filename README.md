## ZED Camera Open Capture (CPU Only)

Fully running on CPU only – no GPU or CUDA required. Ideal for lightweight systems, educational purposes, and CPU-only simulations.

### Key Features:

  - Access ZED camera frames using the ZED Open Capture API

  - Detect objects based on color (e.g., red ball detection using HSV filtering)

  - Estimate object distance by sampling depth at the segmented region's center

  - Lightweight and GPU-free: all processing runs on CPU

### Stack:

  - Camera: ZED (tested with ZED 2)

  - SDK: ZED Open Capture

  - Image Processing: OpenCV (for segmentation and drawing)

  - Language: C++

  - Execution: CPU only

### How to Build:

Install dependencies:

  - OpenCV 4.x or newer

  - ZED Open Capture SDK

  - CMake 3.10+

    Clone and build the project:

      git clone https://github.com/your-username/zed-open-segment-depth.git
      cd zed-open-segment-depth
      mkdir build && cd build
      cmake ..
      make

### How to Run:

Make sure the ZED camera is connected and recognized.

    ./testing

### Result

