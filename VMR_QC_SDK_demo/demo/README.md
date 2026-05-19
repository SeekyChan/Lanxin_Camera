# VMR 二维码相机 SDK 示例 (VMR QR Camera SDK Demo)

本项目包含 VMR 二维码相机 SDK (`VMR_QCSDK`) 的示例程序。它演示了如何连接相机、配置参数、获取图像以及接收二维码检测结果。

## 前提条件 (Prerequisites)

在编译项目之前，请确保已安装以下依赖项：

```bash
sudo apt update
sudo apt install libceres-dev libopencv-dev build-essential cmake
```

## 编译说明 (Build Instructions)

1.  进入 examples 目录：
    ```bash
    cd examples
    ```

2.  创建一个构建目录：
    ```bash
    mkdir -p build
    cd build
    ```

3.  运行 CMake 和 make：
    ```bash
    cmake ..
    make
    ```

## 运行示例 (Running the Demos)

在运行可执行文件之前，你需要设置 `LD_LIBRARY_PATH` 以包含本地的 `lib` 目录。

### 1. 基础示例 (`demo`)

连接到单个相机，启动连续采集，并打印检测结果。
示例： demo.cpp
### 2. 双相机示例 (`demo2`)

演示同时连接两个相机。
示例： demo2.cpp

**用法：**
```bash
export LD_LIBRARY_PATH=$(pwd)/lib:$LD_LIBRARY_PATH
./build/demo2
```
*  **注意：** IP 地址目前在 `demo_2.cpp` 中是硬编码的（`192.168.100.7` 和 `192.168.10.106`）。你可能需要修改源代码以匹配你的相机设置。

### 3. 快照示例 (`snapshot_demo`)

演示拍摄单张快照（软触发），将其保存为 PGM 图像，并获取检测结果。

## 项目结构 (Project Structure)

*   `examples/demo.cpp`: 用于连续采集和检测的基础示例。
*   `examples/demo_2.cpp`: 连接两个相机的示例。
*   `examples/snapshot_demo.cpp`: 快照触发和图像保存的示例。
*   `include/`: SDK 头文件 (`VmrQrCameraApi.h` 等)。
*   `lib/`: SDK 和依赖库 (`libaravis` 等)。
