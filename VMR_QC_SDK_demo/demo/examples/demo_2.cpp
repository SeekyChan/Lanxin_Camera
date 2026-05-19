#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include "../include/VmrQrCameraApi.h"

// 图像回调函数
void OnImageCallback(VmrQrCameraImageData* image, void* user_data) {
    if (image) {
        // 注意：不要在这里做耗时操作，否则会阻塞采集线程
        // std::cout << "[Demo] Image received: "
        //           << "Size=" << image->width << "x" << image->height
        //           << ", Frame=" << image->frame_count << std::endl;
    }
}

// 检测结果回调函数
void OnDetectionCallback(VmrQrCameraDetectionData* result, void* user_data) {
    if (result) {
        std::cout << "------------------------------------------------" << std::endl;
        std::cout << "[Demo] Detection Result (Frame " << result->frame_count << "):" << std::endl;
        std::cout << "       Code: " << result->code << std::endl;
        std::cout << "       Pose: x=" << result->x << ", y=" << result->y << ", theta=" << result->theta << std::endl;
    }else{
        std::cout <<" null "<<std::endl;
    }
}

int main(int argc, char** argv) {
    std::cout << "Starting VmrQrCamera Demo..." << std::endl;

    VmrQrCameraHandle handle = nullptr;
    VmrQrCameraHandle handle2 = nullptr;

    VmrQrCameraExc ret = VMR_QC_CreateHandle(&handle);
    VmrQrCameraExc ret2 = VMR_QC_CreateHandle(&handle2);

    if (ret != VmrQrCameraNormal) {
        std::cerr << "Failed to create handle." << std::endl;
        return -1;
    }

    // 默认 IP，也可以通过命令行参数传入
    const char* camera_ip = "192.168.100.7";
    const char* camera_ip2 = "192.168.10.106";

    if (argc > 1) {
        camera_ip = argv[1];
    }

    std::cout << "Opening device: " << camera_ip << std::endl;
    ret = VMR_QC_OpenDevice(handle, camera_ip);
    ret2 = VMR_QC_OpenDevice(handle2, camera_ip2);

    if (ret != VmrQrCameraNormal) {
        std::cerr << "Failed to open device. Please check IP and connection." << std::endl;
        VMR_QC_ReleaseHandle(handle);
        return -1;
    }

    // 1. 设置相机基本参数
    VMR_QC_SetDeviceParam(handle, VmrQrCameraParam_Exposure, 300);
    VMR_QC_SetDeviceParam(handle, VmrQrCameraParam_Gain, 250);
    VMR_QC_SetDeviceParam(handle, VmrQrCameraParam_LedBrightness, 100);

    VMR_QC_SetDeviceParam(handle2, VmrQrCameraParam_Exposure, 300);
    VMR_QC_SetDeviceParam(handle2, VmrQrCameraParam_Gain, 250);
    VMR_QC_SetDeviceParam(handle2, VmrQrCameraParam_LedBrightness, 100);
    
    // 2. 设置二维码算法参数
    VmrQrCameraDetectionParams qr_param;
    memset(&qr_param, 0, sizeof(VmrQrCameraDetectionParams)); // 清零

    // 填充默认参数 (根据实际场景调整)
    qr_param.rows = 4;
    qr_param.cols = 4;
    qr_param.qr_size = 0.018;
    qr_param.qr_interval = 0.005;
    qr_param.radius_big = 0.016;
    qr_param.radius_small = 0.003;
    qr_param.circle_dis = 0.0425;
    qr_param.point_height = 0.1;
    qr_param.big_circle_width = 0.003;
    // 注意：fx, fy, cx, cy 已移除
    qr_param.qr_need = 3;
    qr_param.dm_symbol = 3;
    qr_param.decode_timeout = 200;
    qr_param.big_circle_flag = false;   // 开启大圆检测
    qr_param.small_circle_flag = false;
    qr_param.save_video = false;

    ret = VMR_QC_SetDetectionParams(handle, &qr_param);
    ret2 = VMR_QC_SetDetectionParams(handle2, &qr_param);
    if (ret != VmrQrCameraNormal) {
        std::cerr << "Failed to set QR params." << std::endl;
    }

    // 3. 注册回调
    VMR_QC_SetImageCallback(handle, OnImageCallback, nullptr);
    VMR_QC_SetDetectionCallback(handle, OnDetectionCallback, nullptr);
    VMR_QC_SetImageCallback(handle2, OnImageCallback, nullptr);
    VMR_QC_SetDetectionCallback(handle2, OnDetectionCallback, nullptr);

    // 4. 开始采集
    std::cout << "Starting capture..." << std::endl;
    ret = VMR_QC_StartCapture(handle);
    ret2 = VMR_QC_StartCapture(handle2);
    if (ret != VmrQrCameraNormal) {
        std::cerr << "Failed to start capture." << std::endl;
        VMR_QC_CloseDevice(handle);
        VMR_QC_ReleaseHandle(handle);
    }
    if (ret2 != VmrQrCameraNormal) {
        std::cerr << "Failed to start capture." << std::endl;
        VMR_QC_CloseDevice(handle2);
        VMR_QC_ReleaseHandle(handle2);
    }
    // 运行一段时间
    std::cout << "Running for 20 seconds..." << std::endl;
    for (int i = 0; i < 20; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (i % 5 == 0) std::cout << "..." << std::endl;
    }

    // 5. 停止采集
    std::cout << "Stopping capture..." << std::endl;
    VMR_QC_StopCapture(handle);

    // 6. 关闭设备并释放资源
    VMR_QC_CloseDevice(handle);
    VMR_QC_ReleaseHandle(handle);


    VMR_QC_StopCapture(handle2);

    // 6. 关闭设备并释放资源
    VMR_QC_CloseDevice(handle2);
    VMR_QC_ReleaseHandle(handle2);
    return 0;
}
