#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <fstream>
#include <string>
#include "../include/VmrQrCameraApi.h"

int image_count = 0;
int accept_count = 0;
int error_count = 0;

void save_pgm(const std::string& filename, const void* data, int width, int height) {
    std::ofstream file(filename, std::ios::binary);
    if (file.is_open()) {
        file << "P5\n" << width << " " << height << "\n255\n";
        file.write(reinterpret_cast<const char*>(data), width * height);
    }
}

// 图像回调函数
void OnImageCallback(VmrQrCameraImageData* image, void* user_data) {
    if (image) {
        std::cout << "[Demo] Image received: "
                  << "Size=" << image->width << "x" << image->height
                  << ", Frame=" << image->frame_count << std::endl;

        // std::string filename = "image_" + std::to_string(image->frame_count) + ".pgm";
        // save_pgm(filename, image->data, image->width, image->height);

        ++image_count;
    }
}

// 检测结果回调函数
void OnDetectionCallback(VmrQrCameraDetectionData* result, void* user_data) {
    if (result) {
        std::cout << "------------------------------------------------" << std::endl;
        std::cout << "[Demo] Detection Result (Frame " << result->frame_count << "):" << std::endl;
        std::cout << "       Code: " << result->code << std::endl;
        std::cout << "       Pose: x=" << result->x << ", y=" << result->y << ", theta=" << result->theta << std::endl;
        std::cout << "       Timestamp: " << result->timestamp_ns << std::endl;
        ++accept_count;
    }else{
        std::cout <<" null "<<std::endl;
        ++error_count;
    }
}

int main(int argc, char** argv) {
    std::cout << "Starting VmrQrCamera Demo..." << std::endl;

    VmrQrCameraVersion version;
    if (VMR_QC_GetVersion(&version) == VmrQrCameraNormal) {
        std::cout << "SDK Version: Driver=" << version.driver_version
                  << ", Algo=" << version.algo_version << std::endl;
    }

    VmrQrCameraHandle handle = nullptr;
    VmrQrCameraExc ret = VMR_QC_CreateHandle(&handle);
    if (ret != VmrQrCameraNormal) {
        std::cerr << "Failed to create handle." << std::endl;
        return -1;
    }

    // 默认 IP，也可以通过命令行参数传入
    const char* camera_ip = "192.168.100.5";
    if (argc > 1) {
        camera_ip = argv[1];
    }

    std::cout << "Opening device: " << camera_ip << std::endl;
    ret = VMR_QC_OpenDevice(handle, camera_ip);
    if (ret != VmrQrCameraNormal) {
        std::cerr << "Failed to open device. Please check IP and connection." << std::endl;
        VMR_QC_ReleaseHandle(handle);
        return -1;
    }

    // 1. 设置相机基本参数
    VMR_QC_SetDeviceParam(handle, VmrQrCameraParam_Exposure, 300);
    VMR_QC_SetDeviceParam(handle, VmrQrCameraParam_Gain, 250);
    VMR_QC_SetDeviceParam(handle, VmrQrCameraParam_LedBrightness, 100);

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
    // 注意：fx, fy, cx, cy 已移除，SDK 内部会自动读取相机内参
    qr_param.qr_need = 3;
    qr_param.dm_symbol = 3;
    qr_param.decode_timeout = 25;
    qr_param.big_circle_flag = false;   // 开启大圆检测
    qr_param.small_circle_flag = false;
    qr_param.save_video = false;
    qr_param.encode_type = 1;

    ret = VMR_QC_SetDetectionParams(handle, &qr_param);
    if (ret != VmrQrCameraNormal) {
        std::cerr << "Failed to set QR params." << std::endl;
    }

    // 3. 注册回调
    VMR_QC_SetImageCallback(handle, OnImageCallback, nullptr);
    VMR_QC_SetDetectionCallback(handle, OnDetectionCallback, nullptr);

    // 4. 开始采集
    std::cout << "Starting capture..." << std::endl;
    ret = VMR_QC_StartCapture(handle);
    if (ret != VmrQrCameraNormal) {
        std::cerr << "Failed to start capture." << std::endl;
        VMR_QC_CloseDevice(handle);
        VMR_QC_ReleaseHandle(handle);
        return -1;
    }

    // 运行一段时间
    std::cout << "Running for 20 seconds..." << std::endl;
    int i=5;
    while(--i){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "****** accept_cout " << accept_count <<" , error_count "<< error_count << " , image_count "<< image_count <<"************" << std::endl;
    }

    // 5. 停止采集
    std::cout << "Stopping capture..." << std::endl;
    VMR_QC_StopCapture(handle);

    // 6. 关闭设备并释放资源
    VMR_QC_CloseDevice(handle);
    VMR_QC_ReleaseHandle(handle);
    return 0;
}
