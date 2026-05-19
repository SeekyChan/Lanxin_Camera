#include <iostream>
#include <string>
#include <fstream>
#include <thread>
#include <chrono>
#include <cstring>
#include <mutex>
#include <condition_variable>
#include "VmrQrCameraApi.h"

// 简单的同步机制，用于 Demo 演示等待结果
std::mutex g_mutex;
std::condition_variable g_cv;
bool g_result_received = false;

// 检测结果回调
void OnDetectionCallback(VmrQrCameraDetectionData* result, void* user_data) {
    if (result) {
        std::cout << "  [Callback] Found Code: " << result->code
                  << " (Frame " << result->frame_count << ")" << std::endl;
        std::cout << "             Pose: x=" << result->x << ", y=" << result->y << ", theta=" << result->theta << std::endl;

        // 通知主线程收到结果了
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_result_received = true;
        }
        g_cv.notify_one();
    }
}

// 辅助函数：保存 PGM 图片
void save_pgm(const char* filename, const void* data, int width, int height) {
    std::ofstream file(filename, std::ios::binary);
    if (file.is_open()) {
        file << "P5\n" << width << " " << height << "\n255\n";
        file.write(reinterpret_cast<const char*>(data), width * height);
        file.close();
        std::cout << "[Demo] Saved image to " << filename << std::endl;
    } else {
        std::cerr << "[Demo] Failed to save image to " << filename << std::endl;
    }
}

int main(int argc, char** argv) {
    std::string ip = "192.168.100.7";
    if (argc > 1) {
        ip = argv[1];
    }

    std::cout << "=== Snapshot & Detect Demo ===" << std::endl;

    VmrQrCameraHandle handle = nullptr;
    VmrQrCameraExc ret = VMR_QC_CreateHandle(&handle);
    if (ret != VmrQrCameraNormal) {
        std::cerr << "Failed to create handle." << std::endl;
        return -1;
    }

    std::cout << "Connecting to " << ip << "..." << std::endl;
    ret = VMR_QC_OpenDevice(handle, ip.c_str());
    if (ret != VmrQrCameraNormal) {
        std::cerr << "Failed to open device." << std::endl;
        VMR_QC_ReleaseHandle(handle);
        return -1;
    }

    // 设置相机参数
    VMR_QC_SetDeviceParam(handle, VmrQrCameraParam_Exposure, 300);
    VMR_QC_SetDeviceParam(handle, VmrQrCameraParam_Gain, 250);
    VMR_QC_SetDeviceParam(handle, VmrQrCameraParam_LedBrightness, 100);

    // 设置二维码参数 (重要：必须设置，否则无法检测)
    VmrQrCameraDetectionParams qr_param;
    memset(&qr_param, 0, sizeof(VmrQrCameraDetectionParams));
    qr_param.rows = 1;
    qr_param.cols = 1;
    qr_param.qr_size = 0.024;
    qr_param.radius_big = 0.02;
    qr_param.radius_small = 0.003;
    qr_param.circle_dis = 0.0425;
    qr_param.point_height = 0.1;
    qr_param.big_circle_width = 0.003;
    // 注意：fx, fy, cx, cy 已移除
    qr_param.qr_need = 1;
    qr_param.dm_symbol = 3;
    qr_param.decode_timeout = 20;
    qr_param.big_circle_flag = true;
    qr_param.small_circle_flag = false;

    VMR_QC_SetDetectionParams(handle, &qr_param);

    // 注册回调
    VMR_QC_SetDetectionCallback(handle, OnDetectionCallback, nullptr);

    // 抓取 3 张图片
    for (int i = 0; i < 3; ++i) {
        std::cout << "\nTaking snapshot " << i + 1 << "..." << std::endl;

        // 重置标志位
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_result_received = false;
        }

        VmrQrCameraImageData image;
        // 同步抓图，超时时间 2000ms
        ret = VMR_QC_Snapshot(handle, &image, 2000);

        if (ret == VmrQrCameraNormal) {
            std::cout << "Snapshot success! Size: " << image.width << "x" << image.height << std::endl;

            // 保存图片
            std::string filename = "snapshot_" + std::to_string(i) + ".pgm";
            save_pgm(filename.c_str(), image.data, image.width, image.height);

            std::cout << "Snapshot success. Waiting for detection..." << std::endl;

            // 等待回调通知，最多等 1 秒
            std::unique_lock<std::mutex> lock(g_mutex);
            if (g_cv.wait_for(lock, std::chrono::milliseconds(1000), []{ return g_result_received; })) {
                std::cout << "Detection completed." << std::endl;
            } else {
                std::cout << "Detection timeout or no code found." << std::endl;
            }

            // 释放图片内存
            VMR_QC_ReleaseImage(&image);
        } else {
            std::cerr << "Snapshot failed. Error code: " << ret << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "Closing device..." << std::endl;
    VMR_QC_CloseDevice(handle);
    VMR_QC_ReleaseHandle(handle);
    return 0;
}
