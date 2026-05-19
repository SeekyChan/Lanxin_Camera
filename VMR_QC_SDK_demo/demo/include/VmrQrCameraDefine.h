#ifndef VMR_QRCAMERA_DEFINE_H
#define VMR_QRCAMERA_DEFINE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 句柄定义
typedef void* VmrQrCameraHandle;

// 错误码定义
typedef enum {
  VmrQrCameraNormal = 0,          // 成功
  VmrQrCameraHandleInvalid = -1,  // 无效句柄
  VmrQrCameraNotCapturing = -2,   // 未处于采集状态
  VmrQrCameraAbnormalStop = -3,   // 异常停止
  VmrQrCameraTimeout = -4,        // 操作超时
  VmrQrCameraFail = -5,           // 一般性失败
  VmrQrCameraInvalidParam = -6,   // 无效参数
} VmrQrCameraExc;

// 相机参数类型枚举
typedef enum {
  VmrQrCameraParam_Exposure = 0,   // 曝光时间
  VmrQrCameraParam_Gain,           // 增益
  VmrQrCameraParam_LedBrightness,  // LED亮度
} VmrQrCameraParamType;

// 版本信息结构体
typedef struct {
  char driver_version[32];  // 驱动版本
  char algo_version[32];    // 算法版本
} VmrQrCameraVersion;

// 相机参数类型定义
typedef struct {
  int exposure_time = 300;   // 曝光时间，单位微秒 (us)
  int gain = 250;            // 模拟增益值
  int led_brightness = 100;  // LED补光灯亮度，范围 0-100
} VmrQrCameraDeviceParams;

// 识别算法参数定义
typedef struct {
  int rows = 1;            // 组合码行数 (如果是单个码则为1)
  int cols = 1;            // 组合码列数 (如果是单个码则为1)
  double qr_size = 0.018;  // 单个二维码的物理边长 (单位: 米)
  double qr_interval = 0.005;  // 组合码中二维码之间的物理间距 (单位: 米)
  double point_height = 0.1;  // 预期的识别距离 (单位: 米)
  int qr_need = 3;            // 二维码最大识别个数
  int dm_symbol = 3;          // DM码规格 (DM12:1, DM14为2, DM16为3)
  float decode_timeout = 50;  // 解码超时时间 (单位: 毫秒)

  bool big_circle_flag = false;     // 是否启用大圆识别
  double radius_big = 0.02;         // 辅助定位大圆半径 (单位: 米)
  double big_circle_width = 0.003;  // 大圆圆环宽度 (单位: 米)

  bool small_circle_flag = false;  // 是否启用小圆识
  double radius_small = 0.003;     // 辅助定位小圆半径 (单位: 米)
  double circle_dis = 0.0425;      // 小圆圆心间距 (单位: 米)

  int encode_type = 1;  // 编码类型 (蓝芯: 1, 倍加福: 5)

  bool save_video = false;  // 是否绘制检测结果 (用于调试)
} VmrQrCameraDetectionParams;

// 图像数据结构体
typedef struct {
  const char* pixel_format;  // 像素格式字符串 (如 "mono8", "bgr8")
  const void* data;          // 图像数据指针
  int width;                 // 图像宽度 (像素)
  int height;                // 图像高度 (像素)
  size_t size;               // 数据缓冲区大小 (字节)
  unsigned int frame_count;  // 帧序号
  uint64_t timestamp_ns;     // 时间戳 (纳秒)
} VmrQrCameraImageData;

// 检测结果结构体
typedef struct {
  char code[64];  // 解码结果字符串
  float x;  // 表示相机在二维码坐标系下的位置 (单位: 米和弧度)
  float y;
  float theta;
  unsigned int frame_count;  // 对应的帧序号
  uint64_t timestamp_ns;     // 时间戳 (纳秒)
} VmrQrCameraDetectionData;

// 图像回调：当相机采集到新图像时调用
typedef void (*VmrQrCameraImageCallback)(VmrQrCameraImageData* image,
                                         void* user_data);

// 检测回调：当算法完成检测并获得结果时调用
typedef void (*VmrQrCameraDetectionCallback)(VmrQrCameraDetectionData* result,
                                             void* user_data);

#ifdef __cplusplus
}
#endif

#endif  // VMR_QRCAMERA_DEFINE_H
