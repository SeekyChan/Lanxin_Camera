#ifndef VMR_QRCAMERA_API_H
#define VMR_QRCAMERA_API_H

#include "VmrQrCameraDefine.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建相机句柄
 * @param handle [out] 返回的句柄
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_CreateHandle(VmrQrCameraHandle* handle);

/**
 * @brief 设置相机参数
 * @param handle [in] 句柄
 * @param type [in] 参数类型
 * @param value [in] 参数值
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_SetDeviceParam(VmrQrCameraHandle handle,
                                     VmrQrCameraParamType type, int value);

/**
 * @brief 设置二维码检测参数
 * @param handle [in] 句柄
 * @param param [in] 参数类型
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_SetDetectionParams(
    VmrQrCameraHandle handle, const VmrQrCameraDetectionParams* param);

/**
 * @brief 打开设备
 * @param handle [in] 句柄
 * @param ip [in] 相机IP地址
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_OpenDevice(VmrQrCameraHandle handle, const char* ip);

/**
 * @brief 设置图像数据回调
 * @param handle [in] 句柄
 * @param callback [in] 回调函数
 * @param user_data [in] 用户数据
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_SetImageCallback(VmrQrCameraHandle handle,
                                       VmrQrCameraImageCallback callback,
                                       void* user_data);

/**
 * @brief 设置检测结果回调
 * @param handle [in] 句柄
 * @param callback [in] 回调函数
 * @param user_data [in] 用户数据
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_SetDetectionCallback(
    VmrQrCameraHandle handle, VmrQrCameraDetectionCallback callback,
    void* user_data);

/**
 * @brief 开始取流
 * @param handle [in] 句柄
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_StartCapture(VmrQrCameraHandle handle);

/**
 * @brief 停止取流
 * @param handle [in] 句柄
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_StopCapture(VmrQrCameraHandle handle);

/**
 * @brief 抓取单张图像 (同步模式)
 * @note 不能与 VMR_QC_StartCapture 同时使用，调用前需确保未开启连续采集
 * @param handle [in] 句柄
 * @param image [out] 返回的图像结构体，使用完后需调用 VMR_QC_ReleaseImage
 * 释放内存
 * @param timeout_ms [in] 超时时间(ms)
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_Snapshot(VmrQrCameraHandle handle,
                               VmrQrCameraImageData* image,
                               unsigned int timeout_ms);

/**
 * @brief 释放图像内存 (配合 VMR_QC_Snapshot 使用)
 * @param image [in] 图像结构体
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_ReleaseImage(VmrQrCameraImageData* image);

/**
 * @brief 关闭设备
 * @param handle [in] 句柄
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_CloseDevice(VmrQrCameraHandle handle);

/**
 * @brief 释放句柄
 * @param handle [in] 句柄
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_ReleaseHandle(VmrQrCameraHandle handle);

/**
 * @brief 获取版本信息
 * @param version [out] 版本信息
 * @return VmrQrCameraExc
 */
VmrQrCameraExc VMR_QC_GetVersion(VmrQrCameraVersion* version);

#ifdef __cplusplus
}
#endif

#endif  // VMR_QRCAMERA_API_H
