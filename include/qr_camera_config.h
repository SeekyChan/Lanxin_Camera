#ifndef QR_CODE_IDENTIFY_LANXIN_QR_CAMERA_CONFIG_H
#define QR_CODE_IDENTIFY_LANXIN_QR_CAMERA_CONFIG_H

#include <string>

#include <ros/ros.h>

#include "common_msgs/config_lx_camera.h"

namespace qr_code_identify
{

struct RuntimeOptions {
    int qr_type;
    int camera_id;
    int placeholder_camera_id;
    bool publish_placeholder_up;
    double publish_rate_hz;
    double stale_timeout_sec;
    bool auto_reconnect;
    double reconnect_interval_sec;
    std::string update_service_name;
};

struct QrCameraConfig {
    common_msgs::config_lx_camera lx;
    RuntimeOptions runtime;
};

common_msgs::config_lx_camera DefaultLxCameraParams();
RuntimeOptions DefaultRuntimeOptions();

void NormalizeLxCameraBooleanFlags(common_msgs::config_lx_camera* params);

bool ValidateLxCameraParams(const common_msgs::config_lx_camera& params, std::string* error);

bool ReadCameraConfig(ros::NodeHandle& private_nh, QrCameraConfig* config, std::string* error);

bool WriteLxCameraParamsToServer(ros::NodeHandle& private_nh, const common_msgs::config_lx_camera& params,
                                 std::string* error);

bool WriteLxCameraParamsToDefaultFile(const common_msgs::config_lx_camera& params, std::string* error);

}  // namespace qr_code_identify

#endif  // QR_CODE_IDENTIFY_LANXIN_QR_CAMERA_CONFIG_H
