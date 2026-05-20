#include "qr_code_identify_node.h"

#include <sstream>

namespace qr_code_identify
{

QrCodeIdentifyNode::QrCodeIdentifyNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), config_(QrCameraConfig()), config_loaded_(false), sdk_client_(&result_buffer_)
{
    std::string error;
    config_loaded_ = ReadCameraConfig(private_nh_, &config_, &error);
    if (!config_loaded_) {
        ROS_ERROR("Failed to read qr_code_identify params: %s", error.c_str());
        config_.lx = DefaultLxCameraParams();
        config_.runtime = DefaultRuntimeOptions();
    }

    qr_data_pub_ = nh_.advertise<common_msgs::qr_data>("qr_data", 1);
    update_params_service_ =
        nh_.advertiseService(config_.runtime.update_service_name, &QrCodeIdentifyNode::UpdateParamsCallback, this);

    const double publish_period = 1.0 / config_.runtime.publish_rate_hz;
    publish_timer_ = nh_.createTimer(ros::Duration(publish_period), &QrCodeIdentifyNode::PublishTimerCallback, this);

    reconnect_timer_ = nh_.createTimer(ros::Duration(config_.runtime.reconnect_interval_sec),
                                       &QrCodeIdentifyNode::ReconnectTimerCallback, this);

    status_timer_ = nh_.createTimer(ros::Duration(1.0), &QrCodeIdentifyNode::StatusTimerCallback, this);
}

bool QrCodeIdentifyNode::Start()
{
    if (!config_loaded_) {
        return false;
    }

    StartCamera("startup");
    return true;
}

bool QrCodeIdentifyNode::StartCamera(const std::string& reason)
{
    std::string error;
    if (sdk_client_.Start(config_, &error)) {
        return true;
    }

    ROS_WARN("Lanxin QR camera start failed during %s: %s", reason.c_str(), error.c_str());
    return false;
}

bool QrCodeIdentifyNode::RestartCamera(const std::string& reason, std::string* error)
{
    ROS_INFO("Restarting Lanxin QR camera: %s", reason.c_str());
    if (sdk_client_.Restart(config_, error)) {
        return true;
    }

    if (error && !error->empty()) {
        ROS_WARN("Lanxin QR camera restart failed: %s", error->c_str());
    }
    return false;
}

void QrCodeIdentifyNode::PublishTimerCallback(const ros::TimerEvent& event)
{
    (void)event;

    common_msgs::qr_data msg;
    msg.qr_datas.push_back(BuildPrimaryQrData());

    if (config_.runtime.publish_placeholder_up) {
        msg.qr_datas.push_back(BuildInvalidQrData(config_.runtime.placeholder_camera_id));
    }

    qr_data_pub_.publish(msg);
}

void QrCodeIdentifyNode::ReconnectTimerCallback(const ros::TimerEvent& event)
{
    (void)event;
    if (!config_loaded_ || !config_.runtime.auto_reconnect || sdk_client_.IsRunning()) {
        return;
    }

    StartCamera("auto reconnect");
}

void QrCodeIdentifyNode::StatusTimerCallback(const ros::TimerEvent& event)
{
    (void)event;

    const SdkStats stats = sdk_client_.TakeStats();

    QrDetection latest;
    const bool has_recent = result_buffer_.Latest(config_.runtime.stale_timeout_sec, &latest);

    ROS_INFO(
        "lanxin_qr running=%d image_fps=%lu detection_fps=%lu null=%lu "
        "decode_fail=%lu recent=%d valid=%d id=%u x=%.4f y=%.4f theta=%.2f "
        "raw=%s",
        sdk_client_.IsRunning() ? 1 : 0, static_cast<unsigned long>(stats.image_count),
        static_cast<unsigned long>(stats.detection_count), static_cast<unsigned long>(stats.null_detection_count),
        static_cast<unsigned long>(stats.decode_failure_count), has_recent ? 1 : 0,
        (has_recent && latest.valid) ? 1 : 0, (has_recent && latest.valid) ? latest.value : 0,
        (has_recent && latest.valid) ? latest.x_offset : 0.0f, (has_recent && latest.valid) ? latest.y_offset : 0.0f,
        (has_recent && latest.valid) ? latest.theta_offset : 0.0f, has_recent ? latest.raw_code.c_str() : "");
}

bool QrCodeIdentifyNode::UpdateParamsCallback(common_msgs::SetLxCameraParams::Request& request,
                                              common_msgs::SetLxCameraParams::Response& response)
{
    std::string error;

  if (!request.flag_update) {
    response.success = true;
    response.message = "read current qr_code_identify params success";
        FillCurrentParams(&response);
    return true;
  }

  // 更新顺序不能随意调整：先持久化，再写参数服务器，最后重启相机。
  // 这样 yaml 写失败时不会污染当前运行参数。
  if (!ValidateLxCameraParams(request.new_lx_param, &error)) {
        response.success = false;
        response.message = error;
        FillCurrentParams(&response);
        return true;
    }

    if (!WriteLxCameraParamsToDefaultFile(request.new_lx_param, &error)) {
        response.success = false;
        response.message = "write config_cameras.yaml failed: " + error;
        FillCurrentParams(&response);
        return true;
    }

    if (!WriteLxCameraParamsToServer(private_nh_, request.new_lx_param, &error)) {
        response.success = false;
        response.message = "write params to server failed: " + error;
        FillCurrentParams(&response);
        return true;
    }

    config_.lx = request.new_lx_param;

    if (!RestartCamera("params updated by service", &error)) {
        response.success = false;
        response.message = "params updated, but camera restart failed: " + error;
        FillCurrentParams(&response);
        return true;
    }

    response.success = true;
    response.message = "update qr_code_identify params success";
    FillCurrentParams(&response);
    return true;
}

common_msgs::one_qr_data QrCodeIdentifyNode::BuildPrimaryQrData() const
{
    QrDetection detection;
    const bool has_recent = result_buffer_.Latest(config_.runtime.stale_timeout_sec, &detection);
    if (!has_recent || !detection.valid) {
        return BuildInvalidQrData(config_.runtime.camera_id);
    }

    common_msgs::one_qr_data data;
    data.type_qr = static_cast<int8_t>(config_.runtime.qr_type);
    data.id_qr = static_cast<int8_t>(config_.runtime.camera_id);
    data.valid_qr = 1;
    data.value_qr = detection.value;
    data.x_offset = detection.x_offset;
    data.y_offset = detection.y_offset;
    data.theta_offset = detection.theta_offset;
    return data;
}

common_msgs::one_qr_data QrCodeIdentifyNode::BuildInvalidQrData(int camera_id) const
{
    common_msgs::one_qr_data data;
    data.type_qr = static_cast<int8_t>(config_.runtime.qr_type);
    data.id_qr = static_cast<int8_t>(camera_id);
    data.valid_qr = 0;
    data.value_qr = 0;
    data.x_offset = 0.0f;
    data.y_offset = 0.0f;
    data.theta_offset = 0.0f;
    return data;
}

void QrCodeIdentifyNode::FillCurrentParams(common_msgs::SetLxCameraParams::Response* response) const
{
    if (response) {
        response->current_lx_param = config_.lx;
    }
}

}  // namespace qr_code_identify
