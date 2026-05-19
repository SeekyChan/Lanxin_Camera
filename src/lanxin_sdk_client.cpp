#include "qr_code_identify_lanxin/lanxin_sdk_client.h"

#include <string.h>

#include <sstream>

#include <ros/ros.h>

#include "qr_code_identify_lanxin/qr_code_parser.h"

namespace qr_code_identify {
namespace {

std::string SdkErrorToString(VmrQrCameraExc code) {
  switch (code) {
    case VmrQrCameraNormal:
      return "normal";
    case VmrQrCameraHandleInvalid:
      return "invalid handle";
    case VmrQrCameraNotCapturing:
      return "not capturing";
    case VmrQrCameraAbnormalStop:
      return "abnormal stop";
    case VmrQrCameraTimeout:
      return "timeout";
    case VmrQrCameraFail:
      return "fail";
    case VmrQrCameraInvalidParam:
      return "invalid param";
    default:
      break;
  }

  std::ostringstream oss;
  oss << "unknown error " << static_cast<int>(code);
  return oss.str();
}

VmrQrCameraDetectionParams BuildDetectionParams(
    const common_msgs::config_lx_camera& params) {
  VmrQrCameraDetectionParams sdk_params;
  sdk_params.rows = params.down_qr_code_param_rows;
  sdk_params.cols = params.down_qr_code_param_cols;
  sdk_params.qr_size = params.down_qr_code_param_qr_size;
  sdk_params.qr_interval = params.down_qr_code_param_qr_interval;
  sdk_params.point_height = params.down_qr_code_param_point_height;
  sdk_params.qr_need = params.down_qr_code_param_qr_need;
  sdk_params.dm_symbol = params.down_qr_code_param_dm_symbol;
  sdk_params.decode_timeout =
      static_cast<float>(params.down_qr_code_param_decode_timeout);
  sdk_params.big_circle_flag = params.down_qr_code_param_big_circle_flag;
  sdk_params.radius_big = params.down_qr_code_param_radius_big;
  sdk_params.big_circle_width = params.down_qr_code_param_big_circle_width;
  sdk_params.small_circle_flag = params.down_qr_code_param_small_circle_flag;
  sdk_params.radius_small = params.down_qr_code_param_radius_small;
  sdk_params.circle_dis = params.down_qr_code_param_circle_dis;
  sdk_params.encode_type = params.down_qr_code_param_encode_type;
  sdk_params.save_video = params.down_qr_code_param_save_video;
  return sdk_params;
}

size_t BoundedStringLength(const char* text, size_t max_len) {
  if (!text) {
    return 0;
  }
  size_t len = 0;
  while (len < max_len && text[len] != '\0') {
    ++len;
  }
  return len;
}

}  // namespace

LanxinSdkClient::CallbackContext::CallbackContext()
    : result_buffer(NULL),
      accepting_callbacks(false),
      encode_type(1),
      image_count(0),
      detection_count(0),
      null_detection_count(0),
      decode_failure_count(0),
      session_id(0) {}

LanxinSdkClient::LanxinSdkClient(QrResultBuffer* result_buffer)
    : handle_(NULL), running_(false) {
  callback_context_.result_buffer = result_buffer;
}

LanxinSdkClient::~LanxinSdkClient() { Stop(); }

bool LanxinSdkClient::Start(const QrCameraConfig& config, std::string* error) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  return StartLocked(config, error);
}

bool LanxinSdkClient::Restart(const QrCameraConfig& config, std::string* error) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  StopLocked();
  return StartLocked(config, error);
}

void LanxinSdkClient::Stop() {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  StopLocked();
}

bool LanxinSdkClient::IsRunning() const {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  return running_;
}

SdkStats LanxinSdkClient::TakeStats() {
  SdkStats stats;
  stats.image_count = callback_context_.image_count.exchange(0);
  stats.detection_count = callback_context_.detection_count.exchange(0);
  stats.null_detection_count = callback_context_.null_detection_count.exchange(0);
  stats.decode_failure_count = callback_context_.decode_failure_count.exchange(0);
  return stats;
}

void LanxinSdkClient::OnImage(VmrQrCameraImageData* image, void* user_data) {
  CallbackContext* context = static_cast<CallbackContext*>(user_data);
  if (!context || !context->accepting_callbacks.load()) {
    return;
  }
  if (image) {
    context->image_count.fetch_add(1);
  }
}

void LanxinSdkClient::OnDetection(VmrQrCameraDetectionData* result,
                                  void* user_data) {
  CallbackContext* context = static_cast<CallbackContext*>(user_data);
  if (!context || !context->accepting_callbacks.load()) {
    return;
  }
  const uint64_t session_id = context->session_id.load();

  if (!result) {
    context->null_detection_count.fetch_add(1);
    return;
  }

  context->detection_count.fetch_add(1);

  const size_t code_len = BoundedStringLength(result->code, sizeof(result->code));
  const std::string raw_code(result->code, code_len);
  const ParsedQrCode parsed =
      ParseQrCode(raw_code, context->encode_type.load());

  if (!parsed.valid) {
    context->decode_failure_count.fetch_add(1);
  }

  QrDetection detection;
  detection.valid = parsed.valid;
  detection.value = parsed.valid ? parsed.value : 0;
  detection.x_offset = result->x;
  detection.y_offset = -result->y;
  detection.theta_offset = ConvertSdkThetaToDegrees(result->theta);
  detection.raw_code = raw_code;
  detection.extracted_code = parsed.extracted_text;
  detection.decode_error = parsed.error;
  detection.frame_count = result->frame_count;
  detection.sdk_timestamp_ns = result->timestamp_ns;
  detection.received_time = ros::Time::now();

  if (context->session_id.load() != session_id) {
    return;
  }

  if (context->result_buffer) {
    context->result_buffer->Update(detection);
  }
}

bool LanxinSdkClient::StartLocked(const QrCameraConfig& config, std::string* error) {
  if (running_) {
    if (error) {
      *error = "camera is already running";
    }
    return true;
  }

  callback_context_.accepting_callbacks.store(false);
  callback_context_.session_id.fetch_add(1);
  callback_context_.encode_type.store(config.lx.down_qr_code_param_encode_type);

  VmrQrCameraVersion version;
  if (VMR_QC_GetVersion(&version) == VmrQrCameraNormal) {
    ROS_INFO("Lanxin SDK version: driver=%s algo=%s", version.driver_version,
             version.algo_version);
  }

  VmrQrCameraExc ret = VMR_QC_CreateHandle(&handle_);
  if (!CheckSdkCall(ret, "create handle", error)) {
    handle_ = NULL;
    return false;
  }

  ret = VMR_QC_OpenDevice(handle_, config.lx.down_qr_camera_ip.c_str());
  if (!CheckSdkCall(ret, "open device " + config.lx.down_qr_camera_ip, error)) {
    StopLocked();
    return false;
  }

  ret = VMR_QC_SetDeviceParam(handle_, VmrQrCameraParam_Exposure,
                              config.lx.down_qr_camera_param_exposure);
  if (!CheckSdkCall(ret, "set exposure", error)) {
    StopLocked();
    return false;
  }

  ret = VMR_QC_SetDeviceParam(handle_, VmrQrCameraParam_Gain,
                              config.lx.down_qr_camera_param_gain);
  if (!CheckSdkCall(ret, "set gain", error)) {
    StopLocked();
    return false;
  }

  ret = VMR_QC_SetDeviceParam(handle_, VmrQrCameraParam_LedBrightness,
                              config.lx.down_qr_camera_param_led_brightness);
  if (!CheckSdkCall(ret, "set LED brightness", error)) {
    StopLocked();
    return false;
  }

  VmrQrCameraDetectionParams detection_params = BuildDetectionParams(config.lx);
  ret = VMR_QC_SetDetectionParams(handle_, &detection_params);
  if (!CheckSdkCall(ret, "set detection params", error)) {
    StopLocked();
    return false;
  }

  ret = VMR_QC_SetImageCallback(handle_, &LanxinSdkClient::OnImage,
                                &callback_context_);
  if (!CheckSdkCall(ret, "set image callback", error)) {
    StopLocked();
    return false;
  }

  ret = VMR_QC_SetDetectionCallback(handle_, &LanxinSdkClient::OnDetection,
                                    &callback_context_);
  if (!CheckSdkCall(ret, "set detection callback", error)) {
    StopLocked();
    return false;
  }

  callback_context_.accepting_callbacks.store(true);

  ret = VMR_QC_StartCapture(handle_);
  if (!CheckSdkCall(ret, "start capture", error)) {
    callback_context_.accepting_callbacks.store(false);
    StopLocked();
    return false;
  }

  running_ = true;
  ROS_INFO("Lanxin QR camera started: ip=%s exposure=%d gain=%d led=%d encode=%d",
           config.lx.down_qr_camera_ip.c_str(),
           config.lx.down_qr_camera_param_exposure,
           config.lx.down_qr_camera_param_gain,
           config.lx.down_qr_camera_param_led_brightness,
           config.lx.down_qr_code_param_encode_type);
  return true;
}

void LanxinSdkClient::StopLocked() {
  callback_context_.accepting_callbacks.store(false);
  callback_context_.session_id.fetch_add(1);

  if (!handle_) {
    running_ = false;
    return;
  }

  if (running_) {
    VMR_QC_StopCapture(handle_);
  }
  VMR_QC_CloseDevice(handle_);
  VMR_QC_ReleaseHandle(handle_);

  handle_ = NULL;
  running_ = false;
  if (callback_context_.result_buffer) {
    callback_context_.result_buffer->Clear();
  }
}

bool LanxinSdkClient::CheckSdkCall(VmrQrCameraExc code, const std::string& action,
                                   std::string* error) const {
  if (code == VmrQrCameraNormal) {
    return true;
  }

  std::ostringstream oss;
  oss << "SDK " << action << " failed: " << SdkErrorToString(code) << " ("
      << static_cast<int>(code) << ")";
  if (error) {
    *error = oss.str();
  }
  ROS_ERROR("%s", oss.str().c_str());
  return false;
}

}  // namespace qr_code_identify
