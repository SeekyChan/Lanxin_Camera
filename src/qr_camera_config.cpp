#include "qr_camera_config.h"

#include <cctype>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <type_traits>

#include <ros/package.h>

namespace qr_code_identify
{
namespace
{

template <typename T>
bool CheckRange(const std::string& name, const T& value, const T& min_value, const T& max_value, std::string* error)
{
    if (value >= min_value && value <= max_value) {
        return true;
    }

    std::ostringstream oss;
    oss << "param '" << name << "' out of range: expected [" << min_value << ", " << max_value << "], got " << value;
    if (error) {
        *error = oss.str();
    }
    return false;
}

bool CheckNotBlank(const std::string& name, const std::string& value, std::string* error)
{
    for (std::string::const_iterator it = value.begin(); it != value.end(); ++it) {
        if (!std::isspace(static_cast<unsigned char>(*it))) {
            return true;
        }
    }

    if (error) {
        *error = "param '" + name + "' must not be empty";
    }
    return false;
}

bool CheckEncodeType(const std::string& name, int value, std::string* error)
{
    if (value == 1 || value == 5) {
        return true;
    }

    std::ostringstream oss;
    oss << "param '" << name << "' only supports 1(Lanxin) or 5(Pepperl+Fuchs), got " << value;
    if (error) {
        *error = oss.str();
    }
    return false;
}

template <typename T>
bool IsBoolFlagTrue(T value)
{
    return static_cast<int>(value) == 1;
}

template <typename T>
void NormalizeBoolFlag(T* value)
{
    if (value) {
        *value = IsBoolFlagTrue(*value);
    }
}

bool ValidateOneCameraParams(const common_msgs::config_lx_camera& params, const std::string& prefix, bool up,
                             std::string* error)
{
    const int exposure = up ? params.up_qr_camera_param_exposure : params.down_qr_camera_param_exposure;
    const int gain = up ? params.up_qr_camera_param_gain : params.down_qr_camera_param_gain;
    const int led = up ? params.up_qr_camera_param_led_brightness : params.down_qr_camera_param_led_brightness;
    const int rows = up ? params.up_qr_code_param_rows : params.down_qr_code_param_rows;
    const int cols = up ? params.up_qr_code_param_cols : params.down_qr_code_param_cols;
    const double qr_size = up ? params.up_qr_code_param_qr_size : params.down_qr_code_param_qr_size;
    const double qr_interval = up ? params.up_qr_code_param_qr_interval : params.down_qr_code_param_qr_interval;
    const double radius_big = up ? params.up_qr_code_param_radius_big : params.down_qr_code_param_radius_big;
    const double radius_small = up ? params.up_qr_code_param_radius_small : params.down_qr_code_param_radius_small;
    const double circle_dis = up ? params.up_qr_code_param_circle_dis : params.down_qr_code_param_circle_dis;
    const double point_height = up ? params.up_qr_code_param_point_height : params.down_qr_code_param_point_height;
    const double big_circle_width =
        up ? params.up_qr_code_param_big_circle_width : params.down_qr_code_param_big_circle_width;
    const int qr_need = up ? params.up_qr_code_param_qr_need : params.down_qr_code_param_qr_need;
    const int dm_symbol = up ? params.up_qr_code_param_dm_symbol : params.down_qr_code_param_dm_symbol;
    const double decode_timeout =
        up ? params.up_qr_code_param_decode_timeout : params.down_qr_code_param_decode_timeout;
    const int encode_type = up ? params.up_qr_code_param_encode_type : params.down_qr_code_param_encode_type;

    return CheckRange(prefix + "_qr_camera_param_exposure", exposure, 1, 2000, error) &&
           CheckRange(prefix + "_qr_camera_param_gain", gain, 1, 2000, error) &&
           CheckRange(prefix + "_qr_camera_param_led_brightness", led, 0, 100, error) &&
           CheckRange(prefix + "_qr_code_param_rows", rows, 1, 20, error) &&
           CheckRange(prefix + "_qr_code_param_cols", cols, 1, 20, error) &&
           CheckRange(prefix + "_qr_code_param_qr_size", qr_size, 0.001, 0.1, error) &&
           CheckRange(prefix + "_qr_code_param_qr_interval", qr_interval, 0.0, 0.1, error) &&
           CheckRange(prefix + "_qr_code_param_radius_big", radius_big, 0.0, 0.2, error) &&
           CheckRange(prefix + "_qr_code_param_radius_small", radius_small, 0.0, 0.2, error) &&
           CheckRange(prefix + "_qr_code_param_circle_dis", circle_dis, 0.0, 0.2, error) &&
           CheckRange(prefix + "_qr_code_param_point_height", point_height, 0.001, 0.2, error) &&
           CheckRange(prefix + "_qr_code_param_big_circle_width", big_circle_width, 0.0, 0.2, error) &&
           CheckRange(prefix + "_qr_code_param_qr_need", qr_need, 1, 20, error) &&
           CheckRange(prefix + "_qr_code_param_dm_symbol", dm_symbol, 1, 3, error) &&
           CheckRange(prefix + "_qr_code_param_decode_timeout", decode_timeout, 1.0, 200.0, error) &&
           CheckEncodeType(prefix + "_qr_code_param_encode_type", encode_type, error);
}

void ReadParam(ros::NodeHandle& nh, const std::string& name, std::string* value)
{
    std::string temp = *value;
    if (nh.getParam(name, temp)) {
        *value = temp;
    }
}

void ReadParam(ros::NodeHandle& nh, const std::string& name, bool* value)
{
    bool temp = *value;
    if (nh.getParam(name, temp)) {
        *value = temp;
    }
}

void ReadParam(ros::NodeHandle& nh, const std::string& name, double* value)
{
    double temp = *value;
    if (nh.getParam(name, temp)) {
        *value = temp;
    }
}

void ReadParam(ros::NodeHandle& nh, const std::string& name, float* value)
{
    double temp = *value;
    if (nh.getParam(name, temp)) {
        *value = static_cast<float>(temp);
    }
}

template <typename T>
typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, void>::type
ReadParam(ros::NodeHandle& nh, const std::string& name, T* value)
{
    // 避免 int32_t/uint16_t 等别名触发模板推导错误。
    int temp = static_cast<int>(*value);
    if (nh.getParam(name, temp)) {
        *value = static_cast<T>(temp);
    }
}

void SetParam(ros::NodeHandle& nh, const std::string& name, const std::string& value)
{
    nh.setParam(name, value);
}

void SetParam(ros::NodeHandle& nh, const std::string& name, bool value)
{
    nh.setParam(name, value);
}

void SetParam(ros::NodeHandle& nh, const std::string& name, double value)
{
    nh.setParam(name, value);
}

void SetParam(ros::NodeHandle& nh, const std::string& name, float value)
{
    nh.setParam(name, static_cast<double>(value));
}

template <typename T>
typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, void>::type
SetParam(ros::NodeHandle& nh, const std::string& name, T value)
{
    nh.setParam(name, static_cast<int>(value));
}

template <typename T>
void WriteYamlValue(std::ostream& out, const std::string& name, const T& value)
{
    out << name << ": " << value << "\n";
}

void WriteYamlValue(std::ostream& out, const std::string& name, bool value)
{
    out << name << ": " << (value ? "true" : "false") << "\n";
}

void WriteYamlValue(std::ostream& out, const std::string& name, const std::string& value)
{
    out << name << ": " << value << "\n";
}

template <typename T>
void WriteYamlBoolFlag(std::ostream& out, const std::string& name, T value)
{
    out << name << ": " << (IsBoolFlagTrue(value) ? "true" : "false") << "\n";
}

bool DefaultConfigFilePath(std::string* path, std::string* error)
{
    // 使用 rospack 解析真实包路径，避免启动目录变化导致写错配置文件。
    const std::string package_path = ros::package::getPath("qr_code_identify_lanxin");
    if (package_path.empty()) {
        if (error) {
            *error = "cannot find package path for qr_code_identify_lanxin";
        }
        return false;
    }

    if (path) {
        *path = package_path + "/config/config_cameras.yaml";
    }
    return true;
}

void ReadLxCameraParams(ros::NodeHandle& nh, common_msgs::config_lx_camera* params)
{
    ReadParam(nh, "down_qr_camera_ip", &params->down_qr_camera_ip);
    ReadParam(nh, "down_qr_camera_param_exposure", &params->down_qr_camera_param_exposure);
    ReadParam(nh, "down_qr_camera_param_gain", &params->down_qr_camera_param_gain);
    ReadParam(nh, "down_qr_camera_param_led_brightness", &params->down_qr_camera_param_led_brightness);

    ReadParam(nh, "up_qr_camera_ip", &params->up_qr_camera_ip);
    ReadParam(nh, "up_qr_camera_param_exposure", &params->up_qr_camera_param_exposure);
    ReadParam(nh, "up_qr_camera_param_gain", &params->up_qr_camera_param_gain);
    ReadParam(nh, "up_qr_camera_param_led_brightness", &params->up_qr_camera_param_led_brightness);

    ReadParam(nh, "down_qr_code_param_rows", &params->down_qr_code_param_rows);
    ReadParam(nh, "down_qr_code_param_cols", &params->down_qr_code_param_cols);
    ReadParam(nh, "down_qr_code_param_qr_size", &params->down_qr_code_param_qr_size);
    ReadParam(nh, "down_qr_code_param_qr_interval", &params->down_qr_code_param_qr_interval);
    ReadParam(nh, "down_qr_code_param_radius_big", &params->down_qr_code_param_radius_big);
    ReadParam(nh, "down_qr_code_param_radius_small", &params->down_qr_code_param_radius_small);
    ReadParam(nh, "down_qr_code_param_circle_dis", &params->down_qr_code_param_circle_dis);
    ReadParam(nh, "down_qr_code_param_point_height", &params->down_qr_code_param_point_height);
    ReadParam(nh, "down_qr_code_param_big_circle_width", &params->down_qr_code_param_big_circle_width);
    ReadParam(nh, "down_qr_code_param_qr_need", &params->down_qr_code_param_qr_need);
    ReadParam(nh, "down_qr_code_param_dm_symbol", &params->down_qr_code_param_dm_symbol);
    ReadParam(nh, "down_qr_code_param_decode_timeout", &params->down_qr_code_param_decode_timeout);
    ReadParam(nh, "down_qr_code_param_big_circle_flag", &params->down_qr_code_param_big_circle_flag);
    ReadParam(nh, "down_qr_code_param_small_circle_flag", &params->down_qr_code_param_small_circle_flag);
    ReadParam(nh, "down_qr_code_param_encode_type", &params->down_qr_code_param_encode_type);
    ReadParam(nh, "down_qr_code_param_save_video", &params->down_qr_code_param_save_video);

    ReadParam(nh, "up_qr_code_param_rows", &params->up_qr_code_param_rows);
    ReadParam(nh, "up_qr_code_param_cols", &params->up_qr_code_param_cols);
    ReadParam(nh, "up_qr_code_param_qr_size", &params->up_qr_code_param_qr_size);
    ReadParam(nh, "up_qr_code_param_qr_interval", &params->up_qr_code_param_qr_interval);
    ReadParam(nh, "up_qr_code_param_radius_big", &params->up_qr_code_param_radius_big);
    ReadParam(nh, "up_qr_code_param_radius_small", &params->up_qr_code_param_radius_small);
    ReadParam(nh, "up_qr_code_param_circle_dis", &params->up_qr_code_param_circle_dis);
    ReadParam(nh, "up_qr_code_param_point_height", &params->up_qr_code_param_point_height);
    ReadParam(nh, "up_qr_code_param_big_circle_width", &params->up_qr_code_param_big_circle_width);
    ReadParam(nh, "up_qr_code_param_qr_need", &params->up_qr_code_param_qr_need);
    ReadParam(nh, "up_qr_code_param_dm_symbol", &params->up_qr_code_param_dm_symbol);
    ReadParam(nh, "up_qr_code_param_decode_timeout", &params->up_qr_code_param_decode_timeout);
    ReadParam(nh, "up_qr_code_param_big_circle_flag", &params->up_qr_code_param_big_circle_flag);
    ReadParam(nh, "up_qr_code_param_small_circle_flag", &params->up_qr_code_param_small_circle_flag);
    ReadParam(nh, "up_qr_code_param_encode_type", &params->up_qr_code_param_encode_type);
    ReadParam(nh, "up_qr_code_param_save_video", &params->up_qr_code_param_save_video);
}

void ReadRuntimeOptions(ros::NodeHandle& nh, RuntimeOptions* options)
{
    ReadParam(nh, "qr_type", &options->qr_type);
    ReadParam(nh, "camera_id", &options->camera_id);
    ReadParam(nh, "placeholder_camera_id", &options->placeholder_camera_id);
    ReadParam(nh, "publish_placeholder_up", &options->publish_placeholder_up);
    ReadParam(nh, "publish_rate_hz", &options->publish_rate_hz);
    ReadParam(nh, "stale_timeout_sec", &options->stale_timeout_sec);
    ReadParam(nh, "auto_reconnect", &options->auto_reconnect);
    ReadParam(nh, "reconnect_interval_sec", &options->reconnect_interval_sec);
    ReadParam(nh, "update_service_name", &options->update_service_name);
}

}  // namespace

common_msgs::config_lx_camera DefaultLxCameraParams()
{
    common_msgs::config_lx_camera params;

    params.down_qr_camera_ip = "192.168.100.6";
    params.down_qr_camera_param_exposure = 300;
    params.down_qr_camera_param_gain = 250;
    params.down_qr_camera_param_led_brightness = 100;

    params.up_qr_camera_ip = "192.168.10.106";
    params.up_qr_camera_param_exposure = 300;
    params.up_qr_camera_param_gain = 250;
    params.up_qr_camera_param_led_brightness = 100;

    params.down_qr_code_param_rows = 4;
    params.down_qr_code_param_cols = 4;
    params.down_qr_code_param_qr_size = 0.015;
    params.down_qr_code_param_qr_interval = 0.005;
    params.down_qr_code_param_radius_big = 0.016;
    params.down_qr_code_param_radius_small = 0.003;
    params.down_qr_code_param_circle_dis = 0.0425;
    params.down_qr_code_param_point_height = 0.1;
    params.down_qr_code_param_big_circle_width = 0.003;
    params.down_qr_code_param_qr_need = 3;
    params.down_qr_code_param_dm_symbol = 1;
    params.down_qr_code_param_decode_timeout = 25.0;
    params.down_qr_code_param_big_circle_flag = false;
    params.down_qr_code_param_small_circle_flag = false;
    params.down_qr_code_param_encode_type = 5;
    params.down_qr_code_param_save_video = false;

    params.up_qr_code_param_rows = 4;
    params.up_qr_code_param_cols = 4;
    params.up_qr_code_param_qr_size = 0.018;
    params.up_qr_code_param_qr_interval = 0.005;
    params.up_qr_code_param_radius_big = 0.016;
    params.up_qr_code_param_radius_small = 0.003;
    params.up_qr_code_param_circle_dis = 0.0425;
    params.up_qr_code_param_point_height = 0.1;
    params.up_qr_code_param_big_circle_width = 0.003;
    params.up_qr_code_param_qr_need = 3;
    params.up_qr_code_param_dm_symbol = 3;
    params.up_qr_code_param_decode_timeout = 25.0;
    params.up_qr_code_param_big_circle_flag = false;
    params.up_qr_code_param_small_circle_flag = false;
    params.up_qr_code_param_encode_type = 5;
    params.up_qr_code_param_save_video = false;

    return params;
}

RuntimeOptions DefaultRuntimeOptions()
{
    RuntimeOptions options;
    options.qr_type = 2;                    // common_msgs/one_qr_data.type_qr: 2 表示蓝芯相机
    options.camera_id = 1;                  // 当前真实接入的下视相机 id_qr
    options.placeholder_camera_id = 2;      // 为兼容旧消息格式保留的上视相机占位 id_qr
    options.publish_placeholder_up = true;  // 是否额外发布一个无效的上视占位二维码数据
    options.publish_rate_hz = 100.0;        // qr_data topic 的发布频率
    options.stale_timeout_sec = 0.06;        // 超过该时间没有新识别结果时，对外发布 valid=0
    options.auto_reconnect = true;          // 相机启动失败或未运行时，是否由定时器自动重连
    options.reconnect_interval_sec = 3.0;   // 自动重连尝试间隔
    options.update_service_name = "update_qr_code_identify_lanxin_params";
    return options;
}

void NormalizeLxCameraBooleanFlags(common_msgs::config_lx_camera* params)
{
    if (!params) {
        return;
    }

    // ROS bool 字段在 C++ 中可能是 uint8_t；这里只认 1/true 为 true，其它值统一压成 false。
    NormalizeBoolFlag(&params->down_qr_code_param_big_circle_flag);
    NormalizeBoolFlag(&params->down_qr_code_param_small_circle_flag);
    NormalizeBoolFlag(&params->down_qr_code_param_save_video);
    NormalizeBoolFlag(&params->up_qr_code_param_big_circle_flag);
    NormalizeBoolFlag(&params->up_qr_code_param_small_circle_flag);
    NormalizeBoolFlag(&params->up_qr_code_param_save_video);
}

bool ValidateLxCameraParams(const common_msgs::config_lx_camera& params, std::string* error)
{
    return CheckNotBlank("down_qr_camera_ip", params.down_qr_camera_ip, error) &&
           CheckNotBlank("up_qr_camera_ip", params.up_qr_camera_ip, error) &&
           ValidateOneCameraParams(params, "down", false, error) && ValidateOneCameraParams(params, "up", true, error);
}

bool ReadCameraConfig(ros::NodeHandle& private_nh, QrCameraConfig* config, std::string* error)
{
    if (!config) {
        if (error) {
            *error = "config pointer is null";
        }
        return false;
    }

    config->lx = DefaultLxCameraParams();
    config->runtime = DefaultRuntimeOptions();
    ReadLxCameraParams(private_nh, &config->lx);
    NormalizeLxCameraBooleanFlags(&config->lx);
    ReadRuntimeOptions(private_nh, &config->runtime);

    if (!ValidateLxCameraParams(config->lx, error)) {
        return false;
    }
    if (!CheckRange("publish_rate_hz", config->runtime.publish_rate_hz, 1.0, 500.0, error) ||
        !CheckRange("stale_timeout_sec", config->runtime.stale_timeout_sec, 0.001, 10.0, error) ||
        !CheckRange("reconnect_interval_sec", config->runtime.reconnect_interval_sec, 0.1, 60.0, error) ||
        !CheckRange("camera_id", config->runtime.camera_id, 1, 127, error) ||
        !CheckRange("placeholder_camera_id", config->runtime.placeholder_camera_id, 1, 127, error) ||
        !CheckRange("qr_type", config->runtime.qr_type, 1, 127, error)) {
        return false;
    }

    return true;
}

bool WriteLxCameraParamsToServer(ros::NodeHandle& private_nh, const common_msgs::config_lx_camera& params,
                                 std::string* error)
{
    common_msgs::config_lx_camera normalized = params;
    NormalizeLxCameraBooleanFlags(&normalized);

    if (!ValidateLxCameraParams(normalized, error)) {
        return false;
    }

    SetParam(private_nh, "down_qr_camera_ip", normalized.down_qr_camera_ip);
    SetParam(private_nh, "down_qr_camera_param_exposure", normalized.down_qr_camera_param_exposure);
    SetParam(private_nh, "down_qr_camera_param_gain", normalized.down_qr_camera_param_gain);
    SetParam(private_nh, "down_qr_camera_param_led_brightness", normalized.down_qr_camera_param_led_brightness);

    SetParam(private_nh, "up_qr_camera_ip", normalized.up_qr_camera_ip);
    SetParam(private_nh, "up_qr_camera_param_exposure", normalized.up_qr_camera_param_exposure);
    SetParam(private_nh, "up_qr_camera_param_gain", normalized.up_qr_camera_param_gain);
    SetParam(private_nh, "up_qr_camera_param_led_brightness", normalized.up_qr_camera_param_led_brightness);

    SetParam(private_nh, "down_qr_code_param_rows", normalized.down_qr_code_param_rows);
    SetParam(private_nh, "down_qr_code_param_cols", normalized.down_qr_code_param_cols);
    SetParam(private_nh, "down_qr_code_param_qr_size", normalized.down_qr_code_param_qr_size);
    SetParam(private_nh, "down_qr_code_param_qr_interval", normalized.down_qr_code_param_qr_interval);
    SetParam(private_nh, "down_qr_code_param_radius_big", normalized.down_qr_code_param_radius_big);
    SetParam(private_nh, "down_qr_code_param_radius_small", normalized.down_qr_code_param_radius_small);
    SetParam(private_nh, "down_qr_code_param_circle_dis", normalized.down_qr_code_param_circle_dis);
    SetParam(private_nh, "down_qr_code_param_point_height", normalized.down_qr_code_param_point_height);
    SetParam(private_nh, "down_qr_code_param_big_circle_width", normalized.down_qr_code_param_big_circle_width);
    SetParam(private_nh, "down_qr_code_param_qr_need", normalized.down_qr_code_param_qr_need);
    SetParam(private_nh, "down_qr_code_param_dm_symbol", normalized.down_qr_code_param_dm_symbol);
    SetParam(private_nh, "down_qr_code_param_decode_timeout", normalized.down_qr_code_param_decode_timeout);
    SetParam(private_nh, "down_qr_code_param_big_circle_flag", normalized.down_qr_code_param_big_circle_flag);
    SetParam(private_nh, "down_qr_code_param_small_circle_flag", normalized.down_qr_code_param_small_circle_flag);
    SetParam(private_nh, "down_qr_code_param_encode_type", normalized.down_qr_code_param_encode_type);
    SetParam(private_nh, "down_qr_code_param_save_video", normalized.down_qr_code_param_save_video);

    SetParam(private_nh, "up_qr_code_param_rows", normalized.up_qr_code_param_rows);
    SetParam(private_nh, "up_qr_code_param_cols", normalized.up_qr_code_param_cols);
    SetParam(private_nh, "up_qr_code_param_qr_size", normalized.up_qr_code_param_qr_size);
    SetParam(private_nh, "up_qr_code_param_qr_interval", normalized.up_qr_code_param_qr_interval);
    SetParam(private_nh, "up_qr_code_param_radius_big", normalized.up_qr_code_param_radius_big);
    SetParam(private_nh, "up_qr_code_param_radius_small", normalized.up_qr_code_param_radius_small);
    SetParam(private_nh, "up_qr_code_param_circle_dis", normalized.up_qr_code_param_circle_dis);
    SetParam(private_nh, "up_qr_code_param_point_height", normalized.up_qr_code_param_point_height);
    SetParam(private_nh, "up_qr_code_param_big_circle_width", normalized.up_qr_code_param_big_circle_width);
    SetParam(private_nh, "up_qr_code_param_qr_need", normalized.up_qr_code_param_qr_need);
    SetParam(private_nh, "up_qr_code_param_dm_symbol", normalized.up_qr_code_param_dm_symbol);
    SetParam(private_nh, "up_qr_code_param_decode_timeout", normalized.up_qr_code_param_decode_timeout);
    SetParam(private_nh, "up_qr_code_param_big_circle_flag", normalized.up_qr_code_param_big_circle_flag);
    SetParam(private_nh, "up_qr_code_param_small_circle_flag", normalized.up_qr_code_param_small_circle_flag);
    SetParam(private_nh, "up_qr_code_param_encode_type", normalized.up_qr_code_param_encode_type);
    SetParam(private_nh, "up_qr_code_param_save_video", normalized.up_qr_code_param_save_video);

    return true;
}

bool WriteLxCameraParamsToDefaultFile(const common_msgs::config_lx_camera& params, std::string* error)
{
    common_msgs::config_lx_camera normalized = params;
    NormalizeLxCameraBooleanFlags(&normalized);

    if (!ValidateLxCameraParams(normalized, error)) {
        return false;
    }

    std::string path;
    if (!DefaultConfigFilePath(&path, error)) {
        return false;
    }

    const std::string temp_path = path + ".tmp";
    {
        // 只写旧版 config_lx_camera 字段；wrapper 运行参数不要混入 config_cameras.yaml。
        std::ofstream out(temp_path.c_str(), std::ios::out | std::ios::trunc);
        if (!out.is_open()) {
            if (error) {
                *error = "cannot open temp config file for write: " + temp_path;
            }
            return false;
        }

        out << std::setprecision(10);
        out << "# camera params\n";
        WriteYamlValue(out, "down_qr_camera_ip", normalized.down_qr_camera_ip);
        WriteYamlValue(out, "down_qr_camera_param_exposure", normalized.down_qr_camera_param_exposure);
        WriteYamlValue(out, "down_qr_camera_param_gain", normalized.down_qr_camera_param_gain);
        WriteYamlValue(out, "down_qr_camera_param_led_brightness", normalized.down_qr_camera_param_led_brightness);
        out << "\n";

        WriteYamlValue(out, "up_qr_camera_ip", normalized.up_qr_camera_ip);
        WriteYamlValue(out, "up_qr_camera_param_exposure", normalized.up_qr_camera_param_exposure);
        WriteYamlValue(out, "up_qr_camera_param_gain", normalized.up_qr_camera_param_gain);
        WriteYamlValue(out, "up_qr_camera_param_led_brightness", normalized.up_qr_camera_param_led_brightness);
        out << "\n";

        out << "# QR detection params\n";
        WriteYamlValue(out, "down_qr_code_param_rows", normalized.down_qr_code_param_rows);
        WriteYamlValue(out, "down_qr_code_param_cols", normalized.down_qr_code_param_cols);
        WriteYamlValue(out, "down_qr_code_param_qr_size", normalized.down_qr_code_param_qr_size);
        WriteYamlValue(out, "down_qr_code_param_qr_interval", normalized.down_qr_code_param_qr_interval);
        WriteYamlValue(out, "down_qr_code_param_radius_big", normalized.down_qr_code_param_radius_big);
        WriteYamlValue(out, "down_qr_code_param_radius_small", normalized.down_qr_code_param_radius_small);
        WriteYamlValue(out, "down_qr_code_param_circle_dis", normalized.down_qr_code_param_circle_dis);
        WriteYamlValue(out, "down_qr_code_param_point_height", normalized.down_qr_code_param_point_height);
        WriteYamlValue(out, "down_qr_code_param_big_circle_width", normalized.down_qr_code_param_big_circle_width);
        WriteYamlValue(out, "down_qr_code_param_qr_need", normalized.down_qr_code_param_qr_need);
        WriteYamlValue(out, "down_qr_code_param_dm_symbol", normalized.down_qr_code_param_dm_symbol);
        WriteYamlValue(out, "down_qr_code_param_decode_timeout", normalized.down_qr_code_param_decode_timeout);
        WriteYamlBoolFlag(out, "down_qr_code_param_big_circle_flag", normalized.down_qr_code_param_big_circle_flag);
        WriteYamlBoolFlag(out, "down_qr_code_param_small_circle_flag", normalized.down_qr_code_param_small_circle_flag);
        WriteYamlValue(out, "down_qr_code_param_encode_type", normalized.down_qr_code_param_encode_type);
        WriteYamlBoolFlag(out, "down_qr_code_param_save_video", normalized.down_qr_code_param_save_video);
        out << "\n";

        WriteYamlValue(out, "up_qr_code_param_rows", normalized.up_qr_code_param_rows);
        WriteYamlValue(out, "up_qr_code_param_cols", normalized.up_qr_code_param_cols);
        WriteYamlValue(out, "up_qr_code_param_qr_size", normalized.up_qr_code_param_qr_size);
        WriteYamlValue(out, "up_qr_code_param_qr_interval", normalized.up_qr_code_param_qr_interval);
        WriteYamlValue(out, "up_qr_code_param_radius_big", normalized.up_qr_code_param_radius_big);
        WriteYamlValue(out, "up_qr_code_param_radius_small", normalized.up_qr_code_param_radius_small);
        WriteYamlValue(out, "up_qr_code_param_circle_dis", normalized.up_qr_code_param_circle_dis);
        WriteYamlValue(out, "up_qr_code_param_point_height", normalized.up_qr_code_param_point_height);
        WriteYamlValue(out, "up_qr_code_param_big_circle_width", normalized.up_qr_code_param_big_circle_width);
        WriteYamlValue(out, "up_qr_code_param_qr_need", normalized.up_qr_code_param_qr_need);
        WriteYamlValue(out, "up_qr_code_param_dm_symbol", normalized.up_qr_code_param_dm_symbol);
        WriteYamlValue(out, "up_qr_code_param_decode_timeout", normalized.up_qr_code_param_decode_timeout);
        WriteYamlBoolFlag(out, "up_qr_code_param_big_circle_flag", normalized.up_qr_code_param_big_circle_flag);
        WriteYamlBoolFlag(out, "up_qr_code_param_small_circle_flag", normalized.up_qr_code_param_small_circle_flag);
        WriteYamlValue(out, "up_qr_code_param_encode_type", normalized.up_qr_code_param_encode_type);
        WriteYamlBoolFlag(out, "up_qr_code_param_save_video", normalized.up_qr_code_param_save_video);

        out.flush();
        if (!out.good()) {
            if (error) {
                *error = "failed while writing temp config file: " + temp_path;
            }
            std::remove(temp_path.c_str());
            return false;
        }
    }

    // 先写临时文件再原子替换，降低异常退出时留下半截 yaml 的概率。
    if (std::rename(temp_path.c_str(), path.c_str()) != 0) {
        if (error) {
            *error = "failed to replace config file: " + path;
        }
        std::remove(temp_path.c_str());
        return false;
    }

    return true;
}

}  // namespace qr_code_identify
