#include "qr_camera_config.h"

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>

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

template <typename T>
void SetParam(ros::NodeHandle& nh, const std::string& name, const T& value)
{
    nh.setParam(name, value);
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
    nh.param("down_qr_camera_ip", params->down_qr_camera_ip, params->down_qr_camera_ip);
    nh.param("down_qr_camera_param_exposure", params->down_qr_camera_param_exposure,
             params->down_qr_camera_param_exposure);
    nh.param("down_qr_camera_param_gain", params->down_qr_camera_param_gain, params->down_qr_camera_param_gain);
    nh.param("down_qr_camera_param_led_brightness", params->down_qr_camera_param_led_brightness,
             params->down_qr_camera_param_led_brightness);

    nh.param("up_qr_camera_ip", params->up_qr_camera_ip, params->up_qr_camera_ip);
    nh.param("up_qr_camera_param_exposure", params->up_qr_camera_param_exposure, params->up_qr_camera_param_exposure);
    nh.param("up_qr_camera_param_gain", params->up_qr_camera_param_gain, params->up_qr_camera_param_gain);
    nh.param("up_qr_camera_param_led_brightness", params->up_qr_camera_param_led_brightness,
             params->up_qr_camera_param_led_brightness);

    nh.param("down_qr_code_param_rows", params->down_qr_code_param_rows, params->down_qr_code_param_rows);
    nh.param("down_qr_code_param_cols", params->down_qr_code_param_cols, params->down_qr_code_param_cols);
    nh.param("down_qr_code_param_qr_size", params->down_qr_code_param_qr_size, params->down_qr_code_param_qr_size);
    nh.param("down_qr_code_param_qr_interval", params->down_qr_code_param_qr_interval,
             params->down_qr_code_param_qr_interval);
    nh.param("down_qr_code_param_radius_big", params->down_qr_code_param_radius_big,
             params->down_qr_code_param_radius_big);
    nh.param("down_qr_code_param_radius_small", params->down_qr_code_param_radius_small,
             params->down_qr_code_param_radius_small);
    nh.param("down_qr_code_param_circle_dis", params->down_qr_code_param_circle_dis,
             params->down_qr_code_param_circle_dis);
    nh.param("down_qr_code_param_point_height", params->down_qr_code_param_point_height,
             params->down_qr_code_param_point_height);
    nh.param("down_qr_code_param_big_circle_width", params->down_qr_code_param_big_circle_width,
             params->down_qr_code_param_big_circle_width);
    nh.param("down_qr_code_param_qr_need", params->down_qr_code_param_qr_need, params->down_qr_code_param_qr_need);
    nh.param("down_qr_code_param_dm_symbol", params->down_qr_code_param_dm_symbol,
             params->down_qr_code_param_dm_symbol);
    nh.param("down_qr_code_param_decode_timeout", params->down_qr_code_param_decode_timeout,
             params->down_qr_code_param_decode_timeout);
    nh.param("down_qr_code_param_big_circle_flag", params->down_qr_code_param_big_circle_flag,
             params->down_qr_code_param_big_circle_flag);
    nh.param("down_qr_code_param_small_circle_flag", params->down_qr_code_param_small_circle_flag,
             params->down_qr_code_param_small_circle_flag);
    nh.param("down_qr_code_param_encode_type", params->down_qr_code_param_encode_type,
             params->down_qr_code_param_encode_type);
    nh.param("down_qr_code_param_save_video", params->down_qr_code_param_save_video,
             params->down_qr_code_param_save_video);

    nh.param("up_qr_code_param_rows", params->up_qr_code_param_rows, params->up_qr_code_param_rows);
    nh.param("up_qr_code_param_cols", params->up_qr_code_param_cols, params->up_qr_code_param_cols);
    nh.param("up_qr_code_param_qr_size", params->up_qr_code_param_qr_size, params->up_qr_code_param_qr_size);
    nh.param("up_qr_code_param_qr_interval", params->up_qr_code_param_qr_interval,
             params->up_qr_code_param_qr_interval);
    nh.param("up_qr_code_param_radius_big", params->up_qr_code_param_radius_big, params->up_qr_code_param_radius_big);
    nh.param("up_qr_code_param_radius_small", params->up_qr_code_param_radius_small,
             params->up_qr_code_param_radius_small);
    nh.param("up_qr_code_param_circle_dis", params->up_qr_code_param_circle_dis, params->up_qr_code_param_circle_dis);
    nh.param("up_qr_code_param_point_height", params->up_qr_code_param_point_height,
             params->up_qr_code_param_point_height);
    nh.param("up_qr_code_param_big_circle_width", params->up_qr_code_param_big_circle_width,
             params->up_qr_code_param_big_circle_width);
    nh.param("up_qr_code_param_qr_need", params->up_qr_code_param_qr_need, params->up_qr_code_param_qr_need);
    nh.param("up_qr_code_param_dm_symbol", params->up_qr_code_param_dm_symbol, params->up_qr_code_param_dm_symbol);
    nh.param("up_qr_code_param_decode_timeout", params->up_qr_code_param_decode_timeout,
             params->up_qr_code_param_decode_timeout);
    nh.param("up_qr_code_param_big_circle_flag", params->up_qr_code_param_big_circle_flag,
             params->up_qr_code_param_big_circle_flag);
    nh.param("up_qr_code_param_small_circle_flag", params->up_qr_code_param_small_circle_flag,
             params->up_qr_code_param_small_circle_flag);
    nh.param("up_qr_code_param_encode_type", params->up_qr_code_param_encode_type,
             params->up_qr_code_param_encode_type);
    nh.param("up_qr_code_param_save_video", params->up_qr_code_param_save_video, params->up_qr_code_param_save_video);
}

void ReadRuntimeOptions(ros::NodeHandle& nh, RuntimeOptions* options)
{
    nh.param("qr_type", options->qr_type, options->qr_type);
    nh.param("camera_id", options->camera_id, options->camera_id);
    nh.param("placeholder_camera_id", options->placeholder_camera_id, options->placeholder_camera_id);
    nh.param("publish_placeholder_up", options->publish_placeholder_up, options->publish_placeholder_up);
    nh.param("publish_rate_hz", options->publish_rate_hz, options->publish_rate_hz);
    nh.param("stale_timeout_sec", options->stale_timeout_sec, options->stale_timeout_sec);
    nh.param("auto_reconnect", options->auto_reconnect, options->auto_reconnect);
    nh.param("reconnect_interval_sec", options->reconnect_interval_sec, options->reconnect_interval_sec);
    nh.param("update_service_name", options->update_service_name, options->update_service_name);
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
    options.stale_timeout_sec = 0.1;        // 超过该时间没有新识别结果时，对外发布 valid=0
    options.auto_reconnect = true;          // 相机启动失败或未运行时，是否由定时器自动重连
    options.reconnect_interval_sec = 3.0;   // 自动重连尝试间隔
    options.update_service_name = "update_qr_code_identify_lanxin_params";
    return options;
}

bool ValidateLxCameraParams(const common_msgs::config_lx_camera& params, std::string* error)
{
    return ValidateOneCameraParams(params, "down", false, error) && ValidateOneCameraParams(params, "up", true, error);
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
    if (!ValidateLxCameraParams(params, error)) {
        return false;
    }

    SetParam(private_nh, "down_qr_camera_ip", params.down_qr_camera_ip);
    SetParam(private_nh, "down_qr_camera_param_exposure", params.down_qr_camera_param_exposure);
    SetParam(private_nh, "down_qr_camera_param_gain", params.down_qr_camera_param_gain);
    SetParam(private_nh, "down_qr_camera_param_led_brightness", params.down_qr_camera_param_led_brightness);

    SetParam(private_nh, "up_qr_camera_ip", params.up_qr_camera_ip);
    SetParam(private_nh, "up_qr_camera_param_exposure", params.up_qr_camera_param_exposure);
    SetParam(private_nh, "up_qr_camera_param_gain", params.up_qr_camera_param_gain);
    SetParam(private_nh, "up_qr_camera_param_led_brightness", params.up_qr_camera_param_led_brightness);

    SetParam(private_nh, "down_qr_code_param_rows", params.down_qr_code_param_rows);
    SetParam(private_nh, "down_qr_code_param_cols", params.down_qr_code_param_cols);
    SetParam(private_nh, "down_qr_code_param_qr_size", params.down_qr_code_param_qr_size);
    SetParam(private_nh, "down_qr_code_param_qr_interval", params.down_qr_code_param_qr_interval);
    SetParam(private_nh, "down_qr_code_param_radius_big", params.down_qr_code_param_radius_big);
    SetParam(private_nh, "down_qr_code_param_radius_small", params.down_qr_code_param_radius_small);
    SetParam(private_nh, "down_qr_code_param_circle_dis", params.down_qr_code_param_circle_dis);
    SetParam(private_nh, "down_qr_code_param_point_height", params.down_qr_code_param_point_height);
    SetParam(private_nh, "down_qr_code_param_big_circle_width", params.down_qr_code_param_big_circle_width);
    SetParam(private_nh, "down_qr_code_param_qr_need", params.down_qr_code_param_qr_need);
    SetParam(private_nh, "down_qr_code_param_dm_symbol", params.down_qr_code_param_dm_symbol);
    SetParam(private_nh, "down_qr_code_param_decode_timeout", params.down_qr_code_param_decode_timeout);
    SetParam(private_nh, "down_qr_code_param_big_circle_flag", params.down_qr_code_param_big_circle_flag);
    SetParam(private_nh, "down_qr_code_param_small_circle_flag", params.down_qr_code_param_small_circle_flag);
    SetParam(private_nh, "down_qr_code_param_encode_type", params.down_qr_code_param_encode_type);
    SetParam(private_nh, "down_qr_code_param_save_video", params.down_qr_code_param_save_video);

    SetParam(private_nh, "up_qr_code_param_rows", params.up_qr_code_param_rows);
    SetParam(private_nh, "up_qr_code_param_cols", params.up_qr_code_param_cols);
    SetParam(private_nh, "up_qr_code_param_qr_size", params.up_qr_code_param_qr_size);
    SetParam(private_nh, "up_qr_code_param_qr_interval", params.up_qr_code_param_qr_interval);
    SetParam(private_nh, "up_qr_code_param_radius_big", params.up_qr_code_param_radius_big);
    SetParam(private_nh, "up_qr_code_param_radius_small", params.up_qr_code_param_radius_small);
    SetParam(private_nh, "up_qr_code_param_circle_dis", params.up_qr_code_param_circle_dis);
    SetParam(private_nh, "up_qr_code_param_point_height", params.up_qr_code_param_point_height);
    SetParam(private_nh, "up_qr_code_param_big_circle_width", params.up_qr_code_param_big_circle_width);
    SetParam(private_nh, "up_qr_code_param_qr_need", params.up_qr_code_param_qr_need);
    SetParam(private_nh, "up_qr_code_param_dm_symbol", params.up_qr_code_param_dm_symbol);
    SetParam(private_nh, "up_qr_code_param_decode_timeout", params.up_qr_code_param_decode_timeout);
    SetParam(private_nh, "up_qr_code_param_big_circle_flag", params.up_qr_code_param_big_circle_flag);
    SetParam(private_nh, "up_qr_code_param_small_circle_flag", params.up_qr_code_param_small_circle_flag);
    SetParam(private_nh, "up_qr_code_param_encode_type", params.up_qr_code_param_encode_type);
    SetParam(private_nh, "up_qr_code_param_save_video", params.up_qr_code_param_save_video);

    return true;
}

bool WriteLxCameraParamsToDefaultFile(const common_msgs::config_lx_camera& params, std::string* error)
{
    if (!ValidateLxCameraParams(params, error)) {
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
        WriteYamlValue(out, "down_qr_camera_ip", params.down_qr_camera_ip);
        WriteYamlValue(out, "down_qr_camera_param_exposure", params.down_qr_camera_param_exposure);
        WriteYamlValue(out, "down_qr_camera_param_gain", params.down_qr_camera_param_gain);
        WriteYamlValue(out, "down_qr_camera_param_led_brightness", params.down_qr_camera_param_led_brightness);
        out << "\n";

        WriteYamlValue(out, "up_qr_camera_ip", params.up_qr_camera_ip);
        WriteYamlValue(out, "up_qr_camera_param_exposure", params.up_qr_camera_param_exposure);
        WriteYamlValue(out, "up_qr_camera_param_gain", params.up_qr_camera_param_gain);
        WriteYamlValue(out, "up_qr_camera_param_led_brightness", params.up_qr_camera_param_led_brightness);
        out << "\n";

        out << "# QR detection params\n";
        WriteYamlValue(out, "down_qr_code_param_rows", params.down_qr_code_param_rows);
        WriteYamlValue(out, "down_qr_code_param_cols", params.down_qr_code_param_cols);
        WriteYamlValue(out, "down_qr_code_param_qr_size", params.down_qr_code_param_qr_size);
        WriteYamlValue(out, "down_qr_code_param_qr_interval", params.down_qr_code_param_qr_interval);
        WriteYamlValue(out, "down_qr_code_param_radius_big", params.down_qr_code_param_radius_big);
        WriteYamlValue(out, "down_qr_code_param_radius_small", params.down_qr_code_param_radius_small);
        WriteYamlValue(out, "down_qr_code_param_circle_dis", params.down_qr_code_param_circle_dis);
        WriteYamlValue(out, "down_qr_code_param_point_height", params.down_qr_code_param_point_height);
        WriteYamlValue(out, "down_qr_code_param_big_circle_width", params.down_qr_code_param_big_circle_width);
        WriteYamlValue(out, "down_qr_code_param_qr_need", params.down_qr_code_param_qr_need);
        WriteYamlValue(out, "down_qr_code_param_dm_symbol", params.down_qr_code_param_dm_symbol);
        WriteYamlValue(out, "down_qr_code_param_decode_timeout", params.down_qr_code_param_decode_timeout);
        WriteYamlValue(out, "down_qr_code_param_big_circle_flag", params.down_qr_code_param_big_circle_flag);
        WriteYamlValue(out, "down_qr_code_param_small_circle_flag", params.down_qr_code_param_small_circle_flag);
        WriteYamlValue(out, "down_qr_code_param_encode_type", params.down_qr_code_param_encode_type);
        WriteYamlValue(out, "down_qr_code_param_save_video", params.down_qr_code_param_save_video);
        out << "\n";

        WriteYamlValue(out, "up_qr_code_param_rows", params.up_qr_code_param_rows);
        WriteYamlValue(out, "up_qr_code_param_cols", params.up_qr_code_param_cols);
        WriteYamlValue(out, "up_qr_code_param_qr_size", params.up_qr_code_param_qr_size);
        WriteYamlValue(out, "up_qr_code_param_qr_interval", params.up_qr_code_param_qr_interval);
        WriteYamlValue(out, "up_qr_code_param_radius_big", params.up_qr_code_param_radius_big);
        WriteYamlValue(out, "up_qr_code_param_radius_small", params.up_qr_code_param_radius_small);
        WriteYamlValue(out, "up_qr_code_param_circle_dis", params.up_qr_code_param_circle_dis);
        WriteYamlValue(out, "up_qr_code_param_point_height", params.up_qr_code_param_point_height);
        WriteYamlValue(out, "up_qr_code_param_big_circle_width", params.up_qr_code_param_big_circle_width);
        WriteYamlValue(out, "up_qr_code_param_qr_need", params.up_qr_code_param_qr_need);
        WriteYamlValue(out, "up_qr_code_param_dm_symbol", params.up_qr_code_param_dm_symbol);
        WriteYamlValue(out, "up_qr_code_param_decode_timeout", params.up_qr_code_param_decode_timeout);
        WriteYamlValue(out, "up_qr_code_param_big_circle_flag", params.up_qr_code_param_big_circle_flag);
        WriteYamlValue(out, "up_qr_code_param_small_circle_flag", params.up_qr_code_param_small_circle_flag);
        WriteYamlValue(out, "up_qr_code_param_encode_type", params.up_qr_code_param_encode_type);
        WriteYamlValue(out, "up_qr_code_param_save_video", params.up_qr_code_param_save_video);

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
