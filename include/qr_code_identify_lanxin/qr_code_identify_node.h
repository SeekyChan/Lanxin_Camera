#ifndef QR_CODE_IDENTIFY_LANXIN_QR_CODE_IDENTIFY_NODE_H
#define QR_CODE_IDENTIFY_LANXIN_QR_CODE_IDENTIFY_NODE_H

#include <string>

#include <ros/ros.h>

#include "common_msgs/SetLxCameraParams.h"
#include "common_msgs/one_qr_data.h"
#include "common_msgs/qr_data.h"
#include "qr_code_identify_lanxin/lanxin_sdk_client.h"
#include "qr_code_identify_lanxin/qr_camera_config.h"
#include "qr_code_identify_lanxin/qr_result_buffer.h"

namespace qr_code_identify {

class QrCodeIdentifyNode {
 public:
  QrCodeIdentifyNode(const ros::NodeHandle& nh,
                     const ros::NodeHandle& private_nh);

  bool Start();

 private:
  bool StartCamera(const std::string& reason);
  bool RestartCamera(const std::string& reason, std::string* error);

  void PublishTimerCallback(const ros::TimerEvent& event);
  void ReconnectTimerCallback(const ros::TimerEvent& event);
  void StatusTimerCallback(const ros::TimerEvent& event);

  bool UpdateParamsCallback(common_msgs::SetLxCameraParams::Request& request,
                            common_msgs::SetLxCameraParams::Response& response);

  common_msgs::one_qr_data BuildPrimaryQrData() const;
  common_msgs::one_qr_data BuildInvalidQrData(int camera_id) const;
  void FillCurrentParams(common_msgs::SetLxCameraParams::Response* response) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher qr_data_pub_;
  ros::ServiceServer update_params_service_;
  ros::Timer publish_timer_;
  ros::Timer reconnect_timer_;
  ros::Timer status_timer_;

  QrCameraConfig config_;
  bool config_loaded_;

  QrResultBuffer result_buffer_;
  LanxinSdkClient sdk_client_;
};

}  // namespace qr_code_identify

#endif  // QR_CODE_IDENTIFY_LANXIN_QR_CODE_IDENTIFY_NODE_H
