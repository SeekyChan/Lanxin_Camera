#ifndef QR_CODE_IDENTIFY_LANXIN_QR_RESULT_BUFFER_H
#define QR_CODE_IDENTIFY_LANXIN_QR_RESULT_BUFFER_H

#include <stdint.h>

#include <mutex>
#include <string>

#include <ros/ros.h>

namespace qr_code_identify {

struct QrDetection {
  bool valid;
  uint32_t value;
  float x_offset;
  float y_offset;
  float theta_offset;
  std::string raw_code;
  std::string extracted_code;
  std::string decode_error;
  uint32_t frame_count;
  uint64_t sdk_timestamp_ns;
  ros::Time received_time;

  QrDetection();
};

class QrResultBuffer {
 public:
  QrResultBuffer();

  void Update(const QrDetection& detection);
  bool Latest(double max_age_sec, QrDetection* detection) const;
  void Clear();

 private:
  mutable std::mutex mutex_;
  bool has_detection_;
  QrDetection latest_;
};

}  // namespace qr_code_identify

#endif  // QR_CODE_IDENTIFY_LANXIN_QR_RESULT_BUFFER_H
