#include "qr_result_buffer.h"

namespace qr_code_identify
{

QrDetection::QrDetection()
    : valid(false),
      value(0),
      x_offset(0.0f),
      y_offset(0.0f),
      theta_offset(0.0f),
      frame_count(0),
      sdk_timestamp_ns(0),
      received_time(ros::Time(0))
{
}

QrResultBuffer::QrResultBuffer() : has_detection_(false), latest_(QrDetection())
{
}

void QrResultBuffer::Update(const QrDetection& detection)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = detection;
    has_detection_ = true;
}

bool QrResultBuffer::Latest(double max_age_sec, QrDetection* detection) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_detection_) {
        return false;
    }

    if (max_age_sec > 0.0) {
        const ros::Duration age = ros::Time::now() - latest_.received_time;
        if (age.toSec() > max_age_sec) {
            return false;
        }
    }

    if (detection) {
        *detection = latest_;
    }
    return true;
}

void QrResultBuffer::Clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    has_detection_ = false;
    latest_ = QrDetection();
}

}  // namespace qr_code_identify
