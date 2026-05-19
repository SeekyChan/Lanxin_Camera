#ifndef QR_CODE_IDENTIFY_LANXIN_LANXIN_SDK_CLIENT_H
#define QR_CODE_IDENTIFY_LANXIN_LANXIN_SDK_CLIENT_H

#include <stdint.h>

#include <atomic>
#include <mutex>
#include <string>

#include "VmrQrCameraApi.h"
#include "qr_code_identify_lanxin/qr_camera_config.h"
#include "qr_code_identify_lanxin/qr_result_buffer.h"

namespace qr_code_identify {

struct SdkStats {
  uint64_t image_count;
  uint64_t detection_count;
  uint64_t null_detection_count;
  uint64_t decode_failure_count;

  SdkStats()
      : image_count(0),
        detection_count(0),
        null_detection_count(0),
        decode_failure_count(0) {}
};

class LanxinSdkClient {
 public:
  explicit LanxinSdkClient(QrResultBuffer* result_buffer);
  ~LanxinSdkClient();

  bool Start(const QrCameraConfig& config, std::string* error);
  bool Restart(const QrCameraConfig& config, std::string* error);
  void Stop();

  bool IsRunning() const;
  SdkStats TakeStats();

 private:
  struct CallbackContext {
    QrResultBuffer* result_buffer;
    std::atomic<bool> accepting_callbacks;
    std::atomic<int> encode_type;
    std::atomic<uint64_t> image_count;
    std::atomic<uint64_t> detection_count;
    std::atomic<uint64_t> null_detection_count;
    std::atomic<uint64_t> decode_failure_count;
    std::atomic<uint64_t> session_id;

    CallbackContext();
  };

  static void OnImage(VmrQrCameraImageData* image, void* user_data);
  static void OnDetection(VmrQrCameraDetectionData* result, void* user_data);

  bool StartLocked(const QrCameraConfig& config, std::string* error);
  void StopLocked();
  bool CheckSdkCall(VmrQrCameraExc code, const std::string& action,
                    std::string* error) const;

  mutable std::mutex lifecycle_mutex_;
  VmrQrCameraHandle handle_;
  bool running_;
  CallbackContext callback_context_;
};

}  // namespace qr_code_identify

#endif  // QR_CODE_IDENTIFY_LANXIN_LANXIN_SDK_CLIENT_H
