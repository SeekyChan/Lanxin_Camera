#include "qr_code_identify_lanxin/qr_code_parser.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>

#include <algorithm>
#include <cctype>
#include <climits>
#include <sstream>

namespace qr_code_identify {
namespace {

const double kPi = 3.14159265358979323846;

bool IsAllDigits(const std::string& text) {
  return !text.empty() &&
         std::all_of(text.begin(), text.end(), [](unsigned char ch) {
           return std::isdigit(ch) != 0;
         });
}

bool ParseUint32(const std::string& text, uint32_t* value, std::string* error) {
  if (!IsAllDigits(text)) {
    if (error) {
      *error = "decoded ID contains non-digit characters";
    }
    return false;
  }

  errno = 0;
  char* end = NULL;
  const unsigned long parsed = std::strtoul(text.c_str(), &end, 10);
  if (errno != 0 || end == NULL || *end != '\0' || parsed > 0xffffffffUL) {
    if (error) {
      *error = "decoded ID is outside uint32 range";
    }
    return false;
  }

  if (value) {
    *value = static_cast<uint32_t>(parsed);
  }
  return true;
}

}  // namespace

ParsedQrCode ParseQrCode(const std::string& raw_code, int encode_type) {
  ParsedQrCode parsed;

  if (encode_type == 1) {
    if (raw_code.size() < 4) {
      parsed.error = "Lanxin code is shorter than 4 characters";
      return parsed;
    }
    parsed.extracted_text = raw_code.substr(1, 3);
  } else if (encode_type == 5) {
    if (raw_code.size() != 10) {
      std::ostringstream oss;
      oss << "Pepperl+Fuchs code expects 10 characters, got " << raw_code.size();
      parsed.error = oss.str();
      return parsed;
    }
    parsed.extracted_text = raw_code.substr(raw_code.size() - 8, 8);
  } else {
    std::ostringstream oss;
    oss << "unsupported encode_type " << encode_type;
    parsed.error = oss.str();
    return parsed;
  }

  if (!ParseUint32(parsed.extracted_text, &parsed.value, &parsed.error)) {
    return parsed;
  }

  parsed.valid = true;
  return parsed;
}

double NormalizeAngleRad(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

float ConvertSdkThetaToDegrees(float sdk_theta) {
  const double theta = NormalizeAngleRad(0.5 * kPi - static_cast<double>(sdk_theta));
  return static_cast<float>(theta * 180.0 / kPi);
}

}  // namespace qr_code_identify
