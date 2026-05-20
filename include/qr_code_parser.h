#ifndef QR_CODE_IDENTIFY_LANXIN_QR_CODE_PARSER_H
#define QR_CODE_IDENTIFY_LANXIN_QR_CODE_PARSER_H

#include <stdint.h>

#include <string>

namespace qr_code_identify
{

struct ParsedQrCode {
    bool valid;
    uint32_t value;
    std::string extracted_text;
    std::string error;

    ParsedQrCode() : valid(false), value(0)
    {
    }
};

ParsedQrCode ParseQrCode(const std::string& raw_code, int encode_type);

double NormalizeAngleRad(double angle);
float ConvertSdkThetaToDegrees(float sdk_theta);

}  // namespace qr_code_identify

#endif  // QR_CODE_IDENTIFY_LANXIN_QR_CODE_PARSER_H
