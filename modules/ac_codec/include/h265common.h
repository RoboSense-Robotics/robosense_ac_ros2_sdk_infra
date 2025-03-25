/*********************************************************************************************************************
  Copyright 2025 RoboSense Technology Co., Ltd

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*********************************************************************************************************************/
#ifndef RSH265COMMON_H
#define RSH265COMMON_H

#include "common.h"
#include <iostream>
#include <memory>
#include <string>
#include <vector>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/common.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
namespace robosense {
namespace h265 {

enum class H265_CODER_TYPE {
  RS_H265_CODER_ENCODE = 0, // 编码
  RS_H265_CODER_DECODE,     // 解码
};

enum class H265_CODES_RATE_TYPE : int {
  RS_H265_CODES_LOW,
  RS_H265_CODES_MID,
  RS_H265_CODES_HIGH,
  RS_H265_CODES_HUGE,
  RS_H265_CODES_CUSTOM,
};

class H265CodesConfig {
public:
  robosense::common::ImageFrameFormat imageFrameFormat; //输入输出的图像格式
  H265_CODER_TYPE codeType;
  int imgWidth;
  int imgHeight;
  int imgFreq{30};
  H265_CODES_RATE_TYPE codesRateType{
      H265_CODES_RATE_TYPE::RS_H265_CODES_CUSTOM};
  int imgCodeRate{5000000}; // 自定义码率

public:
  AVPixelFormat pixelFormatHelper() {
    switch (imageFrameFormat) {
    case robosense::common::FRAME_FORMAT_RGB24: {
      return AV_PIX_FMT_YUV420P;
    }
    case robosense::common::FRAME_FORMAT_NV12: {
      return AV_PIX_FMT_YUV420P;
    }
    case robosense::common::FRAME_FORMAT_YUYV422: {
      return AV_PIX_FMT_YUV422P;
    }
    case robosense::common::FRAME_FORMAT_YUV420P: {
      return AV_PIX_FMT_YUV420P;
    }
    default: {
      return AV_PIX_FMT_YUV420P;
    }
    }
  }

  int bitRateHelper() {
    int bit_rate = 0;
    switch (codesRateType) {
    case H265_CODES_RATE_TYPE::RS_H265_CODES_LOW: {
      bit_rate = imgWidth * imgHeight * 3 / 2;
      break;
    }
    case H265_CODES_RATE_TYPE::RS_H265_CODES_MID: {
      bit_rate = imgWidth * imgHeight * 3;
      break;
    }
    case H265_CODES_RATE_TYPE::RS_H265_CODES_HIGH: {
      bit_rate = imgWidth * imgHeight * 3 * 2;
      break;
    }
    case H265_CODES_RATE_TYPE::RS_H265_CODES_HUGE: {
      bit_rate = imgWidth * imgHeight * 3 * 4;
      break;
    }
    case H265_CODES_RATE_TYPE::RS_H265_CODES_CUSTOM: {
      bit_rate = imgCodeRate;
      break;
    }
    }

    return bit_rate;
  }

  int imageSourceFormatDataSize() {
    int dataSize = 0;
    switch (imageFrameFormat) {
    case robosense::common::FRAME_FORMAT_NV12:
    case robosense::common::FRAME_FORMAT_YUV420P: {
      dataSize =
          av_image_get_buffer_size(AV_PIX_FMT_YUV420P, imgWidth, imgHeight, 1);
      std::cout << "source yuv420x dataSize = " << dataSize << std::endl;
      break;
    }
    case robosense::common::FRAME_FORMAT_RGB24: {
      dataSize =
          av_image_get_buffer_size(AV_PIX_FMT_RGB24, imgWidth, imgHeight, 1);
      std::cout << "source rgb24 dataSize = " << dataSize << std::endl;
      break;
    }
    case robosense::common::FRAME_FORMAT_YUYV422: {
      dataSize =
          av_image_get_buffer_size(AV_PIX_FMT_YUV422P, imgWidth, imgHeight, 1);
      std::cout << "source yuv422x dataSize = " << dataSize << std::endl;
      break;
    }
    default:
      break;
    }
    return dataSize;
  }

  int imageFormatDataSize() {
    int dataSize = 0;
    switch (imageFrameFormat) {
    case robosense::common::FRAME_FORMAT_NV12:
    case robosense::common::FRAME_FORMAT_YUV420P: {
      dataSize =
          av_image_get_buffer_size(AV_PIX_FMT_YUV420P, imgWidth, imgHeight, 1);
      std::cout << "ffmpeg yuv420x dataSize = " << dataSize << std::endl;
      break;
    }
    case robosense::common::FRAME_FORMAT_RGB24: {
      dataSize =
          av_image_get_buffer_size(AV_PIX_FMT_YUV420P, imgWidth, imgHeight, 1);
      std::cout << "ffmpeg rgb24 dataSize = " << dataSize << std::endl;
      break;
    }
    case robosense::common::FRAME_FORMAT_YUYV422: {
      dataSize =
          av_image_get_buffer_size(AV_PIX_FMT_YUV422P, imgWidth, imgHeight, 1);
      std::cout << "ffmpeg yuv422x dataSize = " << dataSize << std::endl;
      break;
    }
    default:
      break;
    }
    return dataSize;
  }

  std::string toString() {
    return std::string(codeType == H265_CODER_TYPE::RS_H265_CODER_ENCODE
                           ? "Encode"
                           : "Decode") +
           "_WxH_" + std::to_string(imgWidth) + "x" +
           std::to_string(imgHeight) + "bitRate_" +
           std::to_string(bitRateHelper());
  }
};

} // namespace h265
} // namespace robosense

#endif // RSH265COMMON_H
