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
#ifndef OPENCVJPEGDECODER_H
#define OPENCVJPEGDECODER_H

#include "jpegcommon.h"

namespace robosense {
namespace jpeg {

class OpencvJpegDecoder {
public:
  using Ptr = std::shared_ptr<OpencvJpegDecoder>;
  using ConstPtr = std::shared_ptr<const OpencvJpegDecoder>;

public:
  OpencvJpegDecoder();
  ~OpencvJpegDecoder();

public:
  int init(const int imageWidth, const int imageHeight,
           const RS_GPU_JPEG_ENCODE_SUPPORT_TYPE output_type =
               RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_RGB);

  int decode(unsigned char *jpegBuffer, int jpegBufferLen,
             unsigned char *rawBuffer, size_t &rawBufferLen);

private:
  int init();

private:
  bool is_initial_;
  int image_width_;
  int image_height_;
  std::vector<unsigned char> buffer_;
  RS_GPU_JPEG_ENCODE_SUPPORT_TYPE output_type_;
};

} // namespace jpeg
} // namespace robosense

#endif // OPENCVJPEGDECODER_H
