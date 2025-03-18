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
#ifndef OPENCVJPEGENCODER_H
#define OPENCVJPEGENCODER_H

#include "jpegcommon.h"

namespace robosense {
namespace jpeg {

class OpencvJpegEncoder {

public:
  using Ptr = std::shared_ptr<OpencvJpegEncoder>;
  using ConstPtr = std::shared_ptr<const OpencvJpegEncoder>;

public:
  OpencvJpegEncoder();
  ~OpencvJpegEncoder();

public:
  int init(const int imageWidth, const int imageHeight, const int jpegQuality,
           const RS_GPU_JPEG_ENCODE_SUPPORT_TYPE source_type =
               RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12,
           const RS_GPU_JPEG_ENCODE_SAMPLE_TYPE sample_type =
               RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_FULL);

  int encode(unsigned char *yuv420Buffer, int yuv420BufferLen,
             unsigned char *jpegBuffer, size_t &jpegBufferLen);

private:
  int init();

private:
  bool is_initial_;
  std::vector<unsigned char> buffer_;
  int jpeg_quality_;
  cv::Mat source_mat_;
  int source_size_;
  cv::Mat rgb_mat_;
  cv::Mat bgr_mat_; 
  int image_width_;
  int image_height_;
  RS_GPU_JPEG_ENCODE_SUPPORT_TYPE source_type_;
  RS_GPU_JPEG_ENCODE_SAMPLE_TYPE sample_type_;
};

} // namespace jpeg
} // namespace robosense

#endif // OPENCVJPEGENCODER_H
