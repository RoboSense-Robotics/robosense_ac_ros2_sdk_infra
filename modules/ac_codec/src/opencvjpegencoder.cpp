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
#include "opencvjpegencoder.h"

namespace robosense {
namespace jpeg {

OpencvJpegEncoder::OpencvJpegEncoder() {
  is_initial_ = false;
  jpeg_quality_ = 70;
}

OpencvJpegEncoder::~OpencvJpegEncoder() {}

int OpencvJpegEncoder::init(const int imageWidth, const int imageHeight,
                            const int jpegQuality,
                            const RS_GPU_JPEG_ENCODE_SUPPORT_TYPE source_type,
                            const RS_GPU_JPEG_ENCODE_SAMPLE_TYPE sample_type) {
  image_width_ = imageWidth;
  image_height_ = imageHeight;
  jpeg_quality_ = jpegQuality;
  source_type_ = source_type;
  sample_type_ = sample_type;

  int ret = init();
  if (ret != 0) {
    return -1;
  }

  return 0;
}

int OpencvJpegEncoder::encode(unsigned char *yuv420Buffer, int yuv420BufferLen,
                              unsigned char *jpegBuffer,
                              size_t &jpegBufferLen) {
  if (is_initial_ == false) {
    return -1;
  } else if (yuv420Buffer == nullptr) {
    return -2;
  } else if (yuv420BufferLen != source_size_) {
    return -3;
  }

  switch (source_type_) {
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_RGB: {
    memcpy(rgb_mat_.data, yuv420Buffer, yuv420BufferLen);
    cv::cvtColor(rgb_mat_, bgr_mat_, cv::COLOR_RGB2BGR);
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12: {
    memcpy(source_mat_.data, yuv420Buffer, yuv420BufferLen);
    cv::cvtColor(source_mat_, bgr_mat_, cv::COLOR_YUV2BGR_NV12);
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUV420P: {
    memcpy(source_mat_.data, yuv420Buffer, yuv420BufferLen);
    cv::cvtColor(source_mat_, bgr_mat_, cv::COLOR_YUV2BGR_I420);
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUYV422: {
    memcpy(source_mat_.data, yuv420Buffer, yuv420BufferLen);
    cv::cvtColor(source_mat_, bgr_mat_, cv::COLOR_YUV2BGR_YUYV);
    break;
  }
  }

  if (bgr_mat_.empty()) {
    return -4;
  }

  int sample_factor = 1;
  if (sample_type_ ==
      RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_HALF) {
    sample_factor = 2;
  } else if (sample_type_ == RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::
                                 RS_GPU_JPEG_ENCODE_SAMPLE_ONE_THIRD) {
    sample_factor = 3;
  }

  cv::Mat encode_mat = bgr_mat_;
  if (sample_factor != 1) {
    cv::resize(
        bgr_mat_, encode_mat,
        cv::Size(image_width_ / sample_factor, image_height_ / sample_factor),
        0, 0, cv::INTER_LINEAR);
    if (encode_mat.empty()) {
      return -5;
    }
  }

  buffer_.clear();
  cv::imencode(".jpeg", encode_mat, buffer_,
               {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});

  if (buffer_.empty()) {
    return -6;
  } else if (buffer_.size() > jpegBufferLen) {
    return -7;
  }

  jpegBufferLen = buffer_.size();
  memcpy(jpegBuffer, buffer_.data(), jpegBufferLen);

  return 0;
}

int OpencvJpegEncoder::init() {
  switch (source_type_) {
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_RGB: {
    source_size_ = image_height_ * image_width_ * 3;
    source_mat_ = cv::Mat(image_height_, image_width_, CV_8UC3);
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12: {
    source_mat_ = cv::Mat(image_height_ * 3 / 2, image_width_, CV_8UC1);
    source_size_ = image_height_ * image_width_ * 3 / 2;
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUV420P: {
    source_mat_ = cv::Mat(image_height_ * 3 / 2, image_width_, CV_8UC1);
    source_size_ = image_height_ * image_width_ * 3 / 2;
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUYV422: {
    source_mat_ = cv::Mat(image_height_, image_width_, CV_8UC2);
    source_size_ = image_height_ * image_width_ * 2;
    break;
  }
  }
  if (!source_mat_.isContinuous()) {
    return -1;
  }
  rgb_mat_ = cv::Mat(image_height_, image_width_, CV_8UC3);
  if (!rgb_mat_.isContinuous()) {
    return -2;
  }
  bgr_mat_ = cv::Mat(image_height_, image_width_, CV_8UC3);
  is_initial_ = true;
  return 0;
}

} // namespace jpeg
} // namespace robosense
