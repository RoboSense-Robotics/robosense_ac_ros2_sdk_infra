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
#include "opencvjpegdecoder.h"

namespace robosense {
namespace jpeg {

OpencvJpegDecoder::OpencvJpegDecoder() { is_initial_ = false; }

OpencvJpegDecoder::~OpencvJpegDecoder() {}

int OpencvJpegDecoder::init(const int imageWidth, const int imageHeight,
                            const RS_GPU_JPEG_ENCODE_SUPPORT_TYPE output_type) {
  image_width_ = imageWidth;
  image_height_ = imageHeight;
  output_type_ = output_type;

  return init();
}

int OpencvJpegDecoder::decode(unsigned char *jpegBuffer, int jpegBufferLen,
                              unsigned char *rawBuffer, size_t &rawBufferLen) {
  if (!is_initial_) {
    return -1;
  } else if (jpegBuffer == nullptr) {
    return -2;
  } else if (jpegBufferLen == 0) {
    return -3;
  }

  if (buffer_.size() != static_cast<size_t>(jpegBufferLen)) {
    buffer_.resize(jpegBufferLen, '\0');
  }
  memcpy(buffer_.data(), jpegBuffer, jpegBufferLen);

  cv::Mat bgr_mat = cv::imdecode(buffer_, cv::IMREAD_COLOR);
  if (bgr_mat.empty()) {
    return -4;
  } else if (!bgr_mat.isContinuous()) {
    return -5;
  }

  switch (output_type_) {
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_RGB: {
    cv::Mat rgb_mat;
    cv::cvtColor(bgr_mat, rgb_mat, cv::COLOR_BGR2RGB);
    if (rgb_mat.empty()) {
      return -6;
    } else if (!rgb_mat.isContinuous()) {
      return -7;
    }
    const size_t rgb_size = image_width_ * image_height_ * 3;
    if (rawBufferLen < rgb_size) {
      return -8;
    }
    memcpy(rawBuffer, rgb_mat.data, rgb_size);
    rawBufferLen = rgb_size;
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12: {
    cv::Mat yuv420p_mat;
    cv::cvtColor(bgr_mat, yuv420p_mat, cv::COLOR_BGR2YUV_I420);
    if (yuv420p_mat.empty()) {
      return -9;
    } else if (!yuv420p_mat.isContinuous()) {
      return -10;
    }
    const size_t yuv420p_size = image_width_ * image_height_ * 3 / 2;
    if (rawBufferLen < yuv420p_size) {
      return -11;
    }
    std::vector<unsigned char> yuv420p_buffer(yuv420p_size, '\0');
    memcpy(yuv420p_buffer.data(), yuv420p_mat.data, yuv420p_size);
    const int y_size = image_width_ * image_height_;
    const int u_v_size = image_width_ * image_height_ / 4;
    const int u_offset = image_width_ * image_height_;
    const int v_offset = image_width_ * image_height_ * 5 / 4;

    memcpy(rawBuffer, yuv420p_buffer.data(), y_size);
    int index = 0;
    for (int i = 0; i < u_v_size; ++i) {
      rawBuffer[y_size + index] = yuv420p_buffer[u_offset + i];
      rawBuffer[y_size + index + 1] = yuv420p_buffer[v_offset + i];
      index += 2;
    }
    rawBufferLen = yuv420p_size; 
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUV420P: {
    cv::Mat yuv420p_mat;
    cv::cvtColor(bgr_mat, yuv420p_mat, cv::COLOR_BGR2YUV_I420);
    if (yuv420p_mat.empty()) {
      return -12;
    } else if (!yuv420p_mat.isContinuous()) {
      return -13;
    }
    const size_t yuv420p_size = image_width_ * image_height_ * 3 / 2;
    if (rawBufferLen < yuv420p_size) {
      return -14;
    }
    memcpy(rawBuffer, yuv420p_mat.data, yuv420p_size);
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUYV422: {
    cv::Mat yuv444_mat;
    cv::cvtColor(bgr_mat, yuv444_mat, cv::COLOR_BGR2YUV);
    if (yuv444_mat.empty()) {
      return -15;
    } else if (!yuv444_mat.isContinuous()) {
      return -16;
    }
    const size_t yuyv_size = image_width_ * image_height_ * 2;
    if (rawBufferLen < yuyv_size) {
      return -17;
    }
    int index = 0;
    for (int y = 0; y < image_height_; ++y) {
      cv::Vec3b *rows = yuv444_mat.ptr<cv::Vec3b>(y);
      for (int x = 0; x < image_width_; ++x) {
        rawBuffer[index] = rows[x][0];
        rawBuffer[index + 1] = (x % 2 == 0 ? rows[x][1] : rows[x][2]);
        index += 2;
      }
    }
    rawBufferLen = yuyv_size;
    break;
  }
  }

  return 0;
}

int OpencvJpegDecoder::init() {
  is_initial_ = true;
  return 0;
}

} // namespace jpeg
} // namespace robosense
