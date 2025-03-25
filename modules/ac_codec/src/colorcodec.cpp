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
#include "colorcodec.h"

namespace robosense {
namespace color {

int ColorCodec::YUYV422ToRGB(unsigned char *pSrcBuffer, const int srcBufferLen,
                             unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuyv422_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_rgb_size_) {
    return -1;
  }

  cv::Mat yuyvMat(image_height_, image_width_, CV_8UC2, pSrcBuffer);
  cv::Mat rgbMat;
  cv::cvtColor(yuyvMat, rgbMat, cv::COLOR_YUV2RGB_YUYV);
  if (rgbMat.empty()) {
    return -1;
  } else if (!rgbMat.isContinuous()) {
    return -2;
  }
  memcpy(pDstBuffer, rgbMat.data, image_rgb_size_);

  dstBufferLen = image_rgb_size_;
  return 0;
}

int ColorCodec::NV12ToRGB(unsigned char *pSrcBuffer, const int srcBufferLen,
                          unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_nv12_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_rgb_size_) {
    return -1;
  }

  cv::Mat yuvMat(image_height_ * 3 / 2, image_width_, CV_8UC1, pSrcBuffer);
  cv::Mat rgbMat;
  cv::cvtColor(yuvMat, rgbMat, cv::COLOR_YUV2RGB_NV12);

  if (rgbMat.empty()) {
    return -2;
  } else if (!rgbMat.isContinuous()) {
    return -3;
  }
  memcpy(pDstBuffer, rgbMat.data, image_rgb_size_);
  dstBufferLen = image_rgb_size_;
  return 0;
}

int ColorCodec::YUV420PToRGB(unsigned char *pSrcBuffer, const int srcBufferLen,
                             unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuv420p_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_rgb_size_) {
    return -1;
  }

  cv::Mat yuvMat(image_height_ * 3 / 2, image_width_, CV_8UC1, pSrcBuffer);
  cv::Mat rgbMat;
  cv::cvtColor(yuvMat, rgbMat, cv::COLOR_YUV2RGB_I420);

  if (rgbMat.empty()) {
    return -2;
  } else if (!rgbMat.isContinuous()) {
    return -3;
  }
  memcpy(pDstBuffer, rgbMat.data, image_rgb_size_);
  dstBufferLen = image_rgb_size_;
  return 0;
}

int ColorCodec::RGBToYUYV422(unsigned char *pSrcBuffer, const int srcBufferLen,
                             unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_rgb_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuyv422_size_) {
    return -1;
  }
  cv::Mat rgbMat(image_height_, image_width_, CV_8UC3, pSrcBuffer);
  cv::Mat yuv444Mat;
  cv::cvtColor(rgbMat, yuv444Mat, cv::COLOR_RGB2YUV); // YUV: 4:4:4
  if (yuv444Mat.empty()) {
    return -2;
  }
  // std::cout << "wxh = " << yuv444Mat.cols << "x" << yuv444Mat.rows <<
  // std::endl;
  int index = 0;
  for (int y = 0; y < image_height_; ++y) {
    cv::Vec3b *row = yuv444Mat.ptr<cv::Vec3b>(y);
    for (int x = 0; x < image_width_; ++x) {
      pDstBuffer[index] = row[x][0];
      pDstBuffer[index + 1] =
          (x % 2 == 0 ? row[x][1] : row[x][2]); // 偶数列取值: u, 奇数列取值: v
      index += 2;
    }
  }

  dstBufferLen = image_yuyv422_size_;
  return 0;
}

int ColorCodec::RGBToNV12(unsigned char *pSrcBuffer, const int srcBufferLen,
                          unsigned char *pDstBuffer, int &dstBufferLen) {

  if (pSrcBuffer == nullptr || srcBufferLen != image_rgb_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_nv12_size_) {
    return -1;
  }
  cv::Mat rgbMat(image_height_, image_width_, CV_8UC3, pSrcBuffer);

  cv::Mat yuv420pMat;
  cv::cvtColor(rgbMat, yuv420pMat, cv::COLOR_RGB2YUV_I420);

  if (yuv420pMat.empty()) {
    return -2;
  } else if (!yuv420pMat.isContinuous()) {
    return -3;
  }

  int ret = YUV420PToNV12(yuv420pMat.data, image_yuv420p_size_, pDstBuffer,
                          dstBufferLen);
  if (ret != 0) {
    return -4;
  }
  dstBufferLen = image_nv12_size_;
  return 0;
}

int ColorCodec::RGBToYUV420P(unsigned char *pSrcBuffer, const int srcBufferLen,
                             unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_rgb_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuv420p_size_) {
    return -1;
  }
  cv::Mat rgbMat(image_height_, image_width_, CV_8UC3, pSrcBuffer);
  cv::Mat yuv420pMat;
  cv::cvtColor(rgbMat, yuv420pMat, cv::COLOR_RGB2YUV_I420);
  if (yuv420pMat.empty()) {
    return -2;
  } else if (!yuv420pMat.isContinuous()) {
    return -3;
  }
  memcpy(pDstBuffer, yuv420pMat.data, image_yuv420p_size_);
  dstBufferLen = image_yuv420p_size_;

  return 0;
}

int ColorCodec::NV12ToRgbToYUV422(unsigned char *pSrcBuffer,
                                  const int srcBufferLen,
                                  unsigned char *pDstBuffer,
                                  int &dstBufferLen) {

  if (pSrcBuffer == nullptr || srcBufferLen != image_nv12_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuyv422_size_) {
    return -1;
  }

  int rgbBufferLen = rgb_buffer_.size();
  int ret =
      NV12ToRGB(pSrcBuffer, srcBufferLen, rgb_buffer_.data(), rgbBufferLen);
  if (ret != 0) {
    return -2;
  }

  ret = RGBToYUYV422(rgb_buffer_.data(), rgb_buffer_.size(), pDstBuffer,
                     dstBufferLen);
  if (ret != 0) {
    return -3;
  }

  return 0;
}

int ColorCodec::YUYV422ToRgbToNV12(unsigned char *pSrcBuffer,
                                   const int srcBufferLen,
                                   unsigned char *pDstBuffer,
                                   int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuyv422_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_nv12_size_) {
    return -1;
  }

  int rgbBufferLen = rgb_buffer_.size();
  int ret =
      YUYV422ToRGB(pSrcBuffer, srcBufferLen, rgb_buffer_.data(), rgbBufferLen);
  if (ret != 0) {
    return -2;
  }

  ret = RGBToNV12(rgb_buffer_.data(), rgb_buffer_.size(), pDstBuffer,
                  dstBufferLen);
  if (ret != 0) {
    return -3;
  }

  return 0;
}

int ColorCodec::YUV420PToRgbToYUYV422(unsigned char *pSrcBuffer,
                                      const int srcBufferLen,
                                      unsigned char *pDstBuffer,
                                      int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuv420p_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuyv422_size_) {
    return -1;
  }

  int rgbBufferLen = rgb_buffer_.size();
  int ret =
      YUV420PToRGB(pSrcBuffer, srcBufferLen, rgb_buffer_.data(), rgbBufferLen);
  if (ret != 0) {
    return -2;
  }

  ret = RGBToYUYV422(rgb_buffer_.data(), rgb_buffer_.size(), pDstBuffer,
                     dstBufferLen);
  if (ret != 0) {
    return -3;
  }

  return 0;
}

int ColorCodec::YUYV422ToRgbToYUV420P(unsigned char *pSrcBuffer,
                                      const int srcBufferLen,
                                      unsigned char *pDstBuffer,
                                      int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuyv422_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuv420p_size_) {
    return -1;
  }

  int rgbBufferLen = rgb_buffer_.size();
  int ret =
      YUYV422ToRGB(pSrcBuffer, srcBufferLen, rgb_buffer_.data(), rgbBufferLen);
  if (ret != 0) {
    return -2;
  }

  ret = RGBToYUV420P(rgb_buffer_.data(), rgb_buffer_.size(), pDstBuffer,
                     dstBufferLen);
  if (ret != 0) {
    return -3;
  }

  return 0;
}

int ColorCodec::NV12ToYUV420P(unsigned char *pSrcBuffer, const int srcBufferLen,
                              unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_nv12_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_nv12_size_) {
    return -1;
  }

  memcpy(pDstBuffer, pSrcBuffer, image_yuv420_y_size_);
  int index = 0;
  for (int i = 0; i < image_yuv420_uv_size_; i += 2) {
    pDstBuffer[image_yuv420p_u_offset_ + index] =
        pSrcBuffer[image_yuv420_y_size_ + i];
    pDstBuffer[image_yuv420p_v_offset_ + index] =
        pSrcBuffer[image_yuv420_y_size_ + i + 1];
    ++index;
  }
  dstBufferLen = image_nv12_size_;

  return 0;
}

int ColorCodec::YUV420PToNV12(unsigned char *pSrcBuffer, const int srcBufferLen,
                              unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_nv12_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_nv12_size_) {
    return -1;
  }

  memcpy(pDstBuffer, pSrcBuffer, image_yuv420_y_size_);
  int index = 0;
  for (int i = 0; i < image_yuv420_u_v_size_; ++i) {
    pDstBuffer[image_yuv420_y_size_ + index] =
        pSrcBuffer[image_yuv420p_u_offset_ + i];
    pDstBuffer[image_yuv420_y_size_ + index + 1] =
        pSrcBuffer[image_yuv420p_v_offset_ + i];
    index += 2;
  }
  dstBufferLen = image_nv12_size_;

  return 0;
}

int ColorCodec::YUYV422ToYUV422P(unsigned char *pSrcBuffer,
                                 const int srcBufferLen,
                                 unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuyv422_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuv422p_size_) {
    return -1;
  }

  int y_index = 0;
  int u_index = image_yuyv422p_u_offset_;
  int v_index = image_yuyv422p_v_offset_;
  for (int y = 0; y < image_height_; ++y) {
    for (int x = 0; x < image_width_; ++x) {
      int y_pos = (y * image_width_ + x) * 2;
      pDstBuffer[y_index++] = pSrcBuffer[y_pos];
      if (x % 2 == 0) {
        pDstBuffer[u_index++] = pSrcBuffer[y_pos + 1];
      } else {
        pDstBuffer[v_index++] = pSrcBuffer[y_pos + 1];
      }
    }
  }
  dstBufferLen = image_yuv422p_size_;

  return 0;
}

int ColorCodec::YUV422PToYUYV422(unsigned char *pSrcBuffer,
                                 const int srcBufferLen,
                                 unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuv422p_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuyv422_size_) {
    return -1;
  }

  int y_index = 0;
  int u_index = image_yuyv422p_u_offset_;
  int v_index = image_yuyv422p_v_offset_;
  int index = 0;
  for (int y = 0; y < image_height_; ++y) {
    for (int x = 0; x < image_width_; ++x) {
      int y_pos = (y * image_width_ + x) * 2;
      pDstBuffer[index++] = pSrcBuffer[y_index++];
      if (x % 2 == 0) {
        pDstBuffer[index++] = pSrcBuffer[u_index++];
      } else {
        pDstBuffer[index++] = pSrcBuffer[v_index++];
      }
    }
  }
  dstBufferLen = image_yuyv422_size_;

  return 0;
}

} // namespace color
} // namespace robosense
