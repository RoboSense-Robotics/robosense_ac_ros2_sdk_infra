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
#ifndef COLORCODEC_H
#define COLORCODEC_H

#include <memory>
#include <opencv2/opencv.hpp>

namespace robosense {
namespace color {

class ColorCodec {
public:
  using Ptr = std::shared_ptr<ColorCodec>;
  using ConstPtr = std::shared_ptr<const ColorCodec>;

public:
  ColorCodec() {}
  ~ColorCodec() {
  }

public:
  // 要求图像的行列数必须为偶数
  int init(const int imageWidth, const int imageHeight) {
    image_width_ = imageWidth;
    image_height_ = imageHeight;
    if (image_width_ % 2 != 0 || image_height_ % 2 != 0) {
      return -1;
    }
    image_rgb_size_ = ColorCodec::RGBImageSize(imageWidth, imageHeight);
    image_nv12_size_ = ColorCodec::NV12ImageSize(imageWidth, imageHeight);
    image_yuv420p_size_ = ColorCodec::YUV420PImageSize(imageWidth, imageHeight);
    image_yuv420_y_size_ =
        ColorCodec::YUV420_Y_ImageSize(imageWidth, imageHeight);
    image_yuv420_uv_size_ =
        ColorCodec::YUV420_UV_ImageSize(imageWidth, imageHeight);
    image_yuv420_u_v_size_ =
        ColorCodec::YUV420_U_V_ImageSize(imageWidth, imageHeight);
    image_yuv420p_u_offset_ = image_yuv420_y_size_;
    image_yuv420p_v_offset_ = image_yuv420_y_size_ + image_yuv420_u_v_size_;
    image_yuyv422_y_size_ =
        ColorCodec::YUYV422_Y_ImageSize(imageWidth, imageHeight);
    image_yuyv422_u_v_size_ =
        ColorCodec::YUYV422_U_V_ImageSize(imageWidth, imageHeight);
    image_yuyv422_uv_size_ =
        ColorCodec::YUYV422_UV_ImageSize(imageWidth, imageHeight);
    image_yuyv422p_u_offset_ = image_yuyv422_y_size_;
    image_yuyv422p_v_offset_ = image_yuyv422_y_size_ + image_yuyv422_u_v_size_;
    image_yuyv422_size_ = ColorCodec::YUYV422ImageSize(imageWidth, imageHeight);
    image_yuv422p_size_ = ColorCodec::YUV422PImageSize(imageWidth, imageHeight);
    yuv420_buffer_.resize(image_nv12_size_, '\0');
    yuyv422_buffer_.resize(image_yuyv422_size_, '\0');
    rgb_buffer_.resize(image_rgb_size_, '\0');
    return 0;
  }

  int YUYV422ToRGB(unsigned char *pSrcBuffer, const int srcBufferLen,
                   unsigned char *pDstBuffer, int &dstBufferLen);

  int NV12ToRGB(unsigned char *pSrcBuffer, const int srcBufferLen,
                unsigned char *pDstBuffer, int &dstBufferLen);

  int YUV420PToRGB(unsigned char *pSrcBuffer, const int srcBufferLen,
                   unsigned char *pDstBuffer, int &dstBufferLen);

  int RGBToYUYV422(unsigned char *pSrcBuffer, const int srcBufferLen,
                   unsigned char *pDstBuffer, int &dstBufferLen);

  int RGBToNV12(unsigned char *pSrcBuffer, const int srcBufferLen,
                unsigned char *pDstBuffer, int &dstBufferLen);

  int RGBToYUV420P(unsigned char *pSrcBuffer, const int srcBufferLen,
                   unsigned char *pDstBuffer, int &dstBufferLen);

  int NV12ToRgbToYUV422(unsigned char *pSrcBuffer, const int srcBufferLen,
                        unsigned char *pDstBuffer, int &dstBufferLen);

  int YUYV422ToRgbToNV12(unsigned char *pSrcBuffer, const int srcBufferLen,
                         unsigned char *pDstBuffer, int &dstBufferLen);

  int YUV420PToRgbToYUYV422(unsigned char *pSrcBuffer, const int srcBufferLen,
                            unsigned char *pDstBuffer, int &dstBufferLen);

  int YUYV422ToRgbToYUV420P(unsigned char *pSrcBuffer, const int srcBufferLen,
                            unsigned char *pDstBuffer, int &dstBufferLen);

  int NV12ToYUV420P(unsigned char *pSrcBuffer, const int srcBufferLen,
                    unsigned char *pDstBuffer, int &dstBufferLen);

  int YUV420PToNV12(unsigned char *pSrcBuffer, const int srcBufferLen,
                    unsigned char *pDstBuffer, int &dstBufferLen);

  int YUYV422ToYUV422P(unsigned char *pSrcBuffer, const int srcBufferLen,
                       unsigned char *pDstBuffer, int &dstBufferLen);

  int YUV422PToYUYV422(unsigned char *pSrcBuffer, const int srcBufferLen,
                       unsigned char *pDstBuffer, int &dstBufferLen);

public:
  // 获取图像的内存大小
  // RGB
  static int RGBImageSize(const int imageWidth, const int imageHeight) {
    return imageWidth * imageHeight * 3;
  }

  // YUV420x: NV12/420P
  static int NV12ImageSize(const int imageWidth, const int imageHeight) {
    return YUV420_Y_ImageSize(imageWidth, imageHeight) +
           YUV420_UV_ImageSize(imageWidth, imageHeight);
  }

  static int YUV420PImageSize(const int imageWidth, const int imageHeight) {
    return YUV420_Y_ImageSize(imageWidth, imageHeight) +
           YUV420_UV_ImageSize(imageWidth, imageHeight);
  }

  static int YUV420_Y_ImageSize(const int imageWidth, const int imageHeight) {
    return imageWidth * imageHeight;
  }

  static int YUV420_U_V_ImageSize(const int imageWidth, const int imageHeight) {
    return imageWidth * imageHeight / 4;
  }

  static int YUV420_UV_ImageSize(const int imageWidth, const int imageHeight) {
    return imageWidth * imageHeight / 2;
  }
  
  // YUV422x: YUYV422/YUV422P
  static int YUYV422_Y_ImageSize(const int imageWidth, const int imageHeight) {
    return imageWidth * imageHeight;
  }

  static int YUYV422_U_V_ImageSize(const int imageWidth,
                                   const int imageHeight) {
    return imageWidth * imageHeight / 2;
  }

  static int YUYV422_UV_ImageSize(const int imageWidth, const int imageHeight) {
    return imageWidth * imageHeight;
  }

  static int YUYV422ImageSize(const int imageWidth, const int imageHeight) {
    return imageWidth * imageHeight * 2;
  }

  static int YUV422PImageSize(const int imageWidth, const int imageHeight) {
    return imageWidth * imageHeight * 2;
  }

private:
  int image_width_;
  int image_height_;
  int image_rgb_size_;
  // yuv420x
  int image_nv12_size_;
  int image_yuv420_y_size_;
  int image_yuv420p_u_offset_;
  int image_yuv420p_v_offset_;
  int image_yuv420_u_v_size_;
  int image_yuv420_uv_size_;
  int image_yuv420p_size_;
  // yuyv422
  int image_yuyv422_y_size_;
  int image_yuyv422p_u_offset_;
  int image_yuyv422p_v_offset_;
  int image_yuyv422_u_v_size_;
  int image_yuyv422_uv_size_;
  int image_yuyv422_size_;
  int image_yuv422p_size_;
  std::vector<unsigned char> yuyv422_buffer_;
  std::vector<unsigned char> yuv420_buffer_;
  std::vector<unsigned char> rgb_buffer_;
};

} // namespace color
} // namespace robosense

#endif // COLORCODEC_H
