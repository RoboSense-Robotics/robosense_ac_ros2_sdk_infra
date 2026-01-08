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

#include "h265coder.h"
#include <rclcpp/rclcpp.hpp>

namespace robosense {
namespace h265 {

H265Coder::H265Coder() {
  m_pCodec = nullptr;
  m_pParser = nullptr;
  m_pCodecCtx = nullptr;
  m_pCodecFrame = nullptr;
  m_pCodecPkt = nullptr;
  m_pSwsCtx = nullptr;
  m_frameCnt = -1;
  m_pBuffer = nullptr;
  m_bufferSize = 0;
}

H265Coder::~H265Coder() {
  switch (m_h265CodesConfig.codeType) {
  case H265_CODER_TYPE::RS_H265_CODER_ENCODE: {
    if (m_pCodecPkt != nullptr) {
      av_packet_free(&m_pCodecPkt);
      m_pCodecPkt = nullptr;
    }

    if (m_pBuffer != nullptr) {
      av_free(m_pBuffer);
      m_pBuffer = nullptr;
    }

    if (m_pCodecFrame != nullptr) {
      av_frame_free(&m_pCodecFrame);
      m_pCodecFrame = nullptr;
    }

    if (m_pCodecCtx != nullptr) {
      avcodec_free_context(&m_pCodecCtx);
      m_pCodecCtx = nullptr;
    }

    break;
  }
  case H265_CODER_TYPE::RS_H265_CODER_DECODE: {
    // Not Need Free By User
    // if (m_pCodecPkt != nullptr) {
    //   av_packet_free(&m_pCodecPkt);
    //   m_pCodecPkt = nullptr;
    // }

    if (m_pParser != nullptr) {
      av_parser_close(m_pParser);
      m_pParser = nullptr;
    }

    if (m_pCodecFrame != nullptr) {
      av_frame_free(&m_pCodecFrame);
      m_pCodecFrame = nullptr;
    }

    if (m_pCodecCtx != nullptr) {
      avcodec_free_context(&m_pCodecCtx);
      m_pCodecCtx = nullptr;
    }

    break;
  }
  }
}

int H265Coder::init(const H265CodesConfig &h265CodesConfig) {
  m_h265CodesConfig = h265CodesConfig;
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 9, 100)
  avcodec_register_all();
#endif

  int ret = 0;
  switch (m_h265CodesConfig.codeType) {
  case H265_CODER_TYPE::RS_H265_CODER_ENCODE: {
    ret = initEncode();
    break;
  }
  case H265_CODER_TYPE::RS_H265_CODER_DECODE: {
    ret = initDecode();
    break;
  }
  }

  return ret;
}

void H265Coder::resetFrameId() { m_frameCnt = -1; }

int H265Coder::encode(unsigned char *data, int dataLen,
                      CODER_BUFFER_PTR_VEC &bufferPtr,
                      std::vector<bool> &iFrameFlags) {
  // Step1: 拷贝数据
  if (data == nullptr) {
    std::cerr << "Encode Input Data Is Nullptr" << std::endl;
    return -1;
  } else if (dataLen != static_cast<int>(m_sourceSize)) {
    std::cerr << "Encode Input Data Size != m_sourceSize " << std::endl;
    return -2;
  }

  int ret = 0;
  int buffer_len = m_bufferSize;
  switch (m_h265CodesConfig.imageFrameFormat) {
  case robosense::common::FRAME_FORMAT_RGB24: {
    ret = m_colorCodecPtr->RGBToYUV420P(data, dataLen, m_pBuffer, buffer_len);
    break;
  }
  case robosense::common::FRAME_FORMAT_NV12: {
    ret = m_colorCodecPtr->NV12ToYUV420P(data, dataLen, m_pBuffer, buffer_len);
    break;
  }
  case robosense::common::FRAME_FORMAT_YUYV422: {
    ret =
        m_colorCodecPtr->YUYV422ToYUV422P(data, dataLen, m_pBuffer, buffer_len);
    break;
  }
  case robosense::common::FRAME_FORMAT_YUV420P: {
    memcpy(m_pBuffer, data, buffer_len);
    break;
  }
  default: {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "H265 Not Support Format Type = %d", static_cast<int>(m_h265CodesConfig.imageFrameFormat));
    return -3;
  }
  }

  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "H265 Color Space Convert Failed: ret = %d", ret);
    return -4;
  }

  bufferPtr.clear();
  iFrameFlags.clear();
  ret = encode(bufferPtr, iFrameFlags);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Encode YUV Failed: ret = %d", ret);
    return -5;
  }

  return 0;
}

int H265Coder::decode(unsigned char *data, const int dataLen,
                      CODER_BUFFER_PTR_VEC &bufferPtr) {
  if (data == nullptr || dataLen == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Decode Input Data is Nullptr or Data Size = 0");
    return -1;
  }

  m_encodeData.resize(dataLen, '\0');
  memcpy(m_encodeData.data(), data, dataLen);

  bufferPtr.clear();
  int ret = decode(bufferPtr);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Decode H265 Failed: ret = %d", ret);
    return -2;
  }

  return 0;
}

int H265Coder::encodeFlush(CODER_BUFFER_PTR_VEC &bufferPtr,
                           std::vector<bool> &iFrameFlags) {
  int ret = 0;
  ret = avcodec_send_frame(m_pCodecCtx, NULL);
  if (ret < 0) {
    return -1;
  }

  bufferPtr.clear();
  iFrameFlags.clear();
  ret = encodeRecv(bufferPtr, iFrameFlags);
  if (ret != 0) {
    return -2;
  }

  return 0;
}

int H265Coder::decodeFlush(CODER_BUFFER_PTR_VEC &bufferPtr) {
  int ret = 0;
  ret = avcodec_send_packet(m_pCodecCtx, NULL);
  if (ret < 0) {
    return -1;
  }

  bufferPtr.clear();
  ret = decodeRecv(bufferPtr);
  if (ret != 0) {
    return -2;
  }

  return 0;
}

int H265Coder::initEncode() {
  int ret = 0;
  // 寻找编码器
  m_pCodec = avcodec_find_encoder(AV_CODEC_ID_HEVC);
  if (m_pCodec == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Find Encode Codec Failed !");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("h265coder"), "encode name = %s", m_pCodec->name);

  // 创建编码器上下文
  m_pCodecCtx = avcodec_alloc_context3(m_pCodec);
  if (m_pCodecCtx == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Allocate Encode CodecContext Failed !");
    return -2;
  }

  m_pCodecCtx->codec_id = m_pCodec->id;
  m_pCodecCtx->width = m_h265CodesConfig.imgWidth;
  m_pCodecCtx->height = m_h265CodesConfig.imgHeight;
  m_pCodecCtx->time_base = (AVRational){1, m_h265CodesConfig.imgFreq};
  m_pCodecCtx->framerate = (AVRational){m_h265CodesConfig.imgFreq, 1};

  m_sourceSize = m_h265CodesConfig.imageSourceFormatDataSize();
  m_bufferSize = m_h265CodesConfig.imageFormatDataSize();
  m_colorCodecPtr.reset(new robosense::color::ColorCodec());
  if (m_colorCodecPtr->init(m_h265CodesConfig.imgWidth,
                            m_h265CodesConfig.imgHeight)) {
    return -3;
  }

  m_pCodecCtx->pix_fmt = m_h265CodesConfig.pixelFormatHelper();
  m_pCodecCtx->profile = FF_PROFILE_HEVC_MAIN;
  m_pCodecCtx->bit_rate = m_h265CodesConfig.bitRateHelper();
  m_pCodecCtx->bit_rate_tolerance = 0.1 * m_pCodecCtx->bit_rate;
  // m_pCodecCtx->gop_size = 30;
  // m_pCodecCtx->max_b_frames = 1;

  // 编码速度:
  // ultrafast、superfast、veryfast、faster、fast、medium、slow、slower、veryslow、placebo
  AVDictionary *params = nullptr;
  av_dict_set(&params, "preset", "medium", 0);
  av_dict_set(&params, "tune", "zero-latency", 0);
  av_dict_set(&params, "x265-params", "keyint=30", 0);
  ret = avcodec_open2(m_pCodecCtx, m_pCodec, &params);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Open Encode Codec Failed !");
    return -3;
  }

  // 创建编码帧
  m_pCodecFrame = av_frame_alloc();
  if (m_pCodecFrame == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Allocate Encode AV Frame Failed !");
    return -4;
  }

  m_pBuffer = static_cast<unsigned char *>(av_malloc(m_bufferSize));
  if (m_pBuffer == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Allocate Encode YUV Buffer Failed !");
    return -5;
  }

  m_pCodecFrame->format = m_pCodecCtx->pix_fmt;
  m_pCodecFrame->width = m_pCodecCtx->width;
  m_pCodecFrame->height = m_pCodecCtx->height;
  m_pCodecFrame->pts = 0;

  ret = av_frame_get_buffer(m_pCodecFrame, 0);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Get Encode AV Frame Buffer Failed !");
    return -1;
  }

  m_pCodecPkt = av_packet_alloc();
  if (m_pCodecPkt == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Allocate Encode AV Packet Failed !");
    return -6;
  }

  return 0;
}

int H265Coder::initDecode() {
  // 寻找编码器
  m_pCodec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
  if (m_pCodec == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Find Decode Codec Failed !");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("h265coder"), "decode name = %s", m_pCodec->name);

  m_pParser = av_parser_init(m_pCodec->id);
  if (m_pParser == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Initial Decode Parser Failed !");
    return -2;
  }

  // 创建编码器上下文
  m_pCodecCtx = avcodec_alloc_context3(m_pCodec);
  if (m_pCodecCtx == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Allocate Decode CodecContext Failed !");
    return -3;
  }

  int ret = 0;
  ret = avcodec_open2(m_pCodecCtx, m_pCodec, NULL);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Allocate Decode Codec Failed: ret = %d !", ret);
    return -4;
  }

  m_pCodecFrame = av_frame_alloc();
  if (m_pCodecFrame == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Allocate Decode AV Frame Failed !");
    return -5;
  }

  m_pCodecPkt = av_packet_alloc();
  if (m_pCodecPkt == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Allocate Decode AV Packet Failed !");
    return -6;
  }

  return 0;
}

int H265Coder::encode(CODER_BUFFER_PTR_VEC &bufferPtr,
                      std::vector<bool> &iFrameFlags) {
  int ret = 0;
  ret = av_frame_make_writable(m_pCodecFrame);
  if (ret < 0) {
    return -1;
  }

  m_pCodecFrame->pts = (++m_frameCnt);
  ret = av_image_fill_arrays(m_pCodecFrame->data, m_pCodecFrame->linesize,
                             m_pBuffer, m_h265CodesConfig.pixelFormatHelper(),
                             m_pCodecFrame->width, m_pCodecFrame->height, 1);
  if (ret < 0) {
    std::cerr << "Fill Encode AV Frame Failed: ret = " << ret << " !"
              << std::endl;
    return -1;
  }

  // std::cerr << "Encode Data linesize[0] = " << m_pCodecFrame->linesize[0]
  //           << ", linesize[1] = " << m_pCodecFrame->linesize[1]
  //           << ", linesize[2] = " << m_pCodecFrame->linesize[2] << std::endl;

  ret = avcodec_send_frame(m_pCodecCtx, m_pCodecFrame);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Send Encode AV Frame Failed: ret = %d !", ret);
    return -2;
  }

  ret = encodeRecv(bufferPtr, iFrameFlags);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Encode Recv Failed: ret = %d !", ret);
    return -3;
  }

  return 0;
}

int H265Coder::decode(CODER_BUFFER_PTR_VEC &bufferPtr) {
  int ret = 0;
  ret = av_packet_from_data(m_pCodecPkt, m_encodeData.data(),
                            m_encodeData.size());
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Parser Decode AV Packet Failed: ret = %d !", ret);
    return -1;
  }

  if (m_pCodecPkt->size > 0) {
    ret = avcodec_send_packet(m_pCodecCtx, m_pCodecPkt);
    if (ret < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Send Decode AV Packet Failed: ret = %d !", ret);
      return -2;
    }

    ret = decodeRecv(bufferPtr);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Decode Recv Failed: ret = %d !", ret);
      return -3;
    }
  }

  return 0;
}

int H265Coder::encodeRecv(CODER_BUFFER_PTR_VEC &vecBufferPtr,
                          std::vector<bool> &iFrameFlags) {
  int ret = 0;
  while (ret >= 0) {
    ret = avcodec_receive_packet(m_pCodecCtx, m_pCodecPkt);
    if (ret == AVERROR(EAGAIN)) {
      // std::cerr << "Encode Try Again Warning !" << std::endl;
      break;
    } else if (ret == AVERROR_EOF) {
      std::cerr << "Encode EOF Error: ret = " << ret << " !" << std::endl;
      break;
    } else if (ret < 0) {
      std::cerr << "Encode Other Error: ret = " << ret << " !" << std::endl;
      return -1;
    }

    // 判断是否为I帧
    bool is_iframe = (m_pCodecPkt->flags & AV_PKT_FLAG_KEY) ? true : false;

    const int encodeLen = m_pCodecPkt->size;
    CODER_BUFFER_PTR bufferPtr(new CODER_BUFFER);
    bufferPtr->resize(encodeLen, '\0');
    memcpy(bufferPtr->data(), m_pCodecPkt->data, encodeLen);
    vecBufferPtr.push_back(bufferPtr);
    iFrameFlags.push_back(is_iframe);

    std::cout << "run here: encodeLen = " << encodeLen
              << ", frameCnt = " << m_frameCnt << std::endl;
  }

  av_packet_unref(m_pCodecPkt);

  return 0;
}

int H265Coder::decodeRecv(CODER_BUFFER_PTR_VEC &vecBufferPtr) {
  vecBufferPtr.clear();

  int ret = 0;
  while (ret >= 0) {
    ret = avcodec_receive_frame(m_pCodecCtx, m_pCodecFrame);
    if (ret == AVERROR(EAGAIN)) {
      // RCLCPP_WARN(rclcpp::get_logger("h265coder"), "Decode Try Again Warning !");
      break;
    } else if (ret == AVERROR_EOF) {
      RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Decode EOF Error: ret = %d !", ret);
      break;
    } else if (ret < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("h265coder"), "Decode Other Error: ret = %d !", ret);
      return -1;
    }

    // std::cout << "m_pCodecFrame format = " << m_pCodecFrame->format
    //           << std::endl;

    AVPixelFormat codecFramePixFormat =
        static_cast<AVPixelFormat>(m_pCodecFrame->format);
    if (m_frameCnt != m_pCodecCtx->frame_num) {
      CODER_BUFFER_PTR bufferPtr(new CODER_BUFFER());
      if (m_h265CodesConfig.imageFrameFormat ==
              robosense::common::FRAME_FORMAT_YUV420P &&
          codecFramePixFormat == AV_PIX_FMT_YUV420P) {
        const int height = m_pCodecFrame->height;
        const int width = m_pCodecFrame->width;
        const int halfHeight = height / 2;
        const int halfWidth = width / 2;

        const int decodeYLen = width * height;
        const int decodeULen = halfWidth * halfHeight;
        const int decodeVLen = halfWidth * halfHeight;
        bufferPtr->resize(decodeYLen + decodeULen + decodeVLen, '\0');
        unsigned char *pYBuffer = bufferPtr->data();
        unsigned char *pUBuffer = bufferPtr->data() + decodeYLen;
        unsigned char *pVBuffer = bufferPtr->data() + decodeYLen + decodeULen;

        int y_offset = 0;
        for (int i = 0; i < height; ++i) {
          memcpy(pYBuffer + y_offset,
                 m_pCodecFrame->data[0] + m_pCodecFrame->linesize[0] * i,
                 width);
          y_offset += width;
        }

        int uv_offset = 0;
        for (int i = 0; i < halfHeight; ++i) {
          memcpy(pUBuffer + uv_offset,
                 m_pCodecFrame->data[1] + m_pCodecFrame->linesize[1] * i,
                 halfWidth);
          memcpy(pVBuffer + uv_offset,
                 m_pCodecFrame->data[2] + m_pCodecFrame->linesize[2] * i,
                 halfWidth);
          uv_offset += halfWidth;
        }
      } else {
        if (m_pSwsCtx == nullptr) {
          switch (m_h265CodesConfig.imageFrameFormat) {
          case robosense::common::FRAME_FORMAT_RGB24: {
            m_pSwsCtx =
                sws_getContext(m_pCodecFrame->width, m_pCodecFrame->height,
                               codecFramePixFormat, m_pCodecFrame->width,
                               m_pCodecFrame->height, AV_PIX_FMT_RGB24,
                               SWS_POINT, NULL, NULL, NULL);

            m_decodeSize =
                av_image_get_buffer_size(AV_PIX_FMT_RGB24, m_pCodecFrame->width,
                                         m_pCodecFrame->height, 1);
            break;
          }
          case robosense::common::FRAME_FORMAT_NV12: {
            m_pSwsCtx =
                sws_getContext(m_pCodecFrame->width, m_pCodecFrame->height,
                               codecFramePixFormat, m_pCodecFrame->width,
                               m_pCodecFrame->height, AV_PIX_FMT_NV12,
                               SWS_POINT, NULL, NULL, NULL);

            m_decodeSize =
                av_image_get_buffer_size(AV_PIX_FMT_NV12, m_pCodecFrame->width,
                                         m_pCodecFrame->height, 1);
            break;
          }
          case robosense::common::FRAME_FORMAT_YUYV422: {
            m_pSwsCtx =
                sws_getContext(m_pCodecFrame->width, m_pCodecFrame->height,
                               codecFramePixFormat, m_pCodecFrame->width,
                               m_pCodecFrame->height, AV_PIX_FMT_YUYV422,
                               SWS_POINT, NULL, NULL, NULL);

            m_decodeSize = av_image_get_buffer_size(AV_PIX_FMT_YUYV422,
                                                    m_pCodecFrame->width,
                                                    m_pCodecFrame->height, 1);
            break;
          }
          default: {
            break;
          }
          }
        }
        bufferPtr->resize(m_decodeSize, '\0');

        uint8_t *outData[4];
        int outLinesize[4];
        switch (m_h265CodesConfig.imageFrameFormat) {
        case robosense::common::FRAME_FORMAT_RGB24: {
          outData[0] = bufferPtr->data();
          outData[1] = NULL;
          outData[2] = NULL;
          outData[3] = NULL;
          outLinesize[0] = m_pCodecFrame->width * 3;
          outLinesize[1] = 0;
          outLinesize[2] = 0;
          outLinesize[3] = 0;
          break;
        }
        case robosense::common::FRAME_FORMAT_NV12: {
          outData[0] = bufferPtr->data();
          outData[1] =
              bufferPtr->data() + m_pCodecFrame->width * m_pCodecFrame->height;
          outData[2] = NULL;
          outData[3] = NULL;
          outLinesize[0] = m_pCodecFrame->width;
          outLinesize[1] = m_pCodecFrame->width;
          outLinesize[2] = 0;
          outLinesize[3] = 0;
          break;
        }
        case robosense::common::FRAME_FORMAT_YUYV422: {
          outData[0] = bufferPtr->data();
          outData[1] = NULL;
          outData[2] = NULL;
          outData[3] = NULL;
          outLinesize[0] = m_pCodecFrame->width * 2;
          outLinesize[1] = 0;
          outLinesize[2] = 0;
          outLinesize[3] = 0;
          break;
        }
        default: {
          break;
        }
        }

        sws_scale(m_pSwsCtx, (const uint8_t *const *)m_pCodecFrame->data,
                  m_pCodecFrame->linesize, 0, m_pCodecCtx->height, outData,
                  outLinesize);
      }

      m_frameCnt = m_pCodecCtx->frame_num;
      vecBufferPtr.push_back(bufferPtr);
    }
  }

  return 0;
}

} // namespace h265
} // namespace robosense
