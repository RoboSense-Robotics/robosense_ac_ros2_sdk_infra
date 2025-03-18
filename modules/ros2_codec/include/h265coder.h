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

#ifndef H265CODER_H
#define H265CODER_H

#include "colorcodec.h"
#include "h265common.h"

namespace robosense {
namespace h265 {

class H265Coder {
public:
  using Ptr = std::shared_ptr<H265Coder>;
  using ConstPtr = std::shared_ptr<const H265Coder>;
  using CODER_BUFFER = std::vector<unsigned char>;
  using CODER_BUFFER_PTR = std::shared_ptr<CODER_BUFFER>;
  using CODER_BUFFER_PTR_VEC = std::vector<CODER_BUFFER_PTR>;

public:
  H265Coder();
  ~H265Coder();

public:
  int init(const H265CodesConfig &h265CodesConfig);
  void resetFrameId();
  int encode(unsigned char *data, const int dataLen,
             CODER_BUFFER_PTR_VEC &bufferPtr, std::vector<bool> &iFrameFlags);
  int decode(unsigned char *data, const int dataLen,
             CODER_BUFFER_PTR_VEC &bufferPtr);
  int encodeFlush(CODER_BUFFER_PTR_VEC &encodes,
                  std::vector<bool> &iFrameFlags);
  int decodeFlush(CODER_BUFFER_PTR_VEC &decodes);

private:
  int initEncode();
  int initDecode();
  int encode(CODER_BUFFER_PTR_VEC &encodes, std::vector<bool> &iFrameFlags);
  int decode(CODER_BUFFER_PTR_VEC &decodes);
  int encodeRecv(CODER_BUFFER_PTR_VEC &encodes, std::vector<bool> &iFrameFlags);
  int decodeRecv(CODER_BUFFER_PTR_VEC &decodes);

private:
  const AVCodec *m_pCodec;                 // 编码器
  AVCodecParserContext *m_pParser;         //
  AVCodecContext *m_pCodecCtx;             // 编码器上下文
  AVFrame *m_pCodecFrame;                  // 原始数据
  AVPacket *m_pCodecPkt;                   // 编码数据
  SwsContext *m_pSwsCtx;                   // 格式变换
  unsigned char *m_pBuffer;                // 待编码数据缓冲区
  unsigned int m_bufferSize;               // 待编码数据大小
  unsigned int m_sourceSize;               // 输入数据的大小
  std::vector<unsigned char> m_encodeData; // 待解码数据缓冲区
  unsigned int m_decodeSize;               // 解码数据大小
  int m_frameCnt;                          // 帧编码序号

private:
  H265CodesConfig m_h265CodesConfig;
  robosense::color::ColorCodec::Ptr m_colorCodecPtr;
};

} // namespace h265
} // namespace robosense

#endif // H265CODER_H
