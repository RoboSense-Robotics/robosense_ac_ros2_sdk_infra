
#ifndef CUDA_NV12_REMAP_NORM_H
#define CUDA_NV12_REMAP_NORM_H

#include <cuda_runtime_api.h>
#include <stdint.h>

struct YUVRemapAndSplitParameters
{
    int32_t intput_w;
    int32_t intput_h;
    int32_t output_w;
    int32_t output_h;
    float mean[3] = {0.0, 0.0, 0.0};
    float std_inv[3] = {1.0, 1.0, 1.0};

    int batchSize = 1;
};

enum YUVType {
    YUV422,
    NV12, //YUV420sp
};

extern "C" int32_t YUVRemapAndSplit(YUVRemapAndSplitParameters& param, const void* yuv_image, const void* map_xy,
  void *gray, void *image_nchw, void* workspace, cudaStream_t stream, YUVType yuv_type);
void NV12ToYUV422(const void* nv12, int width, int height, void* yuv422);

#endif //CUDA_NV12_REMAP_NORM_H
