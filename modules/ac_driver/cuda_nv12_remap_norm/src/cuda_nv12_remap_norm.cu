#include "cuda_runtime_api.h"
#include <stdint.h>
#include <cuda_nv12_remap_norm.h>
#include <iostream>
#include <chrono>

static constexpr uint8_t BASE_OFFSET_BITS = 7;
static constexpr uint8_t EXTRA_OFFSET_BITS = 8;
static constexpr uint8_t OFFSET_BITS = BASE_OFFSET_BITS + EXTRA_OFFSET_BITS; // 15
static constexpr int32_t OFFSET = static_cast<int32_t>(1 << BASE_OFFSET_BITS); // 128
static constexpr float SCALE  = static_cast<float>(1 << OFFSET_BITS); // 32768
static constexpr float SCALE_INV  = 1 / SCALE; // 1/32768

/*
YUV to RGB 参考BT601标准, full range：公式如下：
R = Y + 1.402 * (V - 128)
G = Y − 0.34414 * (U − 128)− 0.71414 * ( V − 128)
B = Y + 1.772 * (U − 128)
*/
// static constexpr int32_t Y_COEFF  = static_cast<int32_t>(1.0 * SCALE); // 32768
// static constexpr int32_t OY       = 0;
// static constexpr float   UV_COEFF = 1.0;
// static constexpr int32_t R_COEFF  = static_cast<int32_t>(1.402 * SCALE); // 45940
// static constexpr int32_t G_COEFF1 = static_cast<int32_t>(0.34413 * SCALE); // 11276
// static constexpr int32_t G_COEFF2 = static_cast<int32_t>(0.71414 * SCALE); // 23400
// static constexpr int32_t B_COEFF  = static_cast<int32_t>(1.772 * SCALE); // 58064

/*
YUV to RGB 参考BT601标准, limited range：公式如下：
R = (255 / 219) * (Y-16) + (255/224) * 1.402 * (V - 128)
G = (255 / 219) * (Y-16) − (255/224) * 0.34414 * (U − 128) − (255/224) * 0.71414 * (V−128)
B = (255 / 219) * (Y-16) + (255/224) * 1.772 * (U − 128)
*/
static constexpr int32_t Y_COEFF  = static_cast<int32_t>(255.0 / 219.0 * SCALE); // 38154
static constexpr int32_t OY       = 16;
static constexpr float   UV_COEFF = 255.0 / 224.0;
static constexpr int32_t R_COEFF  = static_cast<int32_t>(UV_COEFF * 1.402 * SCALE); // 52298
static constexpr int32_t G_COEFF1 = static_cast<int32_t>(UV_COEFF * 0.34413 * SCALE); // 12837
static constexpr int32_t G_COEFF2 = static_cast<int32_t>(UV_COEFF * 0.71414 * SCALE); // 26639
static constexpr int32_t B_COEFF  = static_cast<int32_t>(UV_COEFF * 1.772 * SCALE); // 66100

/*
YUV to RGB 参考BT709标准, full range：公式如下：
R = Y + 1.5748 * (V - 128)
G = Y − 0.1868 * (U − 128) − 0.4680 * ( V − 128)
B = Y + 1.856  * (U − 128)
*/
// static constexpr int32_t Y_COEFF  = static_cast<int32_t>(1.0 * SCALE); // 32768
// static constexpr int32_t OY       = 0;
// static constexpr float   UV_COEFF = 1;
// static constexpr int32_t R_COEFF  = static_cast<int32_t>(UV_COEFF * 1.5748 * SCALE); // 51603
// static constexpr int32_t G_COEFF1 = static_cast<int32_t>(UV_COEFF * 0.1868 * SCALE); // 6121
// static constexpr int32_t G_COEFF2 = static_cast<int32_t>(UV_COEFF * 0.4680 * SCALE); // 15335
// static constexpr int32_t B_COEFF  = static_cast<int32_t>(UV_COEFF * 1.856 * SCALE); // 60817
template<typename _T>
static __host__ __device__ __forceinline__ uint8_t u8cast(_T value){
    return value < 0 ? 0 : (value >= 255 ? 255 : value);
}

template <typename T>
__global__ void REMAP_TO_RGB8_GRAY_CUDA(const uint8_t* src, const int16_t *map_xy, const float* mean,
  const float* std_inv, const float* max_v, int src_width, int src_height, int dst_width, int dst_height, uint8_t* rgb, uint8_t* gray, YUVType type) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    int batch = blockIdx.z;
    const int src_size = src_width * src_height;
    const int dst_size = dst_width * dst_height;
    const int dst_batch_offset = batch * dst_size;
    const int src_batch_offset = batch * (type == NV12 ? (src_size * 3 / 2) : (src_size << 1));

    if (x < dst_width && y < dst_height) {
        int index_dst = y * dst_width + x;

        int16_t index_src_x = map_xy[(index_dst + dst_batch_offset)*2];
        int16_t index_src_y = map_xy[(index_dst + dst_batch_offset)*2 + 1];

        if (index_src_x < 0 || index_src_y < 0 || index_src_x >= src_width || index_src_y >= src_height) {
            rgb[index_dst + dst_batch_offset*3] = u8cast( (0 - mean[0]) * std_inv[0] ); // B
            rgb[index_dst + dst_batch_offset*3 + dst_size] = u8cast((0 - mean[1]) * std_inv[1]) ; // G
            rgb[index_dst + dst_batch_offset*3 + dst_size*2] = u8cast((0 - mean[2]) * std_inv[2]); // R
            if (gray != nullptr) {
                gray[index_dst + dst_batch_offset] = 0;
            }
        } else {
            int index_src;
            uint8_t Y, U, V;

            if (type == NV12) {
                index_src = index_src_y * src_width + index_src_x + src_batch_offset;
                int index_src_uv = (index_src_y / 2) * src_width + (index_src_x / 2) * 2 + src_batch_offset;
                Y = src[index_src];
                U = src[src_size + index_src_uv];
                V = src[src_size + index_src_uv + 1];
            } else {
                index_src = index_src_y * src_width * 2 + index_src_x * 2 + src_batch_offset;
                Y = src[index_src];
                U = src[index_src_x % 2 == 0 ? index_src + 1 : index_src - 1];
                V = src[index_src_x % 2 == 0 ? index_src + 3 : index_src + 1];
            }

            if (gray != nullptr) {
                gray[index_dst + dst_batch_offset] = Y;
            }

            int32_t Y_16 = static_cast<int32_t>(Y - OY);
            T tmp_R = static_cast<T>((Y_COEFF * Y_16 + (R_COEFF * (static_cast<int32_t>(V) - OFFSET))));
            T tmp_G = static_cast<T>((Y_COEFF * Y_16 - (G_COEFF1 * (static_cast<int32_t>(U) - OFFSET) + G_COEFF2 * (static_cast<int32_t>(V) - OFFSET))));
            T tmp_B = static_cast<T>((Y_COEFF * Y_16 + (B_COEFF * (static_cast<int32_t>(U) - OFFSET))));

            // 饱和截断处理
            tmp_R = tmp_R < 0 ? 0 : (tmp_R > (*max_v) ? (*max_v) : tmp_R);
            tmp_G = tmp_G < 0 ? 0 : (tmp_G > (*max_v) ? (*max_v) : tmp_G);
            tmp_B = tmp_B < 0 ? 0 : (tmp_B > (*max_v) ? (*max_v) : tmp_B);

            index_dst += dst_batch_offset*3;
            //rgb[index_dst] = (tmp_B - mean[0]) * std_inv[0];
            //rgb[index_dst + dst_size] = (tmp_G - mean[1]) * std_inv[1];
            //rgb[index_dst + dst_size*2] = (tmp_R - mean[2]) * std_inv[2];
            // for show rgb picture
            rgb[index_dst*3 + dst_batch_offset*3] = u8cast((tmp_R - mean[0]) * std_inv[0]);;
            rgb[index_dst* 3+1 + dst_batch_offset*3] = u8cast((tmp_G - mean[1]) * std_inv[1]);
            rgb[index_dst* 3+2 + dst_batch_offset*3] = u8cast((tmp_B - mean[2]) * std_inv[2]);
        }
    }
}

void NV12ToYUV422(const void* nv12, int width, int height, void* yuv422) {
    const uint8_t* src = (const uint8_t*)nv12;
    uint8_t* dst = (uint8_t*)yuv422;

    int frame_size = width * height;
    int uv_offset = frame_size;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x += 2) {
            // Y values
            uint8_t Y1 = src[y * width + x];
            uint8_t Y2 = src[y * width + x + 1];

            // UV values (NV12 has interleaved UV data for each 2x2 block)
            int uv_index = (y / 2) * width + x;
            uint8_t U = src[uv_offset + uv_index];
            uint8_t V = src[uv_offset + uv_index + 1];

            // YUV422 stores data in YUVY format for 2 pixels
            int output_index = (y * width + x) * 2;
            dst[output_index] = Y1;       // Y1
            dst[output_index + 1] = U;    // U
            dst[output_index + 2] = Y2;   // Y2
            dst[output_index + 3] = V;    // V
        }
    }
}

extern "C" int32_t YUVRemapAndSplit(YUVRemapAndSplitParameters& param, const void* yuv_image, const void* map_xy,
  void *gray, void *image_nchw, void* workspace, cudaStream_t stream, YUVType yuv_type)
{
  // dim3 block_dim(16, 16);
  dim3 block_dim(128, 1);
  dim3 grid_dim((param.output_w + block_dim.x - 1) / block_dim.x, (param.output_h + block_dim.y - 1) / block_dim.y, param.batchSize);
  uint8_t *workspace_ = (uint8_t *)workspace;
  float *mean_ = (float *)workspace_;
  workspace_ += 4 * sizeof(float);
  float *std_inv_ = (float *)workspace_;
  workspace_ += 4 * sizeof(float);
  float *max_v_ = (float *)workspace_;
  workspace_ += 1 * sizeof(float);

#ifdef __aarch64__
  float *mean_stdinv_float_t = mean_;
  float *stdinv_stdinv_float = std_inv_;
  float *max_v = max_v_;
#else
  float mean_stdinv_float_t[4];
  float stdinv_stdinv_float[4];
  float max_v[1];
#endif
  mean_stdinv_float_t[0] = static_cast<float>(param.mean[0]) * SCALE;
  mean_stdinv_float_t[1] = static_cast<float>(param.mean[1]) * SCALE;
  mean_stdinv_float_t[2] = static_cast<float>(param.mean[2]) * SCALE;

  stdinv_stdinv_float[0] = static_cast<float>(param.std_inv[0]) * SCALE_INV;
  stdinv_stdinv_float[1] = static_cast<float>(param.std_inv[1]) * SCALE_INV;
  stdinv_stdinv_float[2] = static_cast<float>(param.std_inv[2]) * SCALE_INV;
  max_v[0] = static_cast<float>(UINT8_MAX * SCALE);

#ifdef __aarch64__
  // Nothing
  // memcpy(mean_, &mean_stdinv_int16_t[0], 8*sizeof(int16_t));
#else
  cudaMemcpyAsync(mean_, &mean_stdinv_float_t[0], 4*sizeof(float), cudaMemcpyHostToDevice, stream);
  cudaMemcpyAsync(std_inv_, &stdinv_stdinv_float[0], 4*sizeof(float), cudaMemcpyHostToDevice, stream);
  cudaMemcpyAsync(max_v_, &max_v[0], 1 * sizeof(float), cudaMemcpyHostToDevice, stream);
#endif
  // cudaMemcpy(std_inv_, &std_inv_int16[0], 3*sizeof(int16_t), cudaMemcpyHostToDevice);
#if 1
  // 检查常量
  // printf("%d\n", UINT8_MAX);
  // printf("%d\n", BASE_OFFSET_BITS);
  // printf("%d\n", EXTRA_OFFSET_BITS);
  // printf("%d\n", OFFSET_BITS);

  // printf("%d\n", Y_COEFF);
  // printf("%d\n", OY);
  // printf("%f\n", UV_COEFF);
  // printf("%d\n", OFFSET);
  // printf("%f\n", SCALE);
  // printf("%f\n", SCALE_INV);
  // printf("%d\n", R_COEFF);
  // printf("%d\n", G_COEFF1);
  // printf("%d\n", G_COEFF2);
  // printf("%d\n", B_COEFF);
  // printf("%f\n", *max_v);
  REMAP_TO_RGB8_GRAY_CUDA<float><<<grid_dim, block_dim, 0, stream>>>((const uint8_t *)yuv_image, (const int16_t *)map_xy, mean_, std_inv_, max_v_,
    param.intput_w, param.intput_h, param.output_w, param.output_h, (uint8_t *)image_nchw, (uint8_t *)gray, yuv_type);

#else
  constexpr int32_t test_count = 100;
  auto time_2 = std::chrono::steady_clock::now();
  for (int i = 0; i < test_count; ++i) {
    // Launch the CUDA kernel
    NV12_REMAP_TO_RGB8_GRAY_CUDA<float><<<grid_dim, block_dim, 0, stream>>>((const uint8_t *)yuv_image, (const int16_t *)map_xy, mean_, std_inv_,
      param.intput_w, param.intput_h, param.output_w, param.output_h, (float *)image_nchw, (uint8_t *)gray);
  }

  // 同步CUDA流
  cudaStreamSynchronize(stream);

  auto time_3 = std::chrono::steady_clock::now();
  auto time_d1 = std::chrono::duration_cast<std::chrono::microseconds>((time_3 - time_2)).count();
  std::cout << "cuda remap + convert: " << time_d1 / 1000.0 / test_count << " ms" << std::endl;
#endif
  cudaError_t status = cudaGetLastError();
  if (!status) {
    return status;
  }

  return 0;
}
