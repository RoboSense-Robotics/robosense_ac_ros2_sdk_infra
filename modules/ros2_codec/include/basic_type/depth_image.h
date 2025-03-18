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

#ifndef HYPER_VISION_COMMON_BASIC_TYPE_DEPTH_IMAGE_H_
#define HYPER_VISION_COMMON_BASIC_TYPE_DEPTH_IMAGE_H_

#include "basic_type/DataType.h"
#include "basic_type/header.h"

namespace robosense {
namespace common {

struct DepthImage {
  Header header;

  unsigned int height;
  unsigned int width;
  std::vector<float> data_vec;

  ::DepthImage ToOutDepthImage() {
    ::DepthImage out_depth_image;
    unsigned int size = height * width;
    if (size != data_vec.size()) {
      std::cout << "Depth: size is not equal the data size! size is " << size << ", data size is " << data_vec.size();
    }
    out_depth_image.height = height;
    out_depth_image.width = width;
    out_depth_image.data = data_vec.data();
    out_depth_image.timeStamp = header.timestamp;
    return out_depth_image;
  }
};

}  // namespace common
}  // namespace robosense

#endif  // HYPER_VISION_COMMON_BASIC_TYPE_DEPTH_IMAGE_H_
