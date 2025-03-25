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

#ifndef HYPER_VISION_COMMON_BASIC_TYPE_HEADER_H_
#define HYPER_VISION_COMMON_BASIC_TYPE_HEADER_H_

#include <iostream>
#include <array>
#include <vector>
#include <string.h>
#include <stdint.h>
#include <memory>
namespace robosense {
namespace common {

struct Header {
  std::string frame_id;
  uint64_t timestamp{0};
  uint32_t seq_id{0};
  double ToSec() const {
    return static_cast<double>(timestamp) / 1000000000UL;
  }
};

struct Quaternion {
  double x{0.};
  double y{0.};
  double z{0.};
  double w{0.};
};

struct Vector3 {
  double x{0.};
  double y{0.};
  double z{0.};
};

}  // namespace common
}  // namespace robosense

#endif  // HYPER_VISION_COMMON_BASIC_TYPE_HEADER_H_
