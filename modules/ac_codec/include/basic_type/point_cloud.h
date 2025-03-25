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

#ifndef HYPER_VISION_COMMON_BASIC_TYPE_POINT_CLOUD_H_
#define HYPER_VISION_COMMON_BASIC_TYPE_POINT_CLOUD_H_

#include "basic_type/DataType.h"

namespace robosense {
namespace common {

struct PointCloud {
  std::vector<Point> points_vec;

  ::PointCloud ToOutPointCloud() {
    ::Tag_PointCloud pc;
    pc.data = points_vec.data();
    pc.size = static_cast<int>(points_vec.size());
    return pc;
  }
};

}  // namespace common
}  // namespace robosense

#endif  // HYPER_VISION_COMMON_BASIC_TYPE_POINT_CLOUD_H_
