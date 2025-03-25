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

#include "hyper_vision/devicemanager/devicemanager.h"
robosense::device::DeviceManager::Ptr deviceManagerPtr;
void regDeviceEventCallback(const robosense::device::DeviceEvent &deviceEvent) {
  std::cout << "deviceEvent: "
            << (deviceEvent.event_type == robosense::device::DEVICE_EVENT_ATTACH
                    ? "Attach"
                    : "Detach")
            << ", uuid = "
            << std::string(deviceEvent.uuid, deviceEvent.uuid_size)
            << std::endl;
  if (deviceEvent.event_type == robosense::device::DEVICE_EVENT_ATTACH) {
    const std::string deviceUUID =
        std::string(deviceEvent.uuid, deviceEvent.uuid_size);
    int ret = deviceManagerPtr->openDevice(deviceUUID);
    if (ret != 0) {
      std::cout << "open device: " << deviceUUID << " failed: ret = " << ret
                << std::endl;
    } else {
      std::cout << "open device: " << deviceUUID << " successed !" << std::endl;
    }
  }
}

void regPointCloudCallback(
    const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &msgPtr,
    const std::string &uuid) {
  std::cout << "PointCloud msgPtr timestamp = "
            << std::to_string(msgPtr->timestamp) << ", uuid = " << uuid
            << std::endl;
}

void regImageDataCallback(
    const std::shared_ptr<robosense::lidar::ImageData> &msgPtr,
    const std::string &uuid) {
  std::cout << "ImageData msgPtr timestamp = "
            << std::to_string(msgPtr->timestamp) << ", uuid = " << uuid
            << std::endl;
}

void regImuDataCallback(
    const std::shared_ptr<robosense::lidar::ImuData> &msgPtr,
    const std::string &uuid) {
  std::cout << "ImuData msgPtr timestamp = "
            << std::to_string(msgPtr->timestamp) << ", uuid = " << uuid
            << std::endl;
}

int main(int argc, char **argv) {

  try {
    deviceManagerPtr.reset(new robosense::device::DeviceManager());
  } catch (...) {
    std::cout << "Malloc DeviceManager Failed !" << std::endl;
    return -1;
  }

  deviceManagerPtr->regDeviceEventCallback(
      std::bind(regDeviceEventCallback, std::placeholders::_1));
  deviceManagerPtr->regPointCloudCallback(std::bind(
      regPointCloudCallback, std::placeholders::_1, std::placeholders::_2));
  deviceManagerPtr->regImageDataCallback(std::bind(
      regImageDataCallback, std::placeholders::_1, std::placeholders::_2));
  deviceManagerPtr->regImuDataCallback(std::bind(
      regImuDataCallback, std::placeholders::_1, std::placeholders::_2));

  bool isSuccess = deviceManagerPtr->init();
  if (!isSuccess) {
    std::cout << "DeviceManager Initial !" << std::endl;
    return -2;
  }

  int index = 0;
  while (true && index < 100) {
    std::set<std::string> uuids = deviceManagerPtr->getDevices();
    for (auto iterSet = uuids.begin(); iterSet != uuids.end(); ++iterSet) {
      std::cout << "index = " << index << ", uuid = " << (*iterSet)
                << std::endl;
    }
    std::cout << "index = " << index << std::endl;
    ++index;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  deviceManagerPtr->stop();

  return 0;
}