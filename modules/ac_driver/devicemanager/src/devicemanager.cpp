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

namespace robosense {
namespace device {

DeviceManager::DeviceManager() {
  _usb_ctx = nullptr;
  _kill_handler_thread = 1;
  _start = false;
  _inited = false;
  _event_cb = nullptr;
  _stopProcessingThreads = false;

  // 启动消息处理线程
  _pointCloudProcessingThread =
      std::thread(&DeviceManager::processPointCloudQueue, this);
  _imageProcessingThread = std::thread(&DeviceManager::processImageQueue, this);
  _imuProcessingThread = std::thread(&DeviceManager::processImuQueue, this);
}

DeviceManager::~DeviceManager() { stop(); }

bool DeviceManager::init(const int image_input_fps, const int imu_input_fps,
                         const bool isEnableDebug) {
  if (_inited) {
    return true;
  }
  _is_stoping_ = false;
  _enable_debug = isEnableDebug;
  _image_input_fps = image_input_fps;
  _imu_input_fps = imu_input_fps;

  RS_INFOL << "DeviceManager: _enable_debug = " << _enable_debug << RS_REND;

  int res;
  if (!_usb_ctx) {
    int res = libusb_init(&_usb_ctx);
    if (res < 0) {
      _usb_ctx = nullptr;
      RS_ERROR << "DeviceManager: Initial USB Context Failed: res = " << res
               << RS_REND;
      return false;
    }
  }

  auto thread_usb_event = [this](void *ptr) {
    struct timeval tv = {0, 100000};
    while (!_kill_handler_thread) {
      libusb_handle_events_timeout_completed(_usb_ctx, &tv,
                                             &_kill_handler_thread);
    }
  };

  _kill_handler_thread = 0;
  try {
    _usb_thread = std::thread(thread_usb_event, this);
  } catch (const std::system_error &e) {
    _kill_handler_thread = 0;
    RS_ERROR << "DeviceManager: Create USB Event Thread Failed !" << RS_REND;
    return false;
  }

  _start = true;
  try {
    _m_thread = std::thread(&DeviceManager::hotplugWorkThread, this);
  } catch (const std::system_error &e) {
    _start = false;
    RS_ERROR << "DeviceManager: Create Device Hotplug Thread Failed !"
             << RS_REND;
    return false;
  }
  _inited = true;

  return true;
}

int DeviceManager::stop() {
  if (!_inited) {
    return 0;
  }
  _is_stoping_ = true;

  if (_start) {
    _start = false;
    if (_m_thread.joinable()) {
      _m_thread.join();
    }
  }

  int ret = closeDevices();
  if (ret != 0) {
    RS_ERROR << "DeviceManager: Close All Devices Failed !" << RS_REND;
    return -1;
  } else {
    RS_INFOL << "DeviceManager: Close All Device Successed !" << RS_REND;
  }

  if (!_kill_handler_thread) {
    _kill_handler_thread = 1;
    if (_usb_thread.joinable()) {
      _usb_thread.join();
    }
  }

  if (_usb_ctx) {
    libusb_exit(_usb_ctx);
    _usb_ctx = nullptr;
  }

  if (_stopProcessingThreads == false) {
    _stopProcessingThreads = true;
    _pointCloudQueueCond.notify_all();
    if (_pointCloudProcessingThread.joinable()) {
      _pointCloudProcessingThread.join();
    }
    _imageQueueCond.notify_all();
    if (_imageProcessingThread.joinable()) {
      _imageProcessingThread.join();
    }
    _imuQueueCond.notify_all();
    if (_imuProcessingThread.joinable()) {
      _imuProcessingThread.join();
    }
  }

  _inited = false;

  return 0;
}

int DeviceManager::openDevice(const std::string &device_uuid) {
  std::lock_guard<std::mutex> lg(_devices_map_mtx);
  auto iterMap = _devices_map.find(device_uuid);
  if (iterMap == _devices_map.end()) {
    RS_ERROR << "DeviceManager: device uuid = " << device_uuid
             << " Not Attach !" << RS_REND;
    return -1;
  } else if (iterMap->second.driver_ptr != nullptr) {
    RS_INFOL << "DeviceManager: device uuid = " << device_uuid
             << " Already Open, Not Need Open Again !" << RS_REND;
    return 0;
  }

  RS_DEVICE_DRIVER_PTR driver_ptr;
  try {
    driver_ptr.reset(new RS_DEVICE_DRIVER());
  } catch (...) {
    RS_ERROR << "DeviceManager: device uuid = " << device_uuid
             << " Malloc Device Driver Failed !" << RS_REND;
    return -2;
  }
  std::weak_ptr<DeviceManager> weak_this = shared_from_this();

  const std::string &regDeviceUUID = device_uuid;
  const auto &get_pc_cb =
      std::bind(&DeviceManager::localGetPointCloudCallback, this);
  const auto &put_pc_cb =
      [weak_this, regDeviceUUID](
          const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &msgPtr) {
        // AERROR << "RUN HERE";
        auto share_this = weak_this.lock();
        if (!share_this || !msgPtr) {
          return;
        }
        // AERROR << "RUN HERE";
        share_this->localRunPointCloudCallback(msgPtr, regDeviceUUID);
      };
  driver_ptr->regPointCloudCallback(get_pc_cb, put_pc_cb);

  const auto &get_image_cb =
      std::bind(&DeviceManager::localGetImageDataCallback, this);
  const auto &put_image_cb =
      [weak_this, regDeviceUUID](
          const std::shared_ptr<robosense::lidar::ImageData> &msgPtr) {
        // AERROR << "RUN HERE";
        auto share_this = weak_this.lock();
        if (!share_this || !msgPtr) {
          return;
        }
        // AERROR << "RUN HERE";
        share_this->localRunImageCallback(msgPtr, regDeviceUUID);
      };
  driver_ptr->regImageDataCallback(get_image_cb, put_image_cb);

  const auto &get_imu_cb =
      std::bind(&DeviceManager::localGetImuDataCallback, this);
  const auto &put_imu_cb =
      [weak_this, regDeviceUUID](
          const std::shared_ptr<robosense::lidar::ImuData> &msgPtr) {
        // AERROR << "RUN HERE";
        auto share_this = weak_this.lock();
        if (!share_this || !msgPtr) {
          return;
        }
        // AERROR << "RUN HERE";
        share_this->localRunImuCallback(msgPtr, regDeviceUUID);
      };
  driver_ptr->regImuDataCallback(get_imu_cb, put_imu_cb);

  robosense::lidar::RSDriverParam driverParam;
  driverParam.input_type = robosense::lidar::InputType::USB;
  driverParam.lidar_type = robosense::lidar::RS_AC1;
  driverParam.input_param.enable_image = true; // enable image output
#if defined(RK3588) || defined(JETSON_ORIN) // 当前只针对rk3588对nv12转码做了优化，默认出nv12
  driverParam.input_param.image_format =
      robosense::lidar::FRAME_FORMAT_NV12; // 设置为RGB24/NV12/BGR24
#else
  driverParam.input_param.image_format =
    robosense::lidar::FRAME_FORMAT_RGB24; // 设置为RGB24/NV12/BGR24
#endif
  driverParam.input_param.device_uuid = iterMap->second.uuid;
  driverParam.input_param.image_fps = _image_input_fps;
  driverParam.input_param.imu_fps = _imu_input_fps;

  bool isSuccess = driver_ptr->init(driverParam);
  if (!isSuccess) {
    RS_ERROR << "DeviceManager: device uuid = " << device_uuid
             << " Inital Failed !" << RS_REND;
    return -3;
  }

  isSuccess = driver_ptr->start();
  if (!isSuccess) {
    RS_ERROR << "DeviceManager: device uuid = " << device_uuid
             << " Start Failed !" << RS_REND;
    return -4;
  }

  iterMap->second.is_pause = false;
  iterMap->second.driver_ptr = driver_ptr;

  return 0;
}

int DeviceManager::closeDevice(const std::string &device_uuid,
                               const bool isMetux) {
  if (isMetux) {
    RS_DEVICE_DRIVER_PTR driver_ptr;
    {
      std::lock_guard<std::mutex> lg(_devices_map_mtx);
      auto iterMap = _devices_map.find(device_uuid);
      if (iterMap == _devices_map.end()) {
        RS_ERROR << "DeviceManager: Device uuid = " << device_uuid
                 << " Already Detach !" << RS_REND;
        return 0;
      } else if (iterMap->second.driver_ptr == nullptr) {
        RS_INFOL << "DeviceManager: Device uuid = " << device_uuid
                 << " Not Open, So Not Need Close !" << RS_REND;
        return 0;
      }
      driver_ptr = iterMap->second.driver_ptr;
    }

    if (driver_ptr != nullptr) {
      driver_ptr->stop();
      driver_ptr.reset();
    }

    {
      std::lock_guard<std::mutex> lg(_devices_map_mtx);
      auto iterMap = _devices_map.find(device_uuid);
      if (iterMap != _devices_map.end()) {
        iterMap->second.driver_ptr.reset();
      }
    }
  } else {
    auto iterMap = _devices_map.find(device_uuid);
    if (iterMap == _devices_map.end()) {
      RS_ERROR << "DeviceManager: Device uuid = " << device_uuid
               << " Already Detach !" << RS_REND;
      return 0;
    } else if (iterMap->second.driver_ptr == nullptr) {
      RS_INFOL << "DeviceManager: Device uuid = " << device_uuid
               << " Not Open, So Not Need Close !" << RS_REND;
      return 0;
    }
    iterMap->second.driver_ptr->stop();
    iterMap->second.driver_ptr.reset();
  }
  return 0;
}

int DeviceManager::pauseDevice(const std::string &device_uuid,
                               const bool isPauseOp) {
  std::lock_guard<std::mutex> lg(_devices_map_mtx);
  auto iterMap = _devices_map.find(device_uuid);
  if (iterMap == _devices_map.end()) {
    RS_ERROR << "DeviceManager: Device uuid = " << device_uuid
             << " Not Attach, So Not Need " << (isPauseOp ? "Pause" : "Play")
             << " !" << RS_REND;
    return -1;
  }
  iterMap->second.is_pause = isPauseOp;

  return 0;
}

int DeviceManager::closeDevices() {
  std::lock_guard<std::mutex> lg(_devices_map_mtx);
  while (!_devices_map.empty()) {
    const std::string &device_uuid = _devices_map.begin()->second.uuid;
    int ret = closeDevice(device_uuid, false);
    if (ret != 0) {
      RS_ERROR << "DeviceManager: device uuid = " << device_uuid
               << " Close Failed: ret = " << ret << RS_REND;
      return -1;
    }
    _devices_map.erase(_devices_map.begin());
  }
  _devices_map.clear();

  return 0;
}

std::set<std::string> DeviceManager::getDevices() {
  std::lock_guard<std::mutex> lg(_devices_map_mtx);
  std::set<std::string> uuids;
  for (auto iterMap = _devices_map.begin(); iterMap != _devices_map.end();
       ++iterMap) {
    const DeviceInfoItem &deviceItem = iterMap->second;
    if (deviceItem.is_attach) {
      uuids.insert(iterMap->first);
    }
  }
  return uuids;
}

bool DeviceManager::getDeviceType(std::string &uuid, bool &is_usb_mode) {
  std::lock_guard<std::mutex> lg(_devices_map_mtx);
  for (auto iterMap = _devices_map.begin(); iterMap != _devices_map.end();
       ++iterMap) {
    const DeviceInfoItem &deviceItem = iterMap->second;
    if (uuid == deviceItem.uuid) {
      is_usb_mode = deviceItem.is_usb_mode;
      return true;
    }
  }

  return false;
}

void DeviceManager::regDeviceEventCallback(
    const RS_DEVICE_EVENT_CALLBACK &event_cb) {
  if (event_cb) {
    _event_cb = event_cb;
  }
}

void DeviceManager::regPointCloudCallback(
    const RS_DEVICE_POINTCLOUD_CALLBACK &pc_cb) {
  if (pc_cb) {
    _pc_cb = pc_cb;
  }
}

void DeviceManager::regImageDataCallback(
    const RS_DEVICE_IMAGE_CALLBACK &image_cb) {
  if (image_cb) {
    _image_cb = image_cb;
  }
}

void DeviceManager::regImuDataCallback(const RS_DEVICE_IMU_CALLBACK &imu_cb) {
  if (imu_cb) {
    _imu_cb = imu_cb;
  }
}

std::shared_ptr<PointCloudT<RsPointXYZIRT>>
DeviceManager::localGetPointCloudCallback() {
  std::shared_ptr<PointCloudT<RsPointXYZIRT>> pointCloudMsgPtr;
  try {
    pointCloudMsgPtr.reset(new PointCloudT<RsPointXYZIRT>());
  } catch (...) {
    return nullptr;
  }
  return pointCloudMsgPtr;
}

std::shared_ptr<robosense::lidar::ImageData>
DeviceManager::localGetImageDataCallback() {
  std::shared_ptr<robosense::lidar::ImageData> imageDataMsgPtr;
  try {
    imageDataMsgPtr.reset(new robosense::lidar::ImageData());
  } catch (...) {
    return nullptr;
  }
  return imageDataMsgPtr;
}

std::shared_ptr<robosense::lidar::ImuData>
DeviceManager::localGetImuDataCallback() {
  std::shared_ptr<robosense::lidar::ImuData> imuDataMsgPtr;
  try {
    imuDataMsgPtr.reset(new robosense::lidar::ImuData());
  } catch (...) {
    return nullptr;
  }
  return imuDataMsgPtr;
}

void DeviceManager::localRunPointCloudCallback(
    const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &msgPtr,
    const std::string &uuid) {
  if (_is_stoping_.load()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lg(_devices_map_mtx);
    if (!(_devices_map.find(uuid) != _devices_map.end() &&
          !_devices_map[uuid].is_pause)) {
      return;
    }
  }
  if (msgPtr) {
    std::lock_guard<std::mutex> lg(_pointCloudQueueMutex);
    _pointCloudQueue.push(std::make_pair(msgPtr, uuid));
    _pointCloudQueueCond.notify_one(); // 通知等待的线程
  }
}

void DeviceManager::localRunImageCallback(
    const std::shared_ptr<robosense::lidar::ImageData> &msgPtr,
    const std::string &uuid) {
  if (_is_stoping_.load()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lg(_devices_map_mtx);
    if (!(_devices_map.find(uuid) != _devices_map.end() &&
          !_devices_map[uuid].is_pause)) {
      return;
    }
  }
  if (msgPtr) {
    std::lock_guard<std::mutex> lg(_imageQueueMutex);
    _imageQueue.push(std::make_pair(msgPtr, uuid));
    _imageQueueCond.notify_one(); // 通知等待的线程
  }
}

void DeviceManager::localRunImuCallback(
    const std::shared_ptr<robosense::lidar::ImuData> &msgPtr,
    const std::string &uuid) {
  if (_is_stoping_.load()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lg(_devices_map_mtx);
    if (!(_devices_map.find(uuid) != _devices_map.end() &&
          !_devices_map[uuid].is_pause)) {
      return;
    }
  }
  if (msgPtr) {
    std::lock_guard<std::mutex> lg(_imuQueueMutex);
    _imuQueue.push(std::make_pair(msgPtr, uuid));
    _imuQueueCond.notify_one(); // 通知等待的线程
  }
}

int DeviceManager::findDevices(
    std::map<std::string, DeviceInfoItem> &devices_map) {
  devices_map.clear();
  libusb_device **usb_dev_list;
  ssize_t num_usb_devices = libusb_get_device_list(_usb_ctx, &usb_dev_list);
  if (num_usb_devices < 0) {
    return -1;
  }

  for (int i = 0; i < num_usb_devices; i++) {
    libusb_device *device = usb_dev_list[i];
    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(device, &desc) != LIBUSB_SUCCESS) {
      continue;
    }
    if (desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID) {
      libusb_device_handle *handle;
      std::string uuid("0");
      std::string compare("0");
      if (libusb_open(device, &handle) == 0) {
        unsigned char serial[256];
        int ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                                                     serial, sizeof(serial));
        if (ret > 0) {
          uuid = std::string(reinterpret_cast<char *>(serial));
        }
        libusb_close(handle);

        // std::cout << "findDevice ===> " << uuid
        //           << ", i_sn = " << desc.iSerialNumber << std::endl;

        // 获取uuid 成功
        if (uuid != compare && devices_map.find(uuid) == devices_map.end()) {
          DeviceInfoItem dev_i;
          // dev_i.dev = device;
          dev_i.is_attach = true;
          dev_i.uuid = uuid;
          dev_i.i_sn = desc.iSerialNumber;
          dev_i.driver_ptr = nullptr;
          if (desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID) {
            dev_i.is_usb_mode = true;
          } else {
            dev_i.is_usb_mode = false;
          }
          // libusb_ref_device(device);
          devices_map.insert({uuid, dev_i});
        }
      }
    }
  }
  libusb_free_device_list(usb_dev_list, 1);

  return 0;
}

void DeviceManager::hotplugWorkThread() {
  std::vector<DeviceInfoItem> attachDevices;
  std::vector<DeviceInfoItem> detachDevices;

  while (_start) {
    // Step1: Search Device(s)
    std::map<std::string, DeviceInfoItem> deviceItems;
    int ret = findDevices(deviceItems);
    if (ret != 0) {
      RS_ERROR << "find Devices Failed: ret = " << ret << RS_REND;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    // std::cout << "deviceItems size = " << deviceItems.size() << std::endl;

    attachDevices.clear();
    detachDevices.clear();
    std::map<std::string, DeviceInfoItem> erase_devices_map;
    {
      std::lock_guard<std::mutex> lg(_devices_map_mtx);
      // Step3: 获取不存在的设备
      for (auto iterMap = _devices_map.begin();
           iterMap != _devices_map.end();) {
        auto iterMap2 = deviceItems.find(iterMap->first);
        if (iterMap2 == deviceItems.end()) {
          // std::cout << "detach: uuid = " << iterMap->first
          //           << ", _devices_map size = " << _devices_map.size()
          //           << std::endl;
          detachDevices.push_back(iterMap->second);
          erase_devices_map.insert({iterMap->first, iterMap->second});
          iterMap = _devices_map.erase(iterMap);
        } else {
          ++iterMap;
        }
      }

      // Step2: 获取新的设备
      for (auto iterMap = deviceItems.begin(); iterMap != deviceItems.end();
           ++iterMap) {
        auto iterMap2 = _devices_map.find(iterMap->second.uuid);
        if (iterMap2 == _devices_map.end()) {
          _devices_map.insert({iterMap->first, iterMap->second});
          attachDevices.push_back(iterMap->second);
          // std::cout << "attach: uuid = " << iterMap->first
          //           << ", _devices_map size = " << _devices_map.size()
          //           << std::endl;
        }
      }
    }

    // 自动删除掉线的设备
    erase_devices_map.clear();

    // Step4: 通知设备事件
    if (_event_cb) {
      // Attach 事件
      for (const auto &item : attachDevices) {
        DeviceEvent_t event;
        event.event_type = robosense::device::DEVICE_EVENT_ATTACH;
        event.uuid_size = std::min(item.uuid.size(), sizeof(event.uuid));
        memcpy(event.uuid, item.uuid.c_str(), event.uuid_size);
        _event_cb(event);
      }
      // Detach 事件
      for (const auto &item : detachDevices) {
        DeviceEvent_t event;
        event.event_type = robosense::device::DEVICE_EVENT_DETACH;
        event.uuid_size = std::min(item.uuid.size(), sizeof(event.uuid));
        memcpy(event.uuid, item.uuid.c_str(), event.uuid_size);
        _event_cb(event);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

void DeviceManager::processPointCloudQueue() {
  while (!_stopProcessingThreads) {
    std::pair<std::shared_ptr<PointCloudT<RsPointXYZIRT>>, std::string> msgPair;
    {
      std::unique_lock<std::mutex> lock(_pointCloudQueueMutex);
      _pointCloudQueueCond.wait(lock, [this] {
        return !_pointCloudQueue.empty() || _stopProcessingThreads;
      });
      if (_stopProcessingThreads)
        break;
      msgPair = _pointCloudQueue.front();
      _pointCloudQueue.pop();
    }
    if (msgPair.first && _pc_cb) {
      _pc_cb(msgPair.first, msgPair.second);
    }
  }
}

void DeviceManager::processImageQueue() {
  while (!_stopProcessingThreads) {
    std::pair<std::shared_ptr<robosense::lidar::ImageData>, std::string>
        msgPair;
    {
      std::unique_lock<std::mutex> lock(_imageQueueMutex);
      _imageQueueCond.wait(lock, [this] {
        return !_imageQueue.empty() || _stopProcessingThreads;
      });
      if (_stopProcessingThreads)
        break;
      msgPair = _imageQueue.front();
      _imageQueue.pop();
    }
    if (msgPair.first && _image_cb) {
      _image_cb(msgPair.first, msgPair.second);
    }
  }
}

void DeviceManager::processImuQueue() {
  while (!_stopProcessingThreads) {
    std::pair<std::shared_ptr<robosense::lidar::ImuData>, std::string> msgPair;
    {
      std::unique_lock<std::mutex> lock(_imuQueueMutex);
      _imuQueueCond.wait(lock, [this] {
        return !_imuQueue.empty() || _stopProcessingThreads;
      });
      if (_stopProcessingThreads)
        break;
      msgPair = _imuQueue.front();
      _imuQueue.pop();
    }
    if (msgPair.first && _imu_cb) {
      _imu_cb(msgPair.first, msgPair.second);
    }
  }
}

} // namespace device
} // namespace robosense
