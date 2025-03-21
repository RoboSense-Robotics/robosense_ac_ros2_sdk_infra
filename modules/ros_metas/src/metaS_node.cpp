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

/**
 * @file metaS_node.cpp
 * @brief ROS/ROS2 Node for publishing RGB, depth, and IMU data from a
 * SuperSense device.
 *
 * This node retrieves data from a metaS device and publishes it to ROS/ROS2
 * topics:
 * - RGB images on the "/rs_camera/rgb" topic
 * - Depth point clouds on the "/rs_lidar/points" topic
 * - IMU data on the "/rs_imu" topic
 */

#include "hyper_vision/devicemanager/devicemanager.h"
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

#ifdef RK3588
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
#include "rga/RgaUtils.h"
#include "rga/im2d.hpp"
#else
#include "colorcodec.h"
#include "jpegcoder.h"
#endif

#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <robosense_msgs/msg/rs_compressed_image.hpp>
#include <robosense_msgs/msg/rs_image.hpp>
#include <robosense_msgs/msg/rs_point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#endif

#define DEBUG_STATISTICS
//#define DEBUG_TO_FILE

class MSPublisher
#ifdef ROS2_FOUND
    : public rclcpp::Node
#endif

{
public:
  /**
   * @brief Constructor initializes the node, sets up publishers, and starts the
   * device streams.
   */
  MSPublisher()
#ifdef ROS2_FOUND
      : Node("ms_node")
#endif
  {
#ifdef ROS2_FOUND
    const char* val = std::getenv("RMW_FASTRTPS_USE_QOS_FROM_XML");
    if (val != nullptr && std::string(val) == "1") {
      zero_copy = true;
    } else {
      zero_copy = false;
    }
#endif
#ifdef DEBUG_STATISTICS
    auto point_options = rclcpp::SubscriptionOptions();
    point_options.topic_stats_options.state =
        rclcpp::TopicStatisticsState::Enable;
    point_options.topic_stats_options.publish_topic = "PointCloud";
    point_options.topic_stats_options.publish_period = std::chrono::seconds(10);
    auto point_callback = [this](sensor_msgs::msg::PointCloud2 msg) {
      this->topic_point_callback(msg);
    };
    if(!zero_copy) {
      point_subscription_ =
          this->create_subscription<sensor_msgs::msg::PointCloud2>(
              "/rs_lidar/points", 10, point_callback, point_options);
    }

    auto imu_options = rclcpp::SubscriptionOptions();
    imu_options.topic_stats_options.state =
        rclcpp::TopicStatisticsState::Enable;
    imu_options.topic_stats_options.publish_topic = "Imu";
    imu_options.topic_stats_options.publish_period = std::chrono::seconds(10);
    auto imu_callback = [this](sensor_msgs::msg::Imu msg) {
      this->topic_imu_callback(msg);
    };
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/rs_imu", 10, imu_callback, imu_options);

    auto frame_options = rclcpp::SubscriptionOptions();
    frame_options.topic_stats_options.state =
        rclcpp::TopicStatisticsState::Enable;
    frame_options.topic_stats_options.publish_topic = "CameraFrame";
    frame_options.topic_stats_options.publish_period = std::chrono::seconds(10);
    auto frame_callback = [this](robosense_msgs::msg::RsCompressedImage msg) {
      this->topic_frame_callback(msg);
    };
    frame_subscription_ =
        this->create_subscription<robosense_msgs::msg::RsCompressedImage>(
            "/rs_camera/compressed", 10, frame_callback,
            frame_options);
#endif // DEBUG_STATISTICS

    // Initialize publishers for RGB image, depth point cloud, and IMU data
#ifdef ROS_FOUND
    ros::NodeHandle nh;
    publisher_rgb = nh.advertise<sensor_msgs::Image>("/rs_camera/rgb", 10);
    publisher_depth =
        nh.advertise<sensor_msgs::PointCloud2>("/rs_lidar/points", 10);
    publisher_imu = nh.advertise<sensor_msgs::Imu>("/rs_imu", 10);
#elif ROS2_FOUND
    if(zero_copy) {
      publisher_rgb_loan =
          this->create_publisher<robosense_msgs::msg::RsImage>("/rs_camera/rgb", 10);
      publisher_depth_loan = 
          this->create_publisher<robosense_msgs::msg::RsPointCloud>(
          "/rs_lidar/points", 10);
    }
    else {
      publisher_rgb =
          this->create_publisher<sensor_msgs::msg::Image>("/rs_camera/rgb", 10);
      publisher_depth = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/rs_lidar/points", 10);
    }
    publisher_imu = this->create_publisher<sensor_msgs::msg::Imu>(
        "/rs_imu", 10);
#ifdef RK3588
    publisher_h265 =
        this->create_publisher<robosense_msgs::msg::RsCompressedImage>(
            "/rs_camera/compressed", 10);
#else
    publisher_jpeg =
        this->create_publisher<robosense_msgs::msg::RsCompressedImage>(
            "/rs_camera/compressed", 10);
#endif // RK3588

#endif // ROS_ROS2_FOUND

    // 图像分辨率信息
    imageWidth = 1920;
    imageHeight = 1080;
#ifdef RK3588
    avdevice_register_all();
    const AVCodec *codec = avcodec_find_encoder_by_name("hevc_rkmpp");
    if (!codec) {
      logError("can not find h.265 codec\n");
      shutdown();
      return;
    }

    codecContext = avcodec_alloc_context3(codec);
    if (!codecContext) {
      logError("can not alloc avcodec context3\n");
      shutdown();
      return;
    }

    codecContext->bit_rate = 4000000;
    codecContext->width = imageWidth;
    codecContext->height = imageHeight;
    codecContext->pix_fmt = AV_PIX_FMT_NV12;
    codecContext->time_base = (AVRational){1, 30};
    codecContext->framerate = (AVRational){30, 1};
    codecContext->gop_size = 30;
    codecContext->max_b_frames = 1;

    av_frame = av_frame_alloc();
    if (!av_frame) {
      logError("can not alloc av frame\n");
      return;
    }
    av_frame->format = codecContext->pix_fmt;
    av_frame->width = codecContext->width;
    av_frame->height = codecContext->height;
    av_frame->pts = 0;
    if (av_frame_get_buffer(av_frame, 32) < 0) {
      logError("can not get av frame buffer\n");
      return;
    }

    if (avcodec_open2(codecContext, codec, NULL) < 0) {
      logError("can not open avcodec\n");
      shutdown();
      return;
    }

    av_pkt = av_packet_alloc();
    if (!av_pkt) {
      logError("can not allocl av packet\n");
      shutdown();
      return;
    }
#else
    // for X86
    robosense::jpeg::JpegCodesConfig config;
    config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE;
    config.imageFrameFormat = robosense::common::FRAME_FORMAT_NV12;
    config.imageWidth = imageWidth;
    config.imageHeight = imageHeight;
    config.gpuDeviceId = 0;

    int ret = jpegEncoder.init(config);
    if (ret != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "jpeg encoder(nv12) initial failed: ret = %d\n", ret);
      rclcpp::shutdown();
      return;
    }

    nv12_image_size =
        robosense::color::ColorCodec::NV12ImageSize(imageWidth, imageHeight);

#endif // RK3588

#ifdef DEBUG_TO_FILE
    outfile = fopen("test.h265", "wb");
    out_count = 100;
#endif // DEBUG_TO_FILE

    try {
      device_manager_ptr.reset(new robosense::device::DeviceManager());
    } catch (...) {
      logError("Malloc Device Manager Failed !");
      return;
    }

    device_manager_ptr->regDeviceEventCallback(std::bind(
        &MSPublisher::deviceEventCallback, this, std::placeholders::_1));

    device_manager_ptr->regPointCloudCallback(
        std::bind(&MSPublisher::pointCloudCallback, this, std::placeholders::_1,
                  std::placeholders::_2));

    device_manager_ptr->regImageDataCallback(
        std::bind(&MSPublisher::imageCallback, this, std::placeholders::_1,
                  std::placeholders::_2));

    device_manager_ptr->regImuDataCallback(
        std::bind(&MSPublisher::imuCallback, this, std::placeholders::_1,
                  std::placeholders::_2));

    bool isSuccess = device_manager_ptr->init(false);
    if (!isSuccess) {
      logError("Device Manager Initial Failed !");
      return;
    }

    logInfo("Start...");
  }

  /**
   * @brief Destructor cleans up the device object.
   */
  ~MSPublisher() {
    if (device_manager_ptr) {
      device_manager_ptr->stop();
    }
  };

private:
  void deviceEventCallback(const robosense::device::DeviceEvent &deviceEvent) {
    switch (deviceEvent.event_type) {
    case robosense::device::DeviceEventType::DEVICE_EVENT_ATTACH: {
      const std::string &uuid =
          std::string(deviceEvent.uuid, deviceEvent.uuid_size);
      {
        std::lock_guard<std::mutex> lg(current_device_uuid_mtx);
        if (uuid == current_device_uuid) {
          logInfo("Device uuid = " + uuid + " Already Open !");
          return;
        } else if (!current_device_uuid.empty()) {
          logInfo("Current Device uuid = " + current_device_uuid +
                  " Already Open, Attach Device uuid = " + uuid +
                  " Not Need Open Again !");
          return;
        }
      }

      int ret = device_manager_ptr->openDevice(uuid);
      if (ret != 0) {
        logError("Device uuid = " + uuid +
                 " Open Device Failed: ret = " + std::to_string(ret));
        return;
      }

      {
        std::lock_guard<std::mutex> lg(current_device_uuid_mtx);
        current_device_uuid = uuid;
        logInfo("Device uuid = " + uuid + " Open Successed !");
      }

      break;
    }
    case robosense::device::DeviceEventType::DEVICE_EVENT_DETACH: {
      const std::string &uuid =
          std::string(deviceEvent.uuid, deviceEvent.uuid_size);
      {
        std::lock_guard<std::mutex> lg(current_device_uuid_mtx);
        if (uuid != current_device_uuid || current_device_uuid.empty()) {
          logInfo("Device uuid = " + uuid + " Detach But Not Need Processed !");
          return;
        }
      }

      int ret = device_manager_ptr->closeDevice(uuid, true);
      if (ret != 0) {
        logError("Device uuid = " + uuid +
                 " Detach Close Failed: ret = " + std::to_string(ret));
        return;
      }

      {
        std::lock_guard<std::mutex> lg(current_device_uuid_mtx);
        current_device_uuid.clear();
        logInfo("Device uuid = " + uuid + " Close Successed !");
      }

      break;
    }
    default: {
      break;
    }
    }
  }

  void
  pointCloudCallback(const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &msgPtr,
                     const std::string &uuid) {
    (void)(uuid);
    if (msgPtr) {
      if (zero_copy) {
        depth_handle_loan(msgPtr);
      }
      else {
        depth_handle(msgPtr);
      }
    }
  }

  void imageCallback(const std::shared_ptr<robosense::lidar::ImageData> &msgPtr,
                     const std::string &uuid) {
    (void)(uuid);
    if (msgPtr) {
      rgb_handle(msgPtr);
#ifdef RK3588
      h265_handle(msgPtr);
#else
      jpeg_handle(msgPtr);
#endif // RK3588
    }
  }

  void imuCallback(const std::shared_ptr<robosense::lidar::ImuData> &msgPtr,
                   const std::string &uuid) {
    (void)(uuid);
    if (msgPtr) {
      imu_handle(msgPtr);
    }
  }

#ifdef RK3588
  void h265_handle(const std::shared_ptr<robosense::lidar::ImageData> &frame) {
    int ret;
    uint64_t timestampNs = frame->timestamp * 1000000000;
    uint32_t sec = timestampNs / 1000000000;
    uint32_t nsec = timestampNs % 1000000000;
#ifdef ROS_FOUND
    auto custom_time = ros::Time(sec, nsec);
    auto h265_msg = std::make_shared<robosense_msgs::msg::RsCompressedImage>();
#elif ROS2_FOUND
    auto custom_time = rclcpp::Time(sec, nsec);
    auto h265_msg = std::make_shared<robosense_msgs::msg::RsCompressedImage>();
#endif // ROS_ROS2_FOUND

    ret = av_frame_make_writable(av_frame);
    if (ret > 0) {
      logError("av frame is not writable\n");
      return;
    }
    memcpy(av_frame->data[0], frame->data.get(),
           frame->width * frame->height); // Y分量
    memcpy(av_frame->data[1], frame->data.get() + frame->width * frame->height,
           frame->width * frame->height / 2); // UV分量

    // av_frame->pts =
    //     (frame->capture_time.tv_sec + frame->capture_time.tv_usec /
    //     1000000.0);
    av_frame->pts = timestampNs;

    ret = avcodec_send_frame(codecContext, av_frame);
    if (ret < 0) {
      fprintf(stderr, "Error sending a frame for encoding\n");
    }

    while (ret >= 0) {
      ret = avcodec_receive_packet(codecContext, av_pkt);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        break;
      else if (ret < 0) {
        fprintf(stderr, "Error during encoding\n");
        break;
      }

      // printf("send %d h265 packet %3"PRId64" (size=%5d)\n", i, av_pkt->pts,
      // av_pkt->size);
#ifdef DEBUG_TO_FILE
      if (out_count > 0) {
        fwrite(av_pkt->data, 1, av_pkt->size, outfile);
        out_count--;
      } else if (out_count == 0) {
        fclose(outfile);
        out_count = -1;
      }
#endif
      // Publish the h254 frame as a robosense message
      h265_msg->header.stamp = custom_time;
      h265_msg->header.frame_id = "h265";
      if (av_pkt->flags & AV_PKT_FLAG_KEY) {
        h265_msg->type = 1; // H265_I
      } else {
        h265_msg->type = 2; // H265_P
      }
      h265_msg->data.resize(av_pkt->size);
      std::memcpy(h265_msg->data.data(), av_pkt->data, av_pkt->size);

#ifdef ROS_FOUND
      publisher_h265.publish(*h265_msg);
#elif ROS2_FOUND
      publisher_h265->publish(*h265_msg);
#endif // ROS_ROS2_FOUND
      av_packet_unref(av_pkt);
    }
  }
#else
  void jpeg_handle(const std::shared_ptr<robosense::lidar::ImageData> &frame) {
    int ret;
    uint64_t timestampNs = frame->timestamp * 1000000000;
    uint32_t sec = timestampNs / 1000000000;
    uint32_t nsec = timestampNs % 1000000000;
#ifdef ROS_FOUND
    auto custom_time = ros::Time(sec, nsec);
    auto jpeg_msg = std::make_shared<robosense_msgs::msg::RsCompressedImage>();
#elif ROS2_FOUND
    auto custom_time = rclcpp::Time(sec, nsec);
    auto jpeg_msg = std::make_shared<robosense_msgs::msg::RsCompressedImage>();
#endif // ROS_ROS2_FOUND
    std::vector<unsigned char> jpegBuffer(imageWidth * imageHeight * 4.5, '\0');

    size_t jpegBufferLen = jpegBuffer.size();
    ret =
        jpegEncoder.encode((unsigned char *)frame->data.get(),
                           frame->data_bytes, jpegBuffer.data(), jpegBufferLen);
    if (ret != 0) {
      fprintf(stderr, "Error jepg encoding\n");
      return;
    }

    // Publish the jpeg frame as a robosense message
    jpeg_msg->header.stamp = custom_time;
    jpeg_msg->header.frame_id = "jpeg nv12";
    jpeg_msg->data.resize(jpegBufferLen);
    std::memcpy(jpeg_msg->data.data(), jpegBuffer.data(), jpegBufferLen);

#ifdef ROS_FOUND
    publisher_jpeg.publish(*jpeg_msg);
#elif ROS2_FOUND
    publisher_jpeg->publish(*jpeg_msg);
#endif // ROS_ROS2_FOUND
  }
#endif // RK3588

  void rgb_handle(const std::shared_ptr<robosense::lidar::ImageData> &frame) {
    uint64_t timestampNs = frame->timestamp * 1000000000;
    uint32_t sec = timestampNs / 1000000000;
    uint32_t nsec = timestampNs % 1000000000;
#ifdef ROS_FOUND
    auto custom_time = ros::Time(sec, nsec);
    auto rgb_msg = std::make_shared<sensor_msgs::Image>();
#elif ROS2_FOUND
    auto custom_time = rclcpp::Time(sec, nsec);

#endif // ROS_ROS2_FOUND

#ifdef RK3588
    // NV12 to RGB24  for rock hw
    int ret = 0;
    int src_format;
    int dst_format;
    char *src_buf, *dst_buf;
    int dst_buf_size;

    rga_buffer_t src_img, dst_img;
    rga_buffer_handle_t src_handle, dst_handle;

    memset(&src_img, 0, sizeof(src_img));
    memset(&dst_img, 0, sizeof(dst_img));

    src_format = RK_FORMAT_YCbCr_420_SP;
    dst_format = RK_FORMAT_RGB_888;

    dst_buf_size =
        frame->width * frame->height * get_bpp_from_format(RK_FORMAT_RGB_888);

    std::vector<uint8_t> rgb_buf(dst_buf_size);

    dst_buf = (char *)rgb_buf.data();

    memset(dst_buf, 0x80, dst_buf_size);

    src_handle = importbuffer_virtualaddr(frame->data.get(), frame->data_bytes);
    dst_handle = importbuffer_virtualaddr(dst_buf, dst_buf_size);
    if (src_handle == 0 || dst_handle == 0) {
      printf("%s importbuffer failed!\n", __func__);
      if (src_handle)
        releasebuffer_handle(src_handle);
      if (dst_handle)
        releasebuffer_handle(dst_handle);
      return;
    }

    src_img =
        wrapbuffer_handle(src_handle, frame->width, frame->height, src_format);
    dst_img =
        wrapbuffer_handle(dst_handle, frame->width, frame->height, dst_format);

    ret = imcheck(src_img, dst_img, {}, {});
    if (IM_STATUS_NOERROR != ret) {
      printf("%s %d, check error! %s", __func__, __LINE__,
             imStrError((IM_STATUS)ret));
      if (src_handle)
        releasebuffer_handle(src_handle);
      if (dst_handle)
        releasebuffer_handle(dst_handle);
      return;
    }

    ret = imcvtcolor(src_img, dst_img, src_format, dst_format);
    if (ret != IM_STATUS_SUCCESS) {
      printf("%s imcvtcolor running failed, %s\n", __func__,
             imStrError((IM_STATUS)ret));
    }

    if (src_handle)
      releasebuffer_handle(src_handle);
    if (dst_handle)
      releasebuffer_handle(dst_handle);
#else
    // NV12 to RGB24  for cpu
    unsigned int required_size;
    unsigned int y_size;
    uint8_t *y_plane;
    uint8_t *uv_plane;
    int camera_height = frame->height;
    int camera_width = frame->width;

    required_size = camera_height * camera_width * 3;
    std::vector<uint8_t> rgb_buf(required_size);

    y_size = camera_height * camera_width;
    y_plane = static_cast<uint8_t *>(frame->data.get());
    uv_plane = y_plane + y_size;

    for (int i = 0; i < camera_height; i++) {
      int y_offset = camera_width * i;
      int uv_offset = camera_width * (i >> 1);
      int rgb_offset = y_offset * 3;

      for (int j = 0; j < camera_width; j++) {
        int y = y_plane[y_offset++];
        int u = uv_plane[uv_offset];
        int v = uv_plane[uv_offset + 1];
        int r = y + (1.402 * (v - 128));
        int g = y - (0.34414 * (u - 128)) - (0.71414 * (v - 128));
        int b = y + (1.772 * (u - 128));

        rgb_buf[rgb_offset++] = (r > 255) ? 255 : (r < 0) ? 0 : r;
        rgb_buf[rgb_offset++] = (g > 255) ? 255 : (g < 0) ? 0 : g;
        rgb_buf[rgb_offset++] = (b > 255) ? 255 : (b < 0) ? 0 : b;

        if (0 != (j & 1)) {
          uv_offset += 2;
        }
      }
    }
#endif // RK3588
    // Publish the RGB image as a ROS Image message
    if (zero_copy) {
      auto loanedMsg = publisher_rgb_loan->borrow_loaned_message();
      // 判断消息是否可用，可能出现获取消息失败导致消息不可用的情况
      if (!loanedMsg.is_valid()) {
        // 获取消息失败，丢弃该消息
        RCLCPP_INFO(this->get_logger(), "Failed to get LoanMessage!");
        return;
      }
      // 引用方式获取实际的消息
      auto& msg = loanedMsg.get();
      auto rgb_msg = &msg;
      rgb_msg->header.stamp = custom_time;
      const char* id_str = "rgb";
      std::copy(id_str, id_str + strlen(id_str) + 1, rgb_msg->header.frame_id.begin());
      rgb_msg->height = frame->height;
      rgb_msg->width = frame->width;
      const char* encoding_str = "rgb8";
      std::copy(encoding_str, encoding_str + strlen(encoding_str) + 1, rgb_msg->encoding.begin());
      rgb_msg->is_bigendian = false;
  #ifdef RK3588
      rgb_msg->step = frame->width * get_bpp_from_format(RK_FORMAT_RGB_888) * 1;
  #else
      rgb_msg->step = frame->width * 3 * 1;
  #endif // RK3588
      // rgb_msg->data = rgb_buf;
      if (rgb_buf.size() <= rgb_msg->data.size()) {
          std::copy(rgb_buf.begin(), rgb_buf.end(), rgb_msg->data.begin());
      } else {
          // 处理错误：尺寸不匹配
          RCLCPP_ERROR(rclcpp::get_logger("metaS_node"), "RGB buffer size too large!");
      }

  #ifdef ROS_FOUND
      publisher_rgb.publish(*rgb_msg);
  #elif ROS2_FOUND
      publisher_rgb_loan->publish(std::move(*rgb_msg));
  #endif // ROS_ROS2_FOUND
    }
    else {
      auto rgb_msg = std::make_shared<sensor_msgs::msg::Image>();
      rgb_msg->header.stamp = custom_time;
      rgb_msg->header.frame_id = "rgb";
      rgb_msg->height = frame->height;
      rgb_msg->width = frame->width;
      rgb_msg->encoding = "rgb8";
      rgb_msg->is_bigendian = false;
  #ifdef RK3588
      rgb_msg->step = frame->width * get_bpp_from_format(RK_FORMAT_RGB_888) * 1;
  #else
      rgb_msg->step = frame->width * 3 * 1;
  #endif // RK3588
    rgb_msg->data = rgb_buf;

  #ifdef ROS_FOUND
      publisher_rgb.publish(*rgb_msg);
  #elif ROS2_FOUND
      publisher_rgb->publish(*rgb_msg);
  #endif // ROS_ROS2_FOUND
    }
  }

  void depth_handle_loan(const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &frame) {
    #ifdef ROS_FOUND
        auto cloud_msg = std::make_shared<sensor_msgs::PointCloud2>();
    #elif ROS2_FOUND
        auto loanedMsg = publisher_depth_loan->borrow_loaned_message();
        // 判断消息是否可用，可能出现获取消息失败导致消息不可用的情况
        if (!loanedMsg.is_valid()) {
          // 获取消息失败，丢弃该消息
          RCLCPP_INFO(this->get_logger(), "Failed to get LoanMessage!");
          return;
        }
        // 引用方式获取实际的消息
        auto& msg = loanedMsg.get();
        auto cloud_msg = &msg;
    #endif // ROS_ROS2_FOUND
        const char* id_str = "rslidar";
        std::copy(id_str, id_str + strlen(id_str) + 1, cloud_msg->header.frame_id.begin());
        cloud_msg->height = 1;
        cloud_msg->width = frame->size();
    
        // Define the structure of the point fields in the point cloud
        const char* field_names[] = {"x", "y", "z", "intensity", "ring", "timestamp"};
        for (int i = 0; i < 6; i++) {
          std::copy(field_names[i], field_names[i] + strlen(field_names[i]) + 1, cloud_msg->fields[i].name.begin());
        }
        cloud_msg->fields[0].offset = 0;
    #ifdef ROS_FOUND
        cloud_msg->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    #elif ROS2_FOUND
        cloud_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    #endif // ROS_ROS2_FOUND
        cloud_msg->fields[0].count = 1;
    
        cloud_msg->fields[1].offset = 4;
    #ifdef ROS_FOUND
        cloud_msg->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    #elif ROS2_FOUND
        cloud_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    #endif // ROS_ROS2_FOUND
        cloud_msg->fields[1].count = 1;
    
        cloud_msg->fields[2].offset = 8;
    #ifdef ROS_FOUND
        cloud_msg->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    #elif ROS2_FOUND
        cloud_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    #endif // ROS_ROS2_FOUND
        cloud_msg->fields[2].count = 1;
    
        cloud_msg->fields[3].offset = 16;
    #ifdef ROS_FOUND
        cloud_msg->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    #elif ROS2_FOUND
        cloud_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    #endif // ROS_ROS2_FOUND
        cloud_msg->fields[3].count = 1;
    
        cloud_msg->fields[4].offset = 20;
    #ifdef ROS_FOUND
        cloud_msg->fields[4].datatype = sensor_msgs::PointField::UINT16;
    #elif ROS2_FOUND
        cloud_msg->fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
    #endif // ROS_ROS2_FOUND
        cloud_msg->fields[4].count = 1;
    
        cloud_msg->fields[5].offset = 24;
    #ifdef ROS_FOUND
        cloud_msg->fields[5].datatype = sensor_msgs::PointField::FLOAT64;
    #elif ROS2_FOUND
        cloud_msg->fields[5].datatype = sensor_msgs::msg::PointField::FLOAT64;
    #endif // ROS_ROS2_FOUND
        cloud_msg->fields[5].count = 1;
    
        cloud_msg->is_bigendian = false;
        cloud_msg->point_step = 32;
        cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
    
        // cloud_msg->data.resize(cloud_msg->row_step);
    
        // copy memory
        memcpy(cloud_msg->data.data(), frame->points.data(), cloud_msg->row_step);
    
        // NOTE: msg header is points tail time
        double tail_stamp = frame->points[frame->size() - 1].timestamp;
    #ifdef ROS_FOUND
        uint32_t sec = static_cast<uint32_t>(tail_stamp);
        uint32_t nsec = static_cast<uint32_t>((tail_stamp - sec) * 1e9);
        auto custom_time = ros::Time(sec, nsec);
    #elif ROS2_FOUND
        uint32_t sec = static_cast<uint32_t>(tail_stamp);
        uint32_t nsec = static_cast<uint32_t>((tail_stamp - sec) * 1e9);
        auto custom_time = rclcpp::Time(sec, nsec);
    #endif // ROS_ROS2_FOUND
    
        cloud_msg->header.stamp = custom_time;
    
    #ifdef ROS_FOUND
        publisher_depth.publish(*cloud_msg);
    #elif ROS2_FOUND
        publisher_depth_loan->publish(std::move(*cloud_msg));
    #endif // ROS_ROS2_FOUND
      }

  void depth_handle(const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &frame) {
#ifdef ROS_FOUND
    auto cloud_msg = std::make_shared<sensor_msgs::PointCloud2>();
#elif ROS2_FOUND
    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
#endif // ROS_ROS2_FOUND

    cloud_msg->header.frame_id = "rslidar";
    cloud_msg->height = 1;
    cloud_msg->width = frame->size();

    // Define the structure of the point fields in the point cloud
    cloud_msg->fields.resize(6);
    cloud_msg->fields[0].name = "x";
    cloud_msg->fields[0].offset = 0;
#ifdef ROS_FOUND
    cloud_msg->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
#elif ROS2_FOUND
    cloud_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[0].count = 1;

    cloud_msg->fields[1].name = "y";
    cloud_msg->fields[1].offset = 4;
#ifdef ROS_FOUND
    cloud_msg->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
#elif ROS2_FOUND
    cloud_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[1].count = 1;

    cloud_msg->fields[2].name = "z";
    cloud_msg->fields[2].offset = 8;
#ifdef ROS_FOUND
    cloud_msg->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
#elif ROS2_FOUND
    cloud_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[2].count = 1;

    cloud_msg->fields[3].name = "intensity";
    cloud_msg->fields[3].offset = 16;
#ifdef ROS_FOUND
    cloud_msg->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
#elif ROS2_FOUND
    cloud_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[3].count = 1;

    cloud_msg->fields[4].name = "ring";
    cloud_msg->fields[4].offset = 20;
#ifdef ROS_FOUND
    cloud_msg->fields[4].datatype = sensor_msgs::PointField::UINT16;
#elif ROS2_FOUND
    cloud_msg->fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[4].count = 1;

    cloud_msg->fields[5].name = "timestamp";
    cloud_msg->fields[5].offset = 24;
#ifdef ROS_FOUND
    cloud_msg->fields[5].datatype = sensor_msgs::PointField::FLOAT64;
#elif ROS2_FOUND
    cloud_msg->fields[5].datatype = sensor_msgs::msg::PointField::FLOAT64;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[5].count = 1;

    cloud_msg->is_bigendian = false;
    cloud_msg->point_step = 32;
    cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;

    cloud_msg->data.resize(cloud_msg->row_step);

    // copy memory
    memcpy(cloud_msg->data.data(), frame->points.data(), cloud_msg->row_step);

    // NOTE: msg header is points tail time
    double tail_stamp = frame->points[frame->size() - 1].timestamp;
#ifdef ROS_FOUND
    uint32_t sec = static_cast<uint32_t>(tail_stamp);
    uint32_t nsec = static_cast<uint32_t>((tail_stamp - sec) * 1e9);
    auto custom_time = ros::Time(sec, nsec);
#elif ROS2_FOUND
    uint32_t sec = static_cast<uint32_t>(tail_stamp);
    uint32_t nsec = static_cast<uint32_t>((tail_stamp - sec) * 1e9);
    auto custom_time = rclcpp::Time(sec, nsec);
#endif // ROS_ROS2_FOUND

    cloud_msg->header.stamp = custom_time;

#ifdef ROS_FOUND
    publisher_depth.publish(*cloud_msg);
#elif ROS2_FOUND
    publisher_depth->publish(*cloud_msg);
#endif // ROS_ROS2_FOUND
  }

  void imu_handle(const std::shared_ptr<robosense::lidar::ImuData> &msgPtr) {
    uint64_t timestampNs = msgPtr->timestamp * 1000000000;
    uint32_t sec = timestampNs / 1000000000;
    uint32_t nsec = timestampNs % 1000000000;
#ifdef ROS_FOUND
    auto custom_time = ros::Time(sec, nsec);
    auto imu_msg = std::make_shared<sensor_msgs::Imu>();
#elif ROS2_FOUND
    auto custom_time = rclcpp::Time(sec, nsec);
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
#endif // ROS_ROS2_FOUND

    imu_msg->header.stamp = custom_time;
    imu_msg->header.frame_id = "rslidar";

    // Populate IMU message with acceleration and gyro data
    imu_msg->linear_acceleration.x = msgPtr->linear_acceleration_x;
    imu_msg->linear_acceleration.y = msgPtr->linear_acceleration_y;
    imu_msg->linear_acceleration.z = msgPtr->linear_acceleration_z;
    imu_msg->angular_velocity.x = msgPtr->angular_velocity_x;
    imu_msg->angular_velocity.y = msgPtr->angular_velocity_y;
    imu_msg->angular_velocity.z = msgPtr->angular_velocity_z;

#ifdef ROS_FOUND
    publisher_imu.publish(*imu_msg);
#elif ROS2_FOUND
    publisher_imu->publish(*imu_msg);
#endif // ROS_ROS2_FOUND
  }

  void logError(const std::string &message) {
#ifdef ROS_FOUND
    ROS_ERROR("%s", message.c_str());
#elif ROS2_FOUND
    RCLCPP_ERROR(this->get_logger(), message.c_str());
#endif // ROS_ROS2_FOUND
  }

  void logInfo(const std::string &message) {
#ifdef ROS_FOUND
    ROS_INFO("%s", message.c_str());
#elif ROS2_FOUND
    RCLCPP_INFO(this->get_logger(), message.c_str());
#endif // ROS_ROS2_FOUND
  }

  void shutdown() {
#ifdef ROS_FOUND
    ros::shutdown();
#elif ROS2_FOUND
    rclcpp::shutdown();
#endif // ROS_ROS2_FOUND
  }

#ifdef DEBUG_STATISTICS
  void topic_point_callback(const sensor_msgs::msg::PointCloud2 msg) const {
    // RCLCPP_INFO(this->get_logger(), "I heard PointCloud2 msg");
  }
  void topic_imu_callback(const sensor_msgs::msg::Imu msg) const {
    // RCLCPP_INFO(this->get_logger(), "I heard PointCloud2 msg");
  }
  void
  topic_frame_callback(const robosense_msgs::msg::RsCompressedImage msg) const {
    // RCLCPP_INFO(this->get_logger(), "I heard PointCloud2 msg");
  }
#endif // DEBUG_STATISTICS

// ROS/ROS2 publishers for RGB images, depth point clouds, and IMU data
#ifdef ROS_FOUND
  ros::Publisher publisher_rgb;
  ros::Publisher publisher_depth;
  ros::Publisher publisher_imu;
#ifdef RK3588
  ros::Publisher publisher_h265;
#else
  ros::Publisher publisher_jpeg;
#endif // RK3588

#elif ROS2_FOUND
    bool zero_copy;
    rclcpp::Publisher<robosense_msgs::msg::RsImage>::SharedPtr publisher_rgb_loan;
    rclcpp::Publisher<robosense_msgs::msg::RsPointCloud>::SharedPtr publisher_depth_loan;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_rgb;
#ifdef RK3588
  rclcpp::Publisher<robosense_msgs::msg::RsCompressedImage>::SharedPtr
      publisher_h265;
#else
  rclcpp::Publisher<robosense_msgs::msg::RsCompressedImage>::SharedPtr
      publisher_jpeg;
#endif // RK3588
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_depth;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu;
#endif // ROS_ROS2_FOUND

  // 设备管理
  std::shared_ptr<robosense::device::DeviceManager> device_manager_ptr;
  std::mutex current_device_uuid_mtx;
  std::string current_device_uuid;

  // 编码相关
  int imageWidth;
  int imageHeight;
#ifdef RK3588
  AVCodecContext *codecContext;
  AVFrame *av_frame;
  AVPacket *av_pkt;
#else
  robosense::jpeg::JpegCoder jpegEncoder;
  int nv12_image_size;
#endif // RK3588

#ifdef DEBUG_STATISTICS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<robosense_msgs::msg::RsCompressedImage>::SharedPtr
      frame_subscription_;
#endif // DEBUG_STATISTICS

#ifdef DEBUG_TO_FILE
  FILE *outfile;
  int out_count;
#endif // DEBUG_TO_FILE
};

/**
 * @brief Main function initializes the ROS2 node and spins it.
 * @param argc Argument count.
 * @param argv Argument values.
 * @return Exit status.
 */
int main(int argc, char **argv) {
#ifdef ROS_FOUND
  ros::init(argc, argv, "ms_node");
  MSPublisher ms_publisher;
  ros::spin();
#elif ROS2_FOUND
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MSPublisher>());
  rclcpp::shutdown();
#endif

  return 0;
}
