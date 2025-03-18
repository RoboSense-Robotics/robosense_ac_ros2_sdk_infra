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
#ifndef SENSOR_DATA_TYPES_H
#define SENSOR_DATA_TYPES_H
#include <memory>
#include <functional>
#include <sys/time.h>

namespace robosense {
namespace common {
    typedef enum ImageFrameFormat
    {
        FRAME_FORMAT_ANY = 0,           /**< Any supported format */
        FRAME_FORMAT_H265,              /**< Compressed: h265 compressed format */
        FRAME_FORMAT_RGB24,             /**< 24-bit RGB format */
        FRAME_FORMAT_NV12,              /**< YUV420: NV12 format */
        FRAME_FORMAT_YUYV422,           /**< YUV422: Y U Y V format */
        FRAME_FORMAT_YUV420P,           /**< YUV420: YYYY UU VV format */ 
    } ImageFrameFormat_t;

    /**
     * @struct xyz
     * @brief Stores IMU (Inertial Measurement Unit) data for x, y, and z axes.
     */
    typedef struct Xyz
    {
        float x;                           /**< IMU data for x-axis */
        float y;                           /**< IMU data for y-axis */
        float z;                           /**< IMU data for z-axis */
    } Xyz_t;

    /**
     * @struct motion_frame
     * @brief Contains motion data from the IMU, including acceleration, gyroscope, and temperature data.
     */
    typedef struct MotionFrame
    {
        Xyz_t accel;                         /**< Accelerometer data (x, y, z) */
        Xyz_t gyro;                          /**< Gyroscope data (x, y, z) */
        float temperature;                   /**< Temperature reading from the IMU */
        struct timeval capture_time;         /**< Capture time of the motion data */
    } MotionFrame_t;

    /**
     * @struct cloud_point
     * @brief Represents a 3D point in a depth frame, including intensity information.
     */
    typedef struct CloudPointXYZIRT
    {
        float x;                           /**< X-coordinate in 3D space */
        float y;                           /**< Y-coordinate in 3D space */
        float z;                           /**< Z-coordinate in 3D space */
        uint8_t intensity;                 /**< Intensity value at this point */
        uint16_t ring;                     /**< Intensity value at this point */
        double timestamp;                  /**< Time value at this point, the unit is seconds*/
    } CloudPointXYZIRT_t;

    /**
     * @struct depth_frame
     * @brief Contains depth data, including an array of points and a timestamp.
     */
    typedef struct DepthFrame
    {
        std::shared_ptr<CloudPointXYZIRT_t> points;   /**< Array of points in the depth data */
        uint16_t point_nums;                          /**< Number of points in the depth data */
        struct timeval capture_time;                  /**< Capture time of the depth data */
    } DepthFrame_t;

    /**
     * @struct image_frame
     * @brief Represents a frame captured from a stream, including metadata and image data.
     */
    typedef struct ImageFrame
    {
        std::shared_ptr<uint8_t> data;          /**< Pointer to image data */
        size_t data_bytes;                      /**< Size of the image data buffer in bytes */
        uint32_t width;                         /**< Width of the image in pixels */
        uint32_t height;                        /**< Height of the image in pixels */
        ImageFrameFormat_t frame_format;        /**< Format of the pixel data */
        size_t step;                            /**< Bytes per horizontal line (not defined for compressed formats) */
        uint32_t sequence;                      /**< Frame sequence number */
        struct timeval capture_time;            /**< Capture time of the image data */
    } ImageFrame_t;

    typedef enum DeviceEventType 
    {
        DEVICE_EVENT_DETACH = 0, 
        DEVICE_EVENT_ATTACH,  

        DEVICE_EVENT_UNKNOWN = 255, 
    } DeviceEventType_t;  

    typedef struct DeviceEvent 
    {   
        DeviceEventType_t event_type; 
        uint32_t          uuid_size; 
        char              uuid[128]; 
    } DeviceEvent_t; 

    typedef enum DeviceOperatorType 
    {
        DEVICE_OPERATOR_CLOSE = 0, 
        DEVICE_OPERATOR_OPEN, 

        DEVICE_OPERATOR_UNKNOWN = 255, 
    } DeviceOperatorType_t; 

    typedef struct DeviceOperator
    {
        DeviceOperatorType_t operation_type; 
        uint32_t             uuid_size; 
        char                 uuid[128]; 
    } DeviceOperator_t; 
}
}

#endif // SENSOR_DATA_TYPES_H