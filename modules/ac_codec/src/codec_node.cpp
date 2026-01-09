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

#include <iostream>
#include <chrono>
#include <memory>
#include <cstring>
#include <vector>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifdef RK3588
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavdevice/avdevice.h>
#include <libavutil/opt.h>
#include <libavutil/error.h>
}
#else
#include "colorcodec.h"
#include "jpegcoder.h"
#endif

#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <robosense_msgs/msg/rs_image.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <robosense_msgs/msg/rs_compressed_image.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#endif

namespace robosense
{
namespace ac
{

class CodecPublisher
#ifdef ROS2_FOUND
    : public rclcpp_lifecycle::LifecycleNode
#endif

{
public:
    /**
     * @brief Constructor initializes the node, sets up publishers, and starts the device streams.
     */
    CodecPublisher(const rclcpp::NodeOptions &options)
    #ifdef ROS2_FOUND
        : rclcpp_lifecycle::LifecycleNode("codec_node", options)
    #endif
    {
        this->declare_parameter<int>("image_width", 1920);
        this->declare_parameter<int>("image_height", 1080);
        imageWidth = this->get_parameter("image_width").as_int();
        imageHeight = this->get_parameter("image_height").as_int();
    }

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
        // QoS standardization
        auto qos = rclcpp::SensorDataQoS();
        // Initialize publishers for image
        #ifdef ROS_FOUND
            ros::NodeHandle nh;
            publisher_image = nh.advertise<sensor_msgs::Image>("/camera/image_color/image", 10);
        #elif ROS2_FOUND
            publisher_image = this->create_publisher<robosense_msgs::msg::RsImage>("/camera/image_color/image", qos);
        #endif

#ifdef RK3588
    	    avdevice_register_all();
	        av_pkt = av_packet_alloc();
            if (!av_pkt) {
            	RCLCPP_ERROR(this->get_logger(), "can not allocl av packet\n");
                return CallbackReturn::FAILURE;
            }

            const AVCodec *codec = avcodec_find_decoder_by_name("hevc_rkmpp");
            if (!codec) {
            	RCLCPP_ERROR(this->get_logger(), "can not find h.265 codec\n");
                return CallbackReturn::FAILURE;
            }

	        codecContext = avcodec_alloc_context3(codec);
	        if (!codecContext) {
            	RCLCPP_ERROR(this->get_logger(), "can not alloc avcodec context3\n");
		        return CallbackReturn::FAILURE;
	        }

	        //codecContext->bit_rate = 4000000;
            codecContext->width = imageWidth;
            codecContext->coded_width = imageWidth;
            codecContext->height = imageHeight; 
            codecContext->coded_height = imageHeight;
            //codecContext->pix_fmt = AV_PIX_FMT_RGB24;
            codecContext->time_base = (AVRational){1, 30};
            codecContext->framerate = (AVRational){30, 1};
            codecContext->gop_size = 30;
            codecContext->max_b_frames = 1;

            if (avcodec_open2(codecContext, codec, NULL) < 0) {
            	RCLCPP_ERROR(this->get_logger(), "can not open avcodec\n");
                return CallbackReturn::FAILURE;
            }

	        av_frame = av_frame_alloc();
	        if (!av_frame) {
            	RCLCPP_ERROR(this->get_logger(), "can not alloc av frame\n");
		        return CallbackReturn::FAILURE;
	        }
#else
            robosense::jpeg::JpegCodesConfig config;
            config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_DECODE;
            config.imageFrameFormat = robosense::common::FRAME_FORMAT_NV12;
            config.imageWidth = imageWidth;
            config.imageHeight = imageHeight;
            config.gpuDeviceId = 0;

            int ret = jpegDecoder.init(config);
            if (ret != 0) {
            	RCLCPP_ERROR(this->get_logger(), "jpeg decoder(nv12) initial failed: ret = %d", ret);
                return CallbackReturn::FAILURE;
            }

            nv12_image_size = robosense::color::ColorCodec::NV12ImageSize(imageWidth, imageHeight);
#endif

            rgb_buf.resize(imageWidth * imageHeight * 3);

	        auto callback =
		        [this](const std::shared_ptr<const robosense_msgs::msg::RsCompressedImage> &msg) -> void
		        {
			        decode_handle(*msg);
		        };
	        sub_ = create_subscription<robosense_msgs::msg::RsCompressedImage>("/rs_camera/compressed", 10, callback);

            RCLCPP_INFO(this->get_logger(), "Configured...");
            return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
        publisher_image->on_activate();
        RCLCPP_INFO(this->get_logger(), "Activated...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
        publisher_image->on_deactivate();
        RCLCPP_INFO(this->get_logger(), "Deactivated...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) {
        sub_.reset();
        publisher_image.reset();
        rgb_buf.clear();
        rgb_buf.shrink_to_fit();
#ifdef RK3588
        if (codecContext) {
            avcodec_close(codecContext);
            avcodec_free_context(&codecContext);
            codecContext = nullptr;
        }
        if (av_frame) {
            av_frame_free(&av_frame);
            av_frame = nullptr;
        }
        if (av_pkt) {
            av_packet_free(&av_pkt);
            av_pkt = nullptr;
        }
#endif
        RCLCPP_INFO(this->get_logger(), "Cleaned up...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) {
        return on_cleanup(state);
    }

    /**
     * @brief Destructor cleans up the device object.
     */
    ~CodecPublisher() {
        on_cleanup(rclcpp_lifecycle::State());
    }

private:
#ifdef RK3588
    int convert_drm_prime_to_rgb(AVFrame *drm_frame, AVFrame *rgb_frame) {
	    AVBufferRef *hw_device_ctx = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_DRM);
	    if (!hw_device_ctx) {
		    RCLCPP_ERROR(this->get_logger(), "Failed to create HW device context");
		    return -1;
	    }

	    AVFrame *mapped_frame = av_frame_alloc();
	    if (!mapped_frame) {
		    RCLCPP_ERROR(this->get_logger(), "Failed to allocate frame");
		    return -1;
	    }

	    if (av_hwframe_transfer_data(mapped_frame, drm_frame, 0) < 0) {
		    RCLCPP_ERROR(this->get_logger(), "Failed to transfer data from DRM Prime frame");
		    return -1;
	    }

	    int a_f = (enum AVPixelFormat)mapped_frame->format;

	    RCLCPP_INFO(this->get_logger(), "format=%d, width=%d, height=%d size=%d", (int)(a_f), mapped_frame->width, mapped_frame->height, mapped_frame->linesize[0]);
	    struct SwsContext *sws_ctx = sws_getContext(
			    mapped_frame->width, mapped_frame->height, (enum AVPixelFormat)mapped_frame->format,
			    rgb_frame->width, rgb_frame->height, AV_PIX_FMT_RGB24,
			    SWS_BILINEAR, NULL, NULL, NULL);

	    if (!sws_ctx) {
		    RCLCPP_ERROR(this->get_logger(), "Failed to create SwsContext");
		    return -1;
	    }

	    sws_scale(sws_ctx, (const uint8_t *const *)mapped_frame->data,
			    mapped_frame->linesize, 0, mapped_frame->height,
			    rgb_frame->data, rgb_frame->linesize);

	    sws_freeContext(sws_ctx);
	    av_frame_free(&mapped_frame);
	    av_buffer_unref(&hw_device_ctx);

	    return 0;
    }
#endif

    void decode_handle(const robosense_msgs::msg::RsCompressedImage &msg)
    {
        int ret;

        auto rgb_msg = std::make_shared<robosense_msgs::msg::RsImage>();
#ifdef RK3588
        AVPacket packet;
        av_init_packet(&packet);
        packet.data = NULL;
        packet.size = 0;
        ret = av_packet_from_data(&packet, const_cast<uint8_t*>(msg.data.data()), msg.data.size());
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing msg data");
            return;
        }

        if (packet.size > 0) {
            RCLCPP_DEBUG(this->get_logger(), "sending a packet for decoding"); 
            ret = avcodec_send_packet(codecContext, &packet);
            if (ret < 0) {
                char err_msg[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, err_msg, sizeof(err_msg));
                RCLCPP_ERROR(this->get_logger(), "Error sending a av_pkt for decoding : %s", err_msg); 
                return;
            }

            while (ret >= 0) {
                ret = avcodec_receive_frame(codecContext, av_frame);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    break;
                else if (ret < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Error during decoding");
                    break;
                }

                RCLCPP_DEBUG(this->get_logger(), "saving frame %3" PRId64 " format=%d", codecContext->frame_num, av_frame->format);

                AVFrame *rgb_frame = av_frame_alloc();
                if (!rgb_frame) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to allocate RGB frame");
                    break;
                }
                rgb_frame->format = AV_PIX_FMT_RGB24;
                rgb_frame->width = codecContext->width;
                rgb_frame->height = codecContext->height;
                if (av_frame_get_buffer(rgb_frame, 0) < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get RGB frame buffer");
                    av_frame_free(&rgb_frame);
                    break;
                }

                if (convert_drm_prime_to_rgb(av_frame, rgb_frame) < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to convert DRM Prime frame to RGB");
                    av_frame_free(&rgb_frame);
                    break;
                }


                if (rgb_buf.size() != rgb_frame->width * rgb_frame->height * 3) {
                    rgb_buf.resize(rgb_frame->width * rgb_frame->height * 3);
                }

                for (int y = 0; y < rgb_frame->height; y++) {
                    memcpy(rgb_buf.data() + y * rgb_frame->width * 3, rgb_frame->data[0] + y * rgb_frame->linesize[0], rgb_frame->width * 3);
                }


                // Publish the RGB image as a ROS Image message
                rgb_msg->header.stamp.sec = msg.header.stamp.sec;
                rgb_msg->header.stamp.nanosec = msg.header.stamp.nanosec;
                const char *id_str = "rgb";
                auto id_len = strnlen(id_str, 16);
                std::copy(id_str, id_str + id_len, rgb_msg->header.frame_id.begin());
                if (id_len < 16) {
                    rgb_msg->header.frame_id[id_len] = '\0';
                }
                rgb_msg->height = rgb_frame->height;
                rgb_msg->width = rgb_frame->width;
                const char *enc_str = "rgb8";
                auto enc_len = strnlen(enc_str, 8);
                std::copy(enc_str, enc_str + enc_len, rgb_msg->encoding.begin());
                if (enc_len < 8) {
                    rgb_msg->encoding[enc_len] = '\0';
                }
                rgb_msg->step = rgb_frame->width * 3 * 1;
                rgb_msg->is_bigendian = false;

                std::copy(rgb_buf.begin(), rgb_buf.end(), rgb_msg->data.begin());
                publisher_image->publish(*rgb_msg);

                av_frame_free(&rgb_frame);
            }
        }
#else
    //for X86
    std::vector<unsigned char> jpeg_decode_buffer(nv12_image_size, '\0'); 
    size_t jpeg_decode_buffer_len = nv12_image_size;
    ret = jpegDecoder.decode(msg.data.data(), msg.data.size(),
                jpeg_decode_buffer.data(),
                jpeg_decode_buffer_len);

    // NV12 to RGB24  for cpu
    int camera_height = imageHeight;
    int camera_width = imageWidth;

    rgb_buf.resize(camera_height * camera_width * 3);

    unsigned int y_size = camera_height * camera_width;
    uint8_t *y_plane = static_cast<uint8_t *>(jpeg_decode_buffer.data());
    uint8_t *uv_plane = y_plane + y_size;

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

    // Publish the RGB image as a ROS Image message
    rgb_msg->header.stamp.sec = msg.header.stamp.sec;
    rgb_msg->header.stamp.nanosec = msg.header.stamp.nanosec;
    const char *id_str = "rgb";
    auto id_len = strnlen(id_str, 16);
    std::copy(id_str, id_str + id_len, rgb_msg->header.frame_id.begin());
    if (id_len < 16) {
        rgb_msg->header.frame_id[id_len] = '\0';
    }
    rgb_msg->height = imageHeight;
    rgb_msg->width = imageWidth;
    const char *enc_str = "rgb8";
    auto enc_len = strnlen(enc_str, 8);
    std::copy(enc_str, enc_str + enc_len, rgb_msg->encoding.begin());
    if (enc_len < 8) {
        rgb_msg->encoding[enc_len] = '\0';
    }
    rgb_msg->step = imageWidth * 3 * 1;
    rgb_msg->is_bigendian = false;

    std::copy(rgb_buf.begin(), rgb_buf.end(), rgb_msg->data.begin());
    publisher_image->publish(*rgb_msg);
#endif
    }

    rclcpp_lifecycle::LifecyclePublisher<robosense_msgs::msg::RsImage>::SharedPtr publisher_image;
    rclcpp::Subscription<robosense_msgs::msg::RsCompressedImage>::SharedPtr sub_;

    int imageWidth;
    int imageHeight;
    std::vector<uint8_t> rgb_buf;
#ifdef RK3588
    AVCodecParserContext *parser;
    AVCodecContext* codecContext;
    AVFrame* av_frame;
    AVPacket *av_pkt;
#else
    robosense::jpeg::JpegCoder jpegDecoder;
    int nv12_image_size;
#endif

};

} // namespace ac
} // namespace robosense

RCLCPP_COMPONENTS_REGISTER_NODE(robosense::ac::CodecPublisher)
