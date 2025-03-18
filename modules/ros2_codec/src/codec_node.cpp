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
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <robosense_msgs/msg/rs_compressed_image.hpp>
#endif

class CodecPublisher
#ifdef ROS2_FOUND
    : public rclcpp::Node
#endif

{
public:
    /**
     * @brief Constructor initializes the node, sets up publishers, and starts the device streams.
     */
    CodecPublisher()
    #ifdef ROS2_FOUND
        : Node("codec_node")
    #endif
    {
        // Initialize publishers for image
        #ifdef ROS_FOUND
            ros::NodeHandle nh;
            publisher_image = nh.advertise<sensor_msgs::Image>("/camera/image_color/image", 10);
        #elif ROS2_FOUND
            publisher_image = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_color/image", 10);
        #endif
            imageWidth = 1920;
            imageHeight = 1080; 
#ifdef RK3588
    	    avdevice_register_all();
	        av_pkt = av_packet_alloc();
            if (!av_pkt) {
            	RCLCPP_ERROR(this->get_logger(), "can not allocl av packet\n");
            	rclcpp::shutdown();
                return;
            }

            const AVCodec *codec = avcodec_find_decoder_by_name("hevc_rkmpp");
            if (!codec) {
            	RCLCPP_ERROR(this->get_logger(), "can not find h.265 codec\n");
            	rclcpp::shutdown();
                return;
            }

	        codecContext = avcodec_alloc_context3(codec);
	        if (!codecContext) {
            	RCLCPP_ERROR(this->get_logger(), "can not alloc avcodec context3\n");
            	rclcpp::shutdown();
		        return;
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
            	rclcpp::shutdown();
                return;
            }

	        av_frame = av_frame_alloc();
	        if (!av_frame) {
            	RCLCPP_ERROR(this->get_logger(), "can not alloc av frame\n");
            	rclcpp::shutdown();
		    return;
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
            	RCLCPP_ERROR(this->get_logger(), "jpeg decoder(nv12) initial failed: ret = %d\n", ret);
            	rclcpp::shutdown();
                return;
            }

            nv12_image_size = robosense::color::ColorCodec::NV12ImageSize(imageWidth, imageHeight);
#endif

	        auto callback =
		        [this](robosense_msgs::msg::RsCompressedImage msg) -> void
		        {
			        decode_handle(msg);
		        };
	        sub_ = create_subscription<robosense_msgs::msg::RsCompressedImage>("/rs_camera/compressed", 10, callback);

            RCLCPP_INFO(this->get_logger(), "Start...");
    }

    /**
     * @brief Destructor cleans up the device object.
     */
    ~CodecPublisher() = default;

private:
#ifdef RK3588
    int convert_drm_prime_to_rgb(AVFrame *drm_frame, AVFrame *rgb_frame) {
	    AVBufferRef *hw_device_ctx = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_DRM);
	    if (!hw_device_ctx) {
		    fprintf(stderr, "Failed to create HW device context\n");
		    return -1;
	    }

	    AVFrame *mapped_frame = av_frame_alloc();
	    if (!mapped_frame) {
		    fprintf(stderr, "Failed to allocate frame\n");
		    return -1;
	    }

	    if (av_hwframe_transfer_data(mapped_frame, drm_frame, 0) < 0) {
		    fprintf(stderr, "Failed to transfer data from DRM Prime frame\n");
		    return -1;
	    }

	    int a_f = (enum AVPixelFormat)mapped_frame->format;

	    printf("format=%d, width=%d, height=%d size=%d\n", (int)(a_f), mapped_frame->width, mapped_frame->height, mapped_frame->linesize[0]);
	    struct SwsContext *sws_ctx = sws_getContext(
			    mapped_frame->width, mapped_frame->height, (enum AVPixelFormat)mapped_frame->format,
			    rgb_frame->width, rgb_frame->height, AV_PIX_FMT_RGB24,
			    SWS_BILINEAR, NULL, NULL, NULL);

	    if (!sws_ctx) {
		    fprintf(stderr, "Failed to create SwsContext\n");
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

    void decode_handle(robosense_msgs::msg::RsCompressedImage msg)
    {
        int ret;

        auto rgb_msg = std::make_shared<sensor_msgs::msg::Image>();
#ifdef RK3588
        AVPacket packet;
        av_init_packet(&packet);
        packet.data = NULL;
        packet.size = 0;
        ret = av_packet_from_data(&packet, msg.data.data(), msg.data.size());
        if (ret < 0) {
            fprintf(stderr, "Error parsing msg data\n");
            return;
        }

        if (packet.size > 0) {
            fprintf(stderr, "sending a packet for decoding\n"); 
            ret = avcodec_send_packet(codecContext, &packet);
            if (ret < 0) {
                char err_msg[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, err_msg, sizeof(err_msg));
                fprintf(stderr, "Error sending a av_pkt for decoding : %s\n", err_msg); 
                return;
            }

            AVFrame *rgb_frame = av_frame_alloc();
            while (ret >= 0) {
                ret = avcodec_receive_frame(codecContext, av_frame);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    break;
                else if (ret < 0) {
                    fprintf(stderr, "Error during decoding\n");
                    break;
                }

                printf("saving frame %3"PRId64" format=%d\n", codecContext->frame_number, av_frame->format);

                AVFrame *rgb_frame = av_frame_alloc();
                rgb_frame->format = AV_PIX_FMT_RGB24;
                rgb_frame->width = av_frame->width;
                rgb_frame->height = av_frame->height;
                av_frame_get_buffer(rgb_frame, 0);

                if (convert_drm_prime_to_rgb(av_frame, rgb_frame) < 0) {
                    fprintf(stderr, "Failed to convert DRM Prime frame to RGB\n");
                    break;
                }


                std::vector<uint8_t> rgb_buf;
                rgb_buf.resize(rgb_frame->width * rgb_frame->height * 3); 

                for (int y = 0; y < rgb_frame->height; y++) {
                    memcpy(rgb_buf.data() + y * rgb_frame->width * 3, rgb_frame->data[0] + y * rgb_frame->linesize[0], rgb_frame->width * 3);
                }


                // Publish the RGB image as a ROS Image message
                rgb_msg->header.stamp = msg.header.stamp;
                rgb_msg->header.frame_id = "rgb";
                rgb_msg->height = av_frame->height;
                rgb_msg->width = av_frame->width;
                rgb_msg->encoding = "rgb8";
                rgb_msg->step = av_frame->width * 3 * 1;
                rgb_msg->is_bigendian = false;

                rgb_msg->data = rgb_buf;
                publisher_image->publish(*rgb_msg);

            }
            av_frame_free(&rgb_frame);
        }
#else
    //for X86
    std::vector<unsigned char> jpeg_decode_buffer(nv12_image_size, '\0'); 
    size_t jpeg_decode_buffer_len = nv12_image_size;
    ret = jpegDecoder.decode(msg.data.data(), msg.data.size(),
                jpeg_decode_buffer.data(),
                jpeg_decode_buffer_len);

    // NV12 to RGB24  for cpu
    unsigned int required_size;
    unsigned int y_size;
    uint8_t *y_plane;
    uint8_t *uv_plane;
    int camera_height = imageHeight;
    int camera_width = imageWidth;

    required_size = camera_height * camera_width * 3;
    std::vector<uint8_t> rgb_buf(required_size);

    y_size = camera_height * camera_width;
    y_plane = static_cast<uint8_t *>(jpeg_decode_buffer.data());
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

    // Publish the RGB image as a ROS Image message
    rgb_msg->header.stamp = msg.header.stamp;
    rgb_msg->header.frame_id = "rgb";
    rgb_msg->height = imageHeight;
    rgb_msg->width = imageWidth;
    rgb_msg->encoding = "rgb8";
    rgb_msg->step = imageWidth * 3 * 1;
    rgb_msg->is_bigendian = false;

    rgb_msg->data = rgb_buf;
    publisher_image->publish(*rgb_msg);
#endif
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image;
    rclcpp::Subscription<robosense_msgs::msg::RsCompressedImage>::SharedPtr sub_;

    int imageWidth;
    int imageHeight;
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

/**
 * @brief Main function initializes the ROS2 node and spins it.
 * @param argc Argument count.
 * @param argv Argument values.
 * @return Exit status.
 */
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CodecPublisher>());
	rclcpp::shutdown();
	return 0;
}
