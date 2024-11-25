#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

extern "C" {
    #include <guideusb2livestream.h>
    #include <stdio.h>
    #include <sys/types.h>
    #include <unistd.h>
    #include <malloc.h>
    #include <string.h>
    #include "sys/time.h"
    #include "time.h"
}

// 前向声明 IRImageNode 类，以便在全局变量中引用
class IRImageNode;

// 声明全局变量，指向 IRImageNode 实例
IRImageNode *g_node_instance = nullptr;

// 声明静态全局回调函数，以供 `IRImageNode` 使用
int frameCallbackStatic(guide_usb_frame_data_t *pVideoData);

class IRImageNode : public rclcpp::Node
{
public:
    IRImageNode() : Node("ir_image_node")
    {
        g_node_instance = this; // 初始化全局指针以指向当前实例

        // 创建图像发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("ir_image", 10);

        guide_usb_setloglevel(LOG_INFO);

        // 初始化 USB 设备
        int ret = guide_usb_initial();
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Initialization failed: %d. Please give device permission: sudo chmod -R 777 /dev/bus/usb", ret);
            rclcpp::shutdown();
            return;
        }

        // 开启指令控制模式
        ret = guide_usb_opencommandcontrol(serialCallback);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open command control: %d", ret);
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "guide_usb_getserialdata return: %d", ret);

        // 设置设备信息
        guide_usb_device_info_t *deviceInfo = (guide_usb_device_info_t *)malloc(sizeof(guide_usb_device_info_t));
        if (!deviceInfo) {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for deviceInfo.");
            rclcpp::shutdown();
            return;
        }

        deviceInfo->width = 256;
        deviceInfo->height = 192;
        deviceInfo->video_mode = Y16_PARAM_YUV;

        // 打开视频流，使用静态回调函数
        ret = guide_usb_openstream(deviceInfo, frameCallbackStatic, connectStatusCallback);
        RCLCPP_INFO(this->get_logger(), "Open return: %d", ret);

        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open stream: %d", ret);
            free(deviceInfo);
            rclcpp::shutdown();
            return;
        }

        // 释放设备信息内存
        free(deviceInfo);

        // 创建一个定时器以保持节点运行
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&IRImageNode::timerCallback, this));
    }

    ~IRImageNode()
    {
        // 关闭视频流和释放资源
        guide_usb_closestream();
        guide_usb_closecommandcontrol();
        guide_usb_exit();
    }

    // 非静态成员函数，用于处理帧数据
    int frameCallback(guide_usb_frame_data_t *pVideoData)
    {
        if (!pVideoData) {
            return 0;
        }

        int width = pVideoData->frame_width;
        int height = pVideoData->frame_height;

        // 处理 Y16 数据
        if (pVideoData->frame_src_data) {
            // 将 Y16 数据转换为 cv::Mat
            cv::Mat img(height, width, CV_16UC1, pVideoData->frame_src_data);

            // 归一化并转换为 8 位，以便可视化
            cv::Mat img8;
            img.convertTo(img8, CV_8UC1, 0.03); // 根据需要调整缩放因子

            // 可选地，应用伪彩色
            cv::Mat color_img;
            cv::applyColorMap(img8, color_img, cv::COLORMAP_INFERNO); // 根据需要选择伪彩色

            // 将图像转换为 ROS Image 消息并发布
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_img).toImageMsg();
            msg->header.stamp = this->get_clock()->now();
            publisher_->publish(*msg);
        }
        // 处理 RGB 数据
        else if (pVideoData->frame_rgb_data) {
            cv::Mat img(height, width, CV_8UC3, pVideoData->frame_rgb_data);

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
            msg->header.stamp = this->get_clock()->now();
            publisher_->publish(*msg);
        }
        // 处理 YUV 数据
        else if (pVideoData->frame_yuv_data) {
            cv::Mat yuv_img(height + height / 2, width, CV_8UC1, pVideoData->frame_yuv_data);
            cv::Mat bgr_img;
            cv::cvtColor(yuv_img, bgr_img, cv::COLOR_YUV2BGR_NV12);

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_img).toImageMsg();
            msg->header.stamp = this->get_clock()->now();
            publisher_->publish(*msg);
        }
        else {
            RCLCPP_WARN(this->get_logger(), "No valid image data received.");
        }

        return 1;
    }

    static int serialCallback(guide_usb_serial_data_t *pSerialData)
    {
        // 在此简化的节点中，我们可以忽略串口数据
        return 1;
    }

    static int connectStatusCallback(guide_usb_device_status_e deviceStatus)
    {
        if (deviceStatus == DEVICE_CONNECT_OK)
        {
            printf("Stream start OK \n");
        }
        else
        {
            printf("Stream end \n");
        }
        return 1;
    }

    void timerCallback()
    {
        // 保持节点活跃
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// 静态全局回调函数，访问全局节点实例并调用成员函数
int frameCallbackStatic(guide_usb_frame_data_t *pVideoData)
{
    if (g_node_instance) {
        return g_node_instance->frameCallback(pVideoData);
    }
    return 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IRImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
