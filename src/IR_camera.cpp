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


// 前向宣告 IRCameraNode 類，以便在全局變數宣告中引用
class IRCameraNode;

// 宣告全域變數，指向 IRCameraNode 實例
IRCameraNode *g_node_instance = nullptr;

// 宣告靜態全局回調函數，以供 `IRCameraNode` 使用
int frameCallbackStatic(guide_usb_frame_data_t *pVideoData);

class IRCameraNode : public rclcpp::Node
{
public:
    IRCameraNode() : Node("ir_camera")
    {
        g_node_instance = this; // 初始化全局指針以指向當前實例

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/ir_camera/image", 10);

        guide_usb_setloglevel(LOG_INFO);

        // 初始化 USB 設備
        int ret = guide_usb_initial();
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Initialization failed: %d. Please give device permission: sudo chmod -R 777 /dev/bus/usb", ret);
            rclcpp::shutdown();
            return;
        }

        // 開啟指令控制模式
        ret = guide_usb_opencommandcontrol(serailCallback);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open command control: %d", ret);
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "guide_usb_getserialdata return: %d", ret);

        // 載入測量曲線
        guide_measure_loadcurve();

        // 設置測量參數
        mDebugParam = (guide_measure_debugparam_t *)malloc(sizeof(guide_measure_debugparam_t));
        if (!mDebugParam) {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for mDebugParam.");
            rclcpp::shutdown();
            return;
        }

        // 設置裝置資訊
        guide_usb_device_info_t *deviceInfo = (guide_usb_device_info_t *)malloc(sizeof(guide_usb_device_info_t));
        if (!deviceInfo) {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for deviceInfo.");
            free(mDebugParam);
            rclcpp::shutdown();
            return;
        }

        deviceInfo->width = 256;
        deviceInfo->height = 192;
        deviceInfo->video_mode = Y16_PARAM_YUV;

        // 開啟影像串流，使用靜態回調函數
        ret = guide_usb_openstream(deviceInfo, frameCallbackStatic, connectStatusCallback);
        startTime_ = tick();
        RCLCPP_INFO(this->get_logger(), "Open return: %d", ret);

        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open stream: %d", ret);
            free(mDebugParam);
            free(deviceInfo);
            rclcpp::shutdown();
            return;
        }

        // 釋放裝置資訊內存
        free(deviceInfo);

        // 使用計時器進行更新，避免阻塞主迴圈
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&IRCameraNode::timerCallback, this));
    }

    ~IRCameraNode()
    {
        // 關閉影像串流和釋放資源
        guide_usb_closestream();
        guide_measure_deloadcurve();
        guide_usb_closecommandcontrol();
        guide_usb_exit();

        if (mDebugParam) {
            free(mDebugParam);
            mDebugParam = nullptr;
        }
    }

    // 非靜態成員函數，用於處理影像框架
    int frameCallback(guide_usb_frame_data_t *pVideoData)
    {
        if (!pVideoData || !pVideoData->paramLine)
            return 0;

        int width = pVideoData->frame_width;
        int height = pVideoData->frame_height;

        // 設定測量參數
        mDebugParam->exkf = 100;
        mDebugParam->exb = 0;
        mDebugParam->emiss = 98;
        mDebugParam->transs = 0;
        mDebugParam->reflectTemp = 23.0f;
        mDebugParam->distance = 30.0f;
        mDebugParam->fEnvironmentIncrement = 2500;

        float *temperature_data = (float *)malloc(width * height * sizeof(float));
        if (!temperature_data) {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for temperature data.");
            return 0;
        }

        int result = guide_measure_convertgray2temper(width, height, temperature_data, pVideoData->frame_src_data, pVideoData->paramLine, mDebugParam, 0);
        if (result < 0) {
            RCLCPP_ERROR(this->get_logger(), "Temperature conversion failed.");
            free(temperature_data);
            return 0;
        }

        // 查找最大溫度和最大灰度
        int x_temp = 0, y_temp = 0, max_temp_idx = 0;
        int x_yuv = 0, y_yuv = 0, max_yuv_idx = 0;
        float max_temp = 0.0f, max_yuv = 0.0f;

        for (int i = 0; i < width * height; i++) {
            float temperature = temperature_data[i];
            if (temperature > max_temp) {
                max_temp = temperature;
                x_temp = i % width;
                y_temp = i / width;
                max_temp_idx = i;
            }

            if (pVideoData->frame_src_data[i] > max_yuv) {
                max_yuv = pVideoData->frame_src_data[i];
                x_yuv = i % width;
                y_yuv = i / width;
                max_yuv_idx = i;
            }
        }

        // 使用單點轉換來驗證最大溫度
        float precise_temp = guide_measure_convertsinglegray2temper(
            pVideoData->frame_src_data[max_temp_idx], pVideoData->paramLine, mDebugParam, 1);

        RCLCPP_INFO(this->get_logger(), "temp max: index %d, x: %d, y: %d, max temp: %.2f", max_temp_idx, x_temp, y_temp, max_temp);
        // RCLCPP_INFO(this->get_logger(), "yuv max: index %d, x: %d, y: %d, max yuv: %.2f", max_yuv_idx, x_yuv, y_yuv, max_yuv);
        // RCLCPP_INFO(this->get_logger(), "yuv max - temp max index diff: %d", max_yuv_idx - max_temp_idx);

        // 計算 FPS
        static int fps_count = 0;
        fps_count++;
        if ((tick() - startTime_) > 1) {
            startTime_ = tick();
            RCLCPP_INFO(this->get_logger(), "FPS: %d", fps_count);
            fps_count = 0;
        }

        // 釋放溫度數據記憶體
        free(temperature_data);
        return 1;
    }

    static int serailCallback(guide_usb_serial_data_t *pSerialData)
    {
        if (!pSerialData)
            return 0;

        for (int i = 0; i < pSerialData->serial_recv_data_length; i++)
        {
            printf("%x ", pSerialData->serial_recv_data[i]);
        }
        printf("\n");
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
        // 每次觸發時保持 10 毫秒的睡眠時間，模擬舊程式中的 while (1) 循環
        usleep(10);
    }

private:
    double tick(void)
    {
        struct timeval t;
        gettimeofday(&t, 0);
        return t.tv_sec + 1E-6 * t.tv_usec;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double startTime_;
    guide_measure_debugparam_t *mDebugParam;
};

// 靜態全局回調函數，訪問全局節點實例並呼叫成員函數
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
    auto node = std::make_shared<IRCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
