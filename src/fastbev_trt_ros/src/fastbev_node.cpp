#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime.h>
#include <fstream>
#include <vector>
#include <iostream>

using namespace std::placeholders;

class MyLogger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[TRT] " << msg << std::endl;
        }
    }
} gLogger;

class MyFastBEVNode : public rclcpp::Node {
public:
    MyFastBEVNode() : Node("my_fastbev_node") {
        // 1. 初始化 TensorRT
        std::string engine_path = "/root/autodl-tmp/CUDA-FastBEV/model/resnet18int8head/fastbev_pre.engine";
        initEngine(engine_path);
        allocateBuffers();
        cudaStreamCreate(&stream_);

        // 2. 初始化发布者
        pub_boxes_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/fastbev/bboxes", 10);

        // 3. 初始化订阅者 (匹配你的 ros2 topic list 结果)
        auto qos = rmw_qos_profile_sensor_data;
        sub_front_.subscribe(this, "/camera/front/image_raw", qos);
        sub_front_right_.subscribe(this, "/camera/front_right/image_raw", qos);
        sub_front_left_.subscribe(this, "/camera/front_left/image_raw", qos);
        sub_back_.subscribe(this, "/camera/back/image_raw", qos);
        sub_back_left_.subscribe(this, "/camera/back_left/image_raw", qos);
        sub_back_right_.subscribe(this, "/camera/back_right/image_raw", qos);

        // 4. 设置同步策略 (ApproximateTime)
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(20), sub_front_, sub_front_right_, sub_front_left_, sub_back_, sub_back_left_, sub_back_right_
        );
        sync_->registerCallback(std::bind(&MyFastBEVNode::callback, this, _1, _2, _3, _4, _5, _6));

        RCLCPP_INFO(this->get_logger(), "✅ FastBEV 节点已启动，监听 6 路相机话题并发布到 /fastbev/bboxes");
    }

    ~MyFastBEVNode() {
        cudaStreamDestroy(stream_);
        cudaFree(device_input_);
        cudaFree(device_output_);
        delete context_;
        delete engine_;
        delete runtime_;
    }

private:
    void initEngine(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开引擎文件: %s", path.c_str());
            return;
        }
        file.seekg(0, file.end);
        size_t size = file.tellg();
        file.seekg(0, file.beg);
        std::vector<char> data(size);
        file.read(data.data(), size);
        file.close();

        runtime_ = nvinfer1::createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(data.data(), size);
        context_ = engine_->createExecutionContext();
    }

    void allocateBuffers() {
        // 输入: 6 Batch, 3 Channel, 256 Height, 704 Width
        input_size_ = 6 * 3 * 256 * 704 * sizeof(float);
        // 输出: 假设 100 个 Box, 每个 9 维 (x, y, z, w, l, h, yaw, score, class)
        output_size_ = 100 * 9 * sizeof(float);

        cudaMalloc(&device_input_, input_size_);
        cudaMalloc(&device_output_, output_size_);
    }

    void callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_f,
        const sensor_msgs::msg::Image::ConstSharedPtr& img_fr,
        const sensor_msgs::msg::Image::ConstSharedPtr& img_fl,
        const sensor_msgs::msg::Image::ConstSharedPtr& img_b,
        const sensor_msgs::msg::Image::ConstSharedPtr& img_bl,
        const sensor_msgs::msg::Image::ConstSharedPtr& img_br) 
    {
        sensor_msgs::msg::Image::ConstSharedPtr msgs[6] = {img_f, img_fr, img_fl, img_b, img_bl, img_br};
        std::vector<float> host_input(6 * 3 * 256 * 704);
        float* ptr = host_input.data();

        // 1. 预处理：OpenCV HWC -> TensorRT NCHW
        for (int i = 0; i < 6; ++i) {
            cv::Mat frame = cv_bridge::toCvShare(msgs[i], "bgr8")->image;
            cv::Mat resized;
            cv::resize(frame, resized, cv::Size(704, 256));

            int channel_size = 256 * 704;
            for (int c = 0; c < 3; ++c) {
                for (int h = 0; h < 256; ++h) {
                    for (int w = 0; w < 704; ++w) {
                        int out_idx = i * (3 * channel_size) + c * channel_size + h * 704 + w;
                        // 简单的归一化: (x/255.0 - 0.5) / 0.5
                        float pixel = static_cast<float>(resized.at<cv::Vec3b>(h, w)[c]);
                        ptr[out_idx] = (pixel / 255.0f - 0.5f) / 0.5f;
                    }
                }
            }
        }

        // 2. 推理
        cudaMemcpyAsync(device_input_, host_input.data(), input_size_, cudaMemcpyHostToDevice, stream_);
        void* bindings[] = {device_input_, device_output_};
        context_->enqueueV2(bindings, stream_, nullptr);

        std::vector<float> host_output(output_size_ / sizeof(float));
        cudaMemcpyAsync(host_output.data(), device_output_, output_size_, cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        // 3. 发布结果
        std_msgs::msg::Float32MultiArray out_msg;
        out_msg.data = host_output;
        pub_boxes_->publish(out_msg);

        RCLCPP_INFO(this->get_logger(), "🚀 推理完成并发布，检测到数据点数: %zu", host_output.size());
    }

    // ROS 2 成员
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_boxes_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_front_, sub_front_right_, sub_front_left_, sub_back_, sub_back_left_, sub_back_right_;
    
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image
    > SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // TensorRT 成员
    nvinfer1::IRuntime* runtime_{nullptr};
    nvinfer1::ICudaEngine* engine_{nullptr};
    nvinfer1::IExecutionContext* context_{nullptr};
    void* device_input_{nullptr};
    void* device_output_{nullptr};
    size_t input_size_;
    size_t output_size_;
    cudaStream_t stream_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyFastBEVNode>());
    rclcpp::shutdown();
    return 0;
}