#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include <deque>
#include <onnxruntime_cxx_api.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

#define SEQ_LEN 10
#define WIDTH 160
#define HEIGHT 120

class OnnxInferenceNode : public rclcpp::Node {
public:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_bgr;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_gray;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_mask;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_steering;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_speed;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_bgr_rs;

    rclcpp::TimerBase::SharedPtr tim_routine;

    // ---Configs
    std::string image_save_dir;
    std::string csv_path;
    std::string camera_path;
    int camera_fps;
    int camera_width;
    int camera_height;
    int8_t use_temporal_model = 0;

    // ---Variables
    cv::Mat image_bgr;
    cv::Mat image_processed;
    std::deque<cv::Mat> frame_queue;

    float speed = 0.0f;
    float steering = 0.0f;

    // ONNX
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::string model_path;
    std::string temporal_model_path;

    //---clock
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    rclcpp::Time start_time;

    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    const char* input_name_;
    const char* output_names_cstr_[2];

    HelpLogger logger;

    OnnxInferenceNode()
        : Node("OnnxInferenceNode")
    {
        if (!logger.init()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        temporal_model_path = "/home/iris/model_temporal_steering_speed.onnx";

        this->declare_parameter<std::string>("image_save_dir", "/home/iris/dataset/images");
        this->get_parameter("image_save_dir", image_save_dir);
        this->declare_parameter<std::string>("csv_path", "/home/iris/dataset/labels.csv");
        this->get_parameter("csv_path", csv_path);
        this->declare_parameter<std::string>("model", "/home/iris/model_16_juni.onnx");
        this->get_parameter("model", model_path);
        this->declare_parameter<std::string>("camera_path", "/dev/video0");
        this->get_parameter("camera_path", camera_path);
        this->declare_parameter<int>("camera_fps", 30);
        this->get_parameter("camera_fps", camera_fps);
        this->declare_parameter<int>("camera_width", 640);
        this->get_parameter("camera_width", camera_width);
        this->declare_parameter<int>("camera_height", 480);
        this->declare_parameter<int8_t>("use_temporal_model", 0);
        this->get_parameter("use_temporal_model", use_temporal_model);
        model_path = "/home/iris/model.onnx";

        if (use_temporal_model < 0 || use_temporal_model > 1) {
            RCLCPP_ERROR(this->get_logger(), "Invalid use_temporal_model value: %d. Must be 0 or 1.", use_temporal_model);
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "use_temporal_model: %d", use_temporal_model);

        if (!std::filesystem::exists(model_path)) {
            RCLCPP_ERROR(this->get_logger(), "Model file does not exist: %s", model_path.c_str());
            rclcpp::shutdown();
        }

        if (!std::filesystem::exists(temporal_model_path)) {
            RCLCPP_ERROR(this->get_logger(), "Temporal model file does not exist: %s", temporal_model_path.c_str());
            rclcpp::shutdown();
        }

        // Inisialisasi ONNX Runtime
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ONNXDemo");
        session_options_ = std::make_unique<Ort::SessionOptions>();
        session_options_->SetIntraOpNumThreads(1);
        session_options_->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        if (use_temporal_model) {
            model_path = temporal_model_path;
        }

        session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), *session_options_);
        // Ambil input & output names
        input_names_ = session_->GetInputNames();
        output_names_ = session_->GetOutputNames();

        input_name_ = input_names_[0].c_str();
        output_names_cstr_[0] = output_names_[0].c_str();
        output_names_cstr_[1] = output_names_[1].c_str();

        start_time = clock->now();

        // // Memory info ONNX
        // memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        pub_image_bgr = this->create_publisher<sensor_msgs::msg::Image>("/onnx_inference/image_bgr", 1);
        pub_image_gray = this->create_publisher<sensor_msgs::msg::Image>("/onnx_inference/image_gray", 1);
        pub_image_mask = this->create_publisher<sensor_msgs::msg::Image>("/onnx_inference/image_mask", 1);
        pub_target_steering = this->create_publisher<std_msgs::msg::Float32>("/onnx_inference/target_steering", 1);
        pub_target_speed = this->create_publisher<std_msgs::msg::Float32>("/onnx_inference/target_speed", 1);

        sub_image_bgr_rs = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rs2_cam_main/color/image_raw", 1, std::bind(&OnnxInferenceNode::callback_sub_image_bgr_rs, this, std::placeholders::_1));

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&OnnxInferenceNode::callback_routine, this));

        logger.info("OnnxInferenceNode node initialized");
    }

    ~OnnxInferenceNode()
    {
    }

    void callback_sub_image_bgr_rs(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            image_bgr = cv_ptr->image.clone();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void callback_routine()
    {
        if (image_bgr.empty()) {
            RCLCPP_WARN(this->get_logger(), "Image is empty");
            return;
        }

        // wait for 5 seconds to ensure the image is ready
        rclcpp::Time current_time = clock->now();
        if (current_time.seconds() - start_time.seconds() < 5.0) {
            return;
        }

        cv::Mat frame_bgr = image_bgr.clone();
        cv::Mat frame;

        // crop image from top 250 pixels and left and right 120 pixels
        cv::Rect roi(15, 200, frame_bgr.cols - 30, frame_bgr.rows - 200);
        frame_bgr = frame_bgr(roi);

        // Resize image to WIDTH x HEIGHT
        cv::resize(frame_bgr, frame_bgr, cv::Size(WIDTH, HEIGHT));
        // Convert image to float and normalize to [0, 1]
        frame_bgr.convertTo(frame, CV_32F, 1.0 / 255.0);

        // catat waktu inferensi
        auto start_inference = std::chrono::high_resolution_clock::now();

        if (use_temporal_model)
            temporal_steering_speed(frame);
        else
            normal_steering_speed(frame);

        auto end_inference = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> inference_duration = end_inference - start_inference;
        // logger.info("Inference time: %.2f ms || Speed: %.2f, Steering: %.2f || temporal: %d ", inference_duration.count(), speed, steering, use_temporal_model);

        // low pass filter for steering
        static float last_steering = 0.0f;
        float alpha = 0.07f; // Adjust alpha for smoothing
        steering = alpha * steering + (1 - alpha) * last_steering;
        last_steering = steering;

        // Publish image
        sensor_msgs::msg::Image image_msg;
        cv_bridge::CvImage image_cv;
        image_cv.header.stamp = this->now();
        image_cv.header.frame_id = "camera_color_optical_frame";
        image_cv.encoding = sensor_msgs::image_encodings::BGR8;
        image_cv.image = frame_bgr;
        image_msg = *image_cv.toImageMsg();
        pub_image_bgr->publish(image_msg);

        std_msgs::msg::Float32 msg_target_steering;
        msg_target_steering.data = last_steering * 1.4;
        pub_target_steering->publish(msg_target_steering);

        std_msgs::msg::Float32 msg_target_speed;
        msg_target_speed.data = speed;
        pub_target_speed->publish(msg_target_speed);
    }

    void
    temporal_steering_speed(const cv::Mat& frame)
    {
        float* speed_ptr = nullptr;
        float* steering_ptr = nullptr;

        std::vector<float> input_tensor_values(SEQ_LEN * HEIGHT * WIDTH * 3);

        frame_queue.push_back(frame.clone());
        if (frame_queue.size() < SEQ_LEN) {
            RCLCPP_INFO(this->get_logger(), "Waiting for enough frames (%lu/%d)...", frame_queue.size(), SEQ_LEN);
            return; // belum cukup frame
        } else if (frame_queue.size() > SEQ_LEN) {
            frame_queue.pop_front(); // jaga agar hanya 10 frame
        }

        float* ptr = input_tensor_values.data();
        for (const auto& f : frame_queue) {
            memcpy(ptr, f.data, HEIGHT * WIDTH * 3 * sizeof(float));
            ptr += HEIGHT * WIDTH * 3;
        }
        // Tensor dengan shape: (1, SEQ_LEN, HEIGHT, WIDTH, 3)
        std::array<int64_t, 5> input_shape = { 1, SEQ_LEN, HEIGHT, WIDTH, 3 };
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            input_tensor_values.data(),
            input_tensor_values.size(),
            input_shape.data(),
            input_shape.size());

        // Jalankan inferensi (2 output)
        auto output_tensors = session_->Run(Ort::RunOptions { nullptr },
            &input_name_, &input_tensor, 1,
            output_names_cstr_, 2);

        // Ambil hasil
        speed_ptr = output_tensors[1].GetTensorMutableData<float>();
        steering_ptr = output_tensors[0].GetTensorMutableData<float>();

        speed = speed_ptr[0]; // Ambil nilai speed dari output
        steering = steering_ptr[0]; // Ambil nilai steering dari output
    }

    void normal_steering_speed(const cv::Mat& frame)
    {
        float* speed_ptr = nullptr;
        float* steering_ptr = nullptr;

        std::vector<float> input_tensor_values(WIDTH * HEIGHT * 3);
        memcpy(input_tensor_values.data(), frame.data, sizeof(float) * input_tensor_values.size());

        // Bentuk input shape: {1, HEIGHT, WIDTH, 3}
        std::array<int64_t, 4> input_shape = { 1, HEIGHT, WIDTH, 3 };

        // Buat tensor ONNX
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            input_tensor_values.data(),
            input_tensor_values.size(),
            input_shape.data(),
            input_shape.size());

        // Jalankan inferensi (2 output)
        auto output_tensors = session_->Run(Ort::RunOptions { nullptr },
            &input_name_, &input_tensor, 1,
            output_names_cstr_, 2);

        // Ambil hasil
        speed_ptr = output_tensors[0].GetTensorMutableData<float>();
        steering_ptr = output_tensors[1].GetTensorMutableData<float>();

        speed = speed_ptr[0]; // Ambil nilai speed dari output
        steering = steering_ptr[0]; // Ambil nilai steering dari output

        // logger.info("Speed: %.2f, Steering: %.2f", speed, steering);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node_OnnxInferenceNode = std::make_shared<OnnxInferenceNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_OnnxInferenceNode);
    executor.spin();

    return 0;
}