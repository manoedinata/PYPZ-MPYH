#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include <onnxruntime_cxx_api.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

#define WIDTH 160
#define HEIGHT 120

class MLDetection : public rclcpp::Node
{
  public:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_bgr;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_gray;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_mask;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_bgr_rs;

    rclcpp::TimerBase::SharedPtr tim_routine;

    // ---Configs
    std::string image_save_dir;
    std::string csv_path;
    std::string camera_path;
    int camera_fps;
    int camera_width;
    int camera_height;

    // ---Variables
    cv::Mat image_bgr;
    cv::Mat image_processed;

    // ONNX
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info_;

    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    const char *input_name_;
    const char *output_names_cstr_[2];

    HelpLogger logger;

    MLDetection()
        : Node("MLDetection")
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }
        this->declare_parameter<std::string>("image_save_dir", "/home/iris/dataset/images");
        this->get_parameter("image_save_dir", image_save_dir);
        this->declare_parameter<std::string>("csv_path", "/home/iris/dataset/labels.csv");
        this->get_parameter("csv_path", csv_path);
        this->declare_parameter<std::string>("camera_path", "/dev/video0");
        this->get_parameter("camera_path", camera_path);

        // Inisialisasi ONNX Runtime
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ONNXDemo");
        session_options_ = std::make_unique<Ort::SessionOptions>();
        session_options_->SetIntraOpNumThreads(1);

        // Load model ONNX
        std::string model_path = this->declare_parameter<std::string>("model_path", "model.onnx");
        session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), *session_options_);

        // Ambil input & output names
        input_names_ = session_->GetInputNames();
        output_names_ = session_->GetOutputNames();

        input_name_ = input_names_[0].c_str();
        output_names_cstr_[0] = output_names_[0].c_str();
        output_names_cstr_[1] = output_names_[1].c_str();

        // Memory info ONNX
        memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        pub_image_bgr = this->create_publisher<sensor_msgs::msg::Image>("/vision_capture/image_bgr", 1);
        pub_image_gray = this->create_publisher<sensor_msgs::msg::Image>("/vision_capture/image_gray", 1);
        pub_image_mask = this->create_publisher<sensor_msgs::msg::Image>("/vision_capture/image_mask", 1);

        sub_image_bgr_rs = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rs2_cam_main/color/image_raw", 1,
            std::bind(&MLDetection::callback_sub_image_bgr_rs, this, std::placeholders::_1));

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&MLDetection::callback_routine, this));

        logger.info("MLDetection node initialized");
    }

    ~MLDetection()
    {
    }

    void callback_sub_image_bgr_rs(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            image_bgr = cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void callback_routine()
    {
        cv::Mat frame = image_bgr.clone();
        if (frame.rows > 250)
            frame = frame(cv::Range(250, frame.rows), cv::Range::all());

        cv::resize(frame, frame, cv::Size(WIDTH, HEIGHT));
        frame.convertTo(frame, CV_32F, 1.0 / 255.0);

        std::vector<float> input_tensor_values(WIDTH * HEIGHT * 3);
        memcpy(input_tensor_values.data(), frame.data, sizeof(float) * input_tensor_values.size());

        // Bentuk input shape: {1, HEIGHT, WIDTH, 3}
        std::array<int64_t, 4> input_shape = {1, HEIGHT, WIDTH, 3};

        // Buat tensor ONNX
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info_,
            input_tensor_values.data(),
            input_tensor_values.size(),
            input_shape.data(),
            input_shape.size());

        // Jalankan inferensi (2 output)
        auto output_tensors = session_->Run(Ort::RunOptions{nullptr},
                                            &input_name_, &input_tensor, 1,
                                            output_names_cstr_, 2);

        // Ambil hasil
        float *speed_ptr = output_tensors[0].GetTensorMutableData<float>();
        float *steering_ptr = output_tensors[1].GetTensorMutableData<float>();

        float speed = speed_ptr[0];
        float steering = steering_ptr[0];

        image_processed = frame.clone();

        cv::putText(image_processed, "Speed: " + std::to_string(speed), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::putText(image_processed, "Steering: " + std::to_string(steering), cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // Publish image
        sensor_msgs::msg::Image image_msg;
        cv_bridge::CvImage image_cv;
        image_cv.header.stamp = this->now();
        image_cv.header.frame_id = "camera_color_optical_frame";
        image_cv.encoding = sensor_msgs::image_encodings::BGR8;
        image_cv.image = image_processed;
        image_msg = *image_cv.toImageMsg();
        pub_image_bgr->publish(image_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_MLDetection = std::make_shared<MLDetection>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_MLDetection);
    executor.spin();

    return 0;
}