#include "gopro_ros2/gopro_stream.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>

#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

namespace gopro_ros2
{

struct CameraHandle
{
    CameraConfig config;
    std::unique_ptr<GoProStreamManager> stream_mgr;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr raw_pub;
    std::unique_ptr<cv::VideoCapture> capture;

    bool hw_decode{true};
    StreamMode mode{StreamMode::WEBCAM};
    std::atomic<uint64_t> frame_count{0};
    std::mutex mtx;  // protects capture
};

class GoProCameraNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    GoProCameraNode()
        : LifecycleNode("gopro_cameras")
    {
        declare_parameter("target_fps", 15);
        declare_parameter("use_hw_decode", true);
        declare_parameter("use_webcam_api", true);
        declare_parameter("keepalive_interval_s", 2.0);
        declare_parameter("reconnect_delay_s", 3.0);
        declare_parameter("webcam_resolution", 12);  // 4=480p, 7=720p, 12=1080p
        declare_parameter("jpeg_quality", 80);
        declare_parameter("use_compressed", true);

        // Auto-transition: configure then activate (matching Livox pattern)
        RCLCPP_INFO(get_logger(), "Auto-transitioning to active state...");

        auto configure_ret = trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        if (configure_ret.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            throw std::runtime_error("Failed to auto-configure");
        }

        auto activate_ret = trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        if (activate_ret.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            throw std::runtime_error("Failed to auto-activate");
        }

        RCLCPP_INFO(get_logger(), "Lifecycle auto-transition complete (active).");
    }

    ~GoProCameraNode() override
    {
        RCLCPP_INFO(get_logger(), "Destroying GoPro camera node...");
        // Safety net: if still active, stop everything
        if (active_.load()) {
            active_ = false;
            for (auto& t : reader_threads_) {
                if (t.joinable()) t.join();
            }
            for (auto& cam : cameras_) {
                std::lock_guard<std::mutex> lock(cam->mtx);
                if (cam->capture) {
                    cam->capture->release();
                    cam->capture.reset();
                }
                cam->stream_mgr->stop_stream();
            }
        }
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override
    {
        RCLCPP_INFO(get_logger(), "Configuring...");

        target_fps_ = get_parameter("target_fps").as_int();
        hw_decode_ = get_parameter("use_hw_decode").as_bool();
        use_webcam_ = get_parameter("use_webcam_api").as_bool();
        keepalive_s_ = get_parameter("keepalive_interval_s").as_double();
        reconnect_delay_s_ = get_parameter("reconnect_delay_s").as_double();
        webcam_resolution_ = get_parameter("webcam_resolution").as_int();
        jpeg_quality_ = get_parameter("jpeg_quality").as_int();
        use_compressed_ = get_parameter("use_compressed").as_bool();

        discover_cameras();

        if (cameras_.empty()) {
            RCLCPP_ERROR(get_logger(), "No cameras configured! Check parameters.");
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(get_logger(), "Configured %zu camera(s).", cameras_.size());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override
    {
        RCLCPP_INFO(get_logger(), "Activating...");
        active_ = true;

        // Start each camera in its own thread (stream init + pipeline open)
        std::vector<std::thread> start_threads;
        for (auto& cam : cameras_) {
            start_threads.emplace_back([this, &cam]() {
                start_camera(*cam);
            });
        }
        for (auto& t : start_threads) {
            t.join();
        }

        // Launch a dedicated reader thread per camera
        for (auto& cam : cameras_) {
            auto* cam_ptr = cam.get();
            reader_threads_.emplace_back([this, cam_ptr]() {
                reader_loop(*cam_ptr);
            });
        }

        // Hz monitoring timer — log publish rate every 5 seconds
        prev_frame_counts_.resize(cameras_.size(), 0);
        hz_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                for (size_t i = 0; i < cameras_.size(); ++i) {
                    uint64_t current = cameras_[i]->frame_count.load();
                    uint64_t delta = current - prev_frame_counts_[i];
                    double hz = static_cast<double>(delta) / 5.0;
                    prev_frame_counts_[i] = current;
                    RCLCPP_INFO(get_logger(),
                        "[%s] publish rate: %.1f Hz (total: %lu frames)",
                        cameras_[i]->config.name.c_str(), hz, current);
                }
            });

        RCLCPP_INFO(get_logger(), "GoPro camera node activated: %zu camera(s), "
                     "target %d FPS, res=%d, compressed=%s (q=%d), %zu reader threads",
                     cameras_.size(), target_fps_, webcam_resolution_,
                     use_compressed_ ? "true" : "false", jpeg_quality_,
                     reader_threads_.size());

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override
    {
        RCLCPP_INFO(get_logger(), "Deactivating...");
        active_ = false;

        // Join reader threads
        for (auto& t : reader_threads_) {
            if (t.joinable()) t.join();
        }
        reader_threads_.clear();

        // Cancel hz timer
        hz_timer_.reset();

        // Release captures and stop streams
        for (auto& cam : cameras_) {
            RCLCPP_INFO(get_logger(), "[%s] Stopping (published %lu frames)",
                         cam->config.name.c_str(),
                         cam->frame_count.load());
            {
                std::lock_guard<std::mutex> lock(cam->mtx);
                if (cam->capture) {
                    cam->capture->release();
                    cam->capture.reset();
                }
            }
            cam->stream_mgr->stop_stream();
            cam->frame_count = 0;

            // Reset publishers so they stop publishing in inactive state
            if (cam->compressed_pub) cam->compressed_pub.reset();
            if (cam->raw_pub) cam->raw_pub.reset();
        }

        RCLCPP_INFO(get_logger(), "Deactivated.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) override
    {
        RCLCPP_INFO(get_logger(), "Cleaning up...");
        cameras_.clear();
        prev_frame_counts_.clear();
        RCLCPP_INFO(get_logger(), "Cleaned up.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override
    {
        RCLCPP_INFO(get_logger(), "Shutting down...");

        if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            on_deactivate(previous_state);
        }
        if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
            previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            on_cleanup(previous_state);
        }

        RCLCPP_INFO(get_logger(), "Shut down complete.");
        return CallbackReturn::SUCCESS;
    }

private:
    void discover_cameras()
    {
        for (int i = 0; i < 8; ++i) {
            std::string prefix = "camera_" + std::to_string(i);
            declare_parameter(prefix + ".name", std::string(""));
            declare_parameter(prefix + ".ip", std::string(""));
            declare_parameter(prefix + ".serial", std::string(""));
            declare_parameter(prefix + ".udp_port", 0);

            auto name = get_parameter(prefix + ".name").as_string();
            auto ip   = get_parameter(prefix + ".ip").as_string();
            auto port = get_parameter(prefix + ".udp_port").as_int();

            if (name.empty() || ip.empty() || port == 0) continue;

            auto serial = get_parameter(prefix + ".serial").as_string();

            CameraConfig cfg;
            cfg.name = name;
            cfg.ip = ip;
            cfg.serial = serial;
            cfg.udp_port = static_cast<int>(port);
            cfg.index = i;
            cfg.resolution = webcam_resolution_;

            auto log_info = [this](const std::string& msg) {
                RCLCPP_INFO(get_logger(), "%s", msg.c_str());
            };
            auto log_warn = [this](const std::string& msg) {
                RCLCPP_WARN(get_logger(), "%s", msg.c_str());
            };

            auto stream_mgr = std::make_unique<GoProStreamManager>(
                cfg, keepalive_s_, log_info, log_warn);

            auto qos = rclcpp::QoS(5)
                .reliability(rclcpp::ReliabilityPolicy::Reliable)
                .durability(rclcpp::DurabilityPolicy::Volatile);

            auto handle = std::make_unique<CameraHandle>();
            handle->config = cfg;
            handle->stream_mgr = std::move(stream_mgr);
            handle->hw_decode = hw_decode_;
            handle->mode = use_webcam_ ? StreamMode::WEBCAM : StreamMode::PREVIEW;

            std::string topic_base = "/gopro/camera_" + std::to_string(i);
            if (use_compressed_) {
                handle->compressed_pub =
                    create_publisher<sensor_msgs::msg::CompressedImage>(
                        topic_base + "/image_raw/compressed", qos);
            } else {
                handle->raw_pub =
                    create_publisher<sensor_msgs::msg::Image>(
                        topic_base + "/image_raw", qos);
            }

            cameras_.push_back(std::move(handle));

            std::string topic = use_compressed_
                ? topic_base + "/image_raw/compressed"
                : topic_base + "/image_raw";
            RCLCPP_INFO(get_logger(),
                "Configured %s (%s) -> UDP:%d -> %s",
                name.c_str(), ip.c_str(), static_cast<int>(port),
                topic.c_str());
        }
    }

    void start_camera(CameraHandle& cam)
    {
        StreamMode requested = cam.mode;
        if (!cam.stream_mgr->start_stream(requested)) {
            RCLCPP_ERROR(get_logger(), "[%s] Failed to start stream",
                          cam.config.name.c_str());
            return;
        }
        cam.mode = cam.stream_mgr->active_mode();

        // Wait for UDP packets to arrive
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        open_capture(cam);
    }

    void open_capture(CameraHandle& cam)
    {
        std::lock_guard<std::mutex> lock(cam.mtx);

        if (cam.capture) {
            cam.capture->release();
            cam.capture.reset();
        }

        std::string pipeline = build_gstreamer_pipeline(
            cam.config.udp_port, cam.mode, cam.hw_decode);
        RCLCPP_INFO(get_logger(), "[%s] Opening GStreamer: %s",
                     cam.config.name.c_str(), pipeline.c_str());

        auto cap = std::make_unique<cv::VideoCapture>(pipeline, cv::CAP_GSTREAMER);
        if (cap->isOpened()) {
            cam.capture = std::move(cap);
            RCLCPP_INFO(get_logger(), "[%s] Pipeline opened",
                         cam.config.name.c_str());
            return;
        }
        cap->release();

        // Try software decode fallback
        if (cam.hw_decode) {
            RCLCPP_WARN(get_logger(), "[%s] HW decode failed, trying software",
                         cam.config.name.c_str());
            std::string sw_pipeline = build_gstreamer_pipeline(
                cam.config.udp_port, cam.mode, false);
            auto cap_sw = std::make_unique<cv::VideoCapture>(
                sw_pipeline, cv::CAP_GSTREAMER);
            if (cap_sw->isOpened()) {
                cam.capture = std::move(cap_sw);
                cam.hw_decode = false;
                RCLCPP_INFO(get_logger(),
                    "[%s] Software decode pipeline opened",
                    cam.config.name.c_str());
                return;
            }
            cap_sw->release();
        }

        RCLCPP_ERROR(get_logger(), "[%s] All pipeline attempts failed",
                      cam.config.name.c_str());
    }

    /// Dedicated reader loop per camera — runs in its own thread.
    /// read() blocks until next frame is available (rate-limited by GStreamer
    /// appsink drop=true), so no explicit sleep is needed.
    void reader_loop(CameraHandle& cam)
    {
        int consecutive_failures = 0;

        RCLCPP_INFO(get_logger(), "[%s] Reader thread started",
                     cam.config.name.c_str());

        while (active_.load()) {
            bool need_reconnect = false;

            {
                std::lock_guard<std::mutex> lock(cam.mtx);

                if (!cam.capture || !cam.capture->isOpened()) {
                    consecutive_failures++;
                    need_reconnect = (consecutive_failures == 1);
                } else {
                    cv::Mat frame;
                    if (cam.capture->read(frame) && !frame.empty()) {
                        consecutive_failures = 0;
                        cam.frame_count++;

                        auto stamp = this->get_clock()->now();
                        if (use_compressed_) {
                            std::vector<uchar> buf;
                            std::vector<int> params = {
                                cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
                            cv::imencode(".jpg", frame, buf, params);

                            sensor_msgs::msg::CompressedImage msg;
                            msg.header.stamp = stamp;
                            msg.header.frame_id = cam.config.name;
                            msg.format = "jpeg";
                            msg.data = std::move(buf);
                            cam.compressed_pub->publish(msg);
                        } else {
                            auto msg = cv_bridge::CvImage(
                                std_msgs::msg::Header(), "bgr8", frame)
                                .toImageMsg();
                            msg->header.stamp = stamp;
                            msg->header.frame_id = cam.config.name;
                            cam.raw_pub->publish(*msg);
                        }
                    } else {
                        consecutive_failures++;
                        need_reconnect =
                            (consecutive_failures > target_fps_ * 5);
                    }
                }
            }  // release mutex

            if (need_reconnect) {
                reconnect_camera(cam);
                consecutive_failures = 0;
            }

            // Brief yield if no capture (avoid busy spin during reconnect)
            if (consecutive_failures > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        RCLCPP_INFO(get_logger(), "[%s] Reader thread stopped",
                     cam.config.name.c_str());
    }

    void reconnect_camera(CameraHandle& cam)
    {
        RCLCPP_INFO(get_logger(), "[%s] Attempting reconnect...",
                     cam.config.name.c_str());

        {
            std::lock_guard<std::mutex> lock(cam.mtx);
            if (cam.capture) {
                cam.capture->release();
                cam.capture.reset();
            }
        }

        cam.stream_mgr->stop_stream();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        StreamMode requested = use_webcam_ ? StreamMode::WEBCAM : StreamMode::PREVIEW;
        if (cam.stream_mgr->start_stream(requested)) {
            cam.mode = cam.stream_mgr->active_mode();
            std::this_thread::sleep_for(
                std::chrono::duration<double>(reconnect_delay_s_));
            open_capture(cam);
        } else {
            RCLCPP_ERROR(get_logger(),
                "[%s] Reconnect failed", cam.config.name.c_str());
        }
    }

    // Parameters
    int target_fps_{15};
    bool hw_decode_{true};
    bool use_webcam_{true};
    double keepalive_s_{2.0};
    double reconnect_delay_s_{3.0};
    int webcam_resolution_{12};
    int jpeg_quality_{80};
    bool use_compressed_{true};

    std::vector<std::unique_ptr<CameraHandle>> cameras_;
    std::vector<std::thread> reader_threads_;
    std::atomic<bool> active_{false};

    // Hz monitoring
    rclcpp::TimerBase::SharedPtr hz_timer_;
    std::vector<uint64_t> prev_frame_counts_;
};

}  // namespace gopro_ros2

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gopro_ros2::GoProCameraNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
