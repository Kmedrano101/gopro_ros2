#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <functional>

namespace gopro_ros2
{

struct CameraConfig
{
    std::string name;
    std::string ip;
    std::string serial;
    int udp_port{8554};
    int index{0};
};

enum class StreamMode
{
    WEBCAM,   // /gopro/webcam/start — H.264 1080p
    PREVIEW   // /gopro/camera/stream/start — H.265 1440p
};

/// Builds a GStreamer pipeline string for OpenCV VideoCapture.
/// @param udp_port   UDP port the GoPro streams to
/// @param mode       WEBCAM (H.264) or PREVIEW (H.265)
/// @param hw_decode  true = nvv4l2decoder (Jetson), false = software decode
std::string build_gstreamer_pipeline(int udp_port, StreamMode mode, bool hw_decode);

/// Manages HTTP API interactions with a single GoPro camera.
class GoProStreamManager
{
public:
    using LogFunc = std::function<void(const std::string&)>;

    GoProStreamManager(const CameraConfig& config, double keepalive_interval_s,
                       LogFunc log_info, LogFunc log_warn);
    ~GoProStreamManager();

    // Non-copyable
    GoProStreamManager(const GoProStreamManager&) = delete;
    GoProStreamManager& operator=(const GoProStreamManager&) = delete;

    /// Start the stream via HTTP API. Returns true on success.
    bool start_stream(StreamMode mode);

    /// Stop the stream and keepalive thread.
    void stop_stream();

    /// Returns the active stream mode.
    StreamMode active_mode() const { return mode_; }

private:
    bool http_get(const std::string& url, long timeout_ms = 10000);
    void start_keepalive();
    void stop_keepalive();
    void keepalive_loop();

    CameraConfig config_;
    double keepalive_interval_s_;
    StreamMode mode_{StreamMode::WEBCAM};
    LogFunc log_info_;
    LogFunc log_warn_;

    std::atomic<bool> running_{false};
    std::thread keepalive_thread_;
};

}  // namespace gopro_ros2
