#include "gopro_ros2/gopro_stream.hpp"

#include <curl/curl.h>
#include <sstream>

namespace gopro_ros2
{

// ── GStreamer pipeline builder ───────────────────────────────────────────────

std::string build_gstreamer_pipeline(int udp_port, StreamMode mode, bool hw_decode)
{
    std::ostringstream ss;

    // UDP source → MPEG-TS demux → queue
    ss << "udpsrc port=" << udp_port << " "
       << "! tsdemux name=demux demux. ";

    // Parser depends on codec
    if (mode == StreamMode::WEBCAM) {
        ss << "! video/x-h264 ! h264parse ! ";
    } else {
        ss << "! video/x-h265 ! h265parse ! ";
    }

    // Decoder
    if (hw_decode) {
        ss << "nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx "
           << "! videoconvert ! video/x-raw,format=BGR ";
    } else {
        if (mode == StreamMode::WEBCAM) {
            ss << "avdec_h264 ! videoconvert ! video/x-raw,format=BGR ";
        } else {
            ss << "avdec_h265 ! videoconvert ! video/x-raw,format=BGR ";
        }
    }

    // App sink — drop old frames, no sync
    ss << "! appsink max-buffers=1 drop=true sync=false";

    return ss.str();
}

// ── libcurl write callback (discard response body) ──────────────────────────

static size_t curl_discard_cb(void* /*ptr*/, size_t size, size_t nmemb, void* /*userdata*/)
{
    return size * nmemb;
}

// ── GoProStreamManager ──────────────────────────────────────────────────────

GoProStreamManager::GoProStreamManager(const CameraConfig& config,
                                       double keepalive_interval_s,
                                       LogFunc log_info,
                                       LogFunc log_warn)
    : config_(config)
    , keepalive_interval_s_(keepalive_interval_s)
    , log_info_(std::move(log_info))
    , log_warn_(std::move(log_warn))
{
}

GoProStreamManager::~GoProStreamManager()
{
    stop_stream();
}

bool GoProStreamManager::http_get(const std::string& url, long timeout_ms)
{
    CURL* curl = curl_easy_init();
    if (!curl) {
        log_warn_("[" + config_.name + "] curl_easy_init failed");
        return false;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 5000L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_discard_cb);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK) {
        log_warn_("[" + config_.name + "] HTTP request failed: " + url +
                  " - " + curl_easy_strerror(res));
        return false;
    }

    if (http_code != 200) {
        log_warn_("[" + config_.name + "] HTTP " + std::to_string(http_code) +
                  " from " + url);
        return false;
    }

    return true;
}

bool GoProStreamManager::start_stream(StreamMode mode)
{
    mode_ = mode;
    std::string url;

    if (mode == StreamMode::WEBCAM) {
        // Exit any existing webcam session first
        http_get("http://" + config_.ip + ":8080/gopro/webcam/exit", 5000);

        url = "http://" + config_.ip + ":8080/gopro/webcam/start?port=" +
              std::to_string(config_.udp_port);
        log_info_("[" + config_.name + "] Starting webcam stream -> UDP port " +
                  std::to_string(config_.udp_port));
    } else {
        url = "http://" + config_.ip + ":8080/gopro/camera/stream/start?port=" +
              std::to_string(config_.udp_port);
        log_info_("[" + config_.name + "] Starting preview stream -> UDP port " +
                  std::to_string(config_.udp_port));
    }

    if (http_get(url)) {
        log_info_("[" + config_.name + "] Stream start OK (" +
                  (mode == StreamMode::WEBCAM ? "webcam/H.264" : "preview/H.265") + ")");
        start_keepalive();
        return true;
    }

    // If webcam API fails, try preview as fallback
    if (mode == StreamMode::WEBCAM) {
        log_warn_("[" + config_.name + "] Webcam API failed, falling back to preview stream");
        mode_ = StreamMode::PREVIEW;
        url = "http://" + config_.ip + ":8080/gopro/camera/stream/start?port=" +
              std::to_string(config_.udp_port);
        if (http_get(url)) {
            log_info_("[" + config_.name + "] Preview stream fallback OK (H.265)");
            start_keepalive();
            return true;
        }
    }

    log_warn_("[" + config_.name + "] Stream start FAILED");
    return false;
}

void GoProStreamManager::stop_stream()
{
    stop_keepalive();

    if (mode_ == StreamMode::WEBCAM) {
        http_get("http://" + config_.ip + ":8080/gopro/webcam/stop", 5000);
        http_get("http://" + config_.ip + ":8080/gopro/webcam/exit", 5000);
    } else {
        http_get("http://" + config_.ip + ":8080/gopro/camera/stream/stop", 5000);
    }

    log_info_("[" + config_.name + "] Stream stopped");
}

void GoProStreamManager::start_keepalive()
{
    running_ = true;
    keepalive_thread_ = std::thread(&GoProStreamManager::keepalive_loop, this);
}

void GoProStreamManager::stop_keepalive()
{
    running_ = false;
    if (keepalive_thread_.joinable()) {
        keepalive_thread_.join();
    }
}

void GoProStreamManager::keepalive_loop()
{
    const std::string url = "http://" + config_.ip + ":8080/gopro/camera/state";
    const auto interval = std::chrono::milliseconds(
        static_cast<int>(keepalive_interval_s_ * 1000));

    while (running_) {
        std::this_thread::sleep_for(interval);
        if (!running_) break;

        if (!http_get(url, 5000)) {
            log_warn_("[" + config_.name + "] Keepalive failed");
        }
    }
}

}  // namespace gopro_ros2
