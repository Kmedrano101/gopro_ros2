<div align="center">

# 📷 gopro_ros2

**ROS 2 C++ driver for real-time GoPro camera streaming with NVIDIA hardware-accelerated decode**

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![GoPro Hero 13](https://img.shields.io/badge/GoPro-Hero%2013%20Black-0080FF?logo=gopro&logoColor=white)](https://gopro.com/en/us/cameras/hero13-black)
[![NVIDIA Jetson](https://img.shields.io/badge/NVIDIA-Jetson%20AGX%20Orin-76B900?logo=nvidia&logoColor=white)](https://developer.nvidia.com/embedded/jetson-agx-orin)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

<br/>

Streams video from multiple GoPro cameras connected via **USB-C** and publishes `sensor_msgs/Image` topics at up to **~15 Hz** using GStreamer with **NVIDIA NVDEC** hardware-accelerated **H.265 (HEVC)** decode.

</div>

---

## 🔑 Key Features

- **🎥 Multi-camera support** — streams up to 8 GoPro cameras simultaneously, each on its own thread
- **⚡ Hardware-accelerated decode** — `nvv4l2decoder` on NVIDIA Jetson for H.265 (HEVC) via GStreamer
- **🔄 Software fallback** — automatic `avdec_h265` fallback if HW decode is unavailable
- **🔌 USB-C Ethernet** — communicates over CDC-NCM (Ethernet-over-USB), not WiFi
- **♻️ Auto-reconnect** — re-initializes stream and GStreamer pipeline on connection loss
- **💓 Keepalive** — background thread pings the camera HTTP API to prevent stream timeout
- **🎯 Webcam API with fallback** — tries the Webcam API (H.264/1080p) first, falls back to Preview Stream (H.265) automatically
- **📊 Built-in Hz monitoring** — logs per-camera publish rate every 5 seconds for live performance tracking

## 📐 Architecture

```
┌──────────────┐   USB-C    ┌──────────────────────────────────────────────┐
│  GoPro #1    │◄──(NCM)──► │  gopro_camera_node                          │
│  172.2X.51   │            │                                              │
└──────────────┘   HTTP     │  ┌─────────────┐    ┌─────────────────────┐ │
                   8080     │  │ Stream Mgr  │───►│ Reader Thread #1    │ │
┌──────────────┐            │  │ (libcurl)   │    │ GStreamer → OpenCV  │──►  /gopro/camera_0/image_raw
│  GoPro #2    │◄──(NCM)──► │  │ start/stop  │    │ nvv4l2decoder (HW)  │ │
│  172.2Y.51   │            │  │ keepalive   │    └─────────────────────┘ │
└──────────────┘            │  └─────────────┘    ┌─────────────────────┐ │
                            │                     │ Reader Thread #2    │──►  /gopro/camera_1/image_raw
                            │                     │ GStreamer → OpenCV  │ │
                            │                     │ nvv4l2decoder (HW)  │ │
                            │                     └─────────────────────┘ │
                            └──────────────────────────────────────────────┘
```

## 🛠️ Prerequisites

| Requirement | Version | Notes |
|---|---|---|
| **ROS 2** | Humble | Tested on Ubuntu 22.04 / L4T |
| **OpenCV** | ≥ 4.5 | With GStreamer support (`cv2.getBuildInformation()`) |
| **GStreamer** | ≥ 1.20 | Including `gstreamer1.0-plugins-bad` for `tsdemux`, `h265parse` |
| **libcurl** | ≥ 7.80 | `sudo apt install libcurl4-openssl-dev` |
| **cv_bridge** | Humble | `sudo apt install ros-humble-cv-bridge` |
| **NVIDIA GStreamer** | L4T | `nvv4l2decoder`, `nvvidconv` (Jetson only, optional) |

### GoPro Setup

- **GoPro Hero 13 Black** (or Hero 9/10/11/12) with latest firmware
- Connected via **USB-C** — camera appears as a network interface (`usb0`, `usb1`)
- Camera USB mode set to **GoPro Connect** (not MTP)

## 📦 Installation

### 1. Clone into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/Kmedrano101/gopro_ros2.git
```

### 2. Install dependencies

```bash
sudo apt update
sudo apt install libcurl4-openssl-dev libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad
```

### 3. Build

```bash
cd ~/ros2_ws
colcon build --packages-select gopro_ros2
source install/setup.bash
```

## 🚀 Quick Start

### 1. Verify GoPro USB connection

Each GoPro connected via USB-C creates a CDC-NCM network interface. The camera IP follows the pattern `172.2X.1YZ.51` based on the last 3 digits of the serial number.

```bash
# Check USB network interfaces
ip a | grep usb

# Ping your cameras
ping -c1 172.22.152.51   # Front camera
ping -c1 172.21.106.51   # Back camera
```

### 2. Configure cameras

Edit the config file to match your camera IPs and serial numbers:

```bash
nano ~/ros2_ws/src/gopro_ros2/config/gopro_cameras.yaml
```

### 3. (Optional) Set camera properties via HTTP API

Before launching, you can configure camera settings. The settings API endpoint is:

```
GET http://{camera_ip}:8080/gopro/camera/setting?setting={ID}&option={VALUE}
```

Recommended settings for computer vision:

```bash
# Set both cameras to 1080p, Linear FOV, HyperSmooth Off
for IP in 172.22.152.51 172.21.106.51; do
  curl "http://$IP:8080/gopro/camera/setting?setting=2&option=9"     # 1080p
  curl "http://$IP:8080/gopro/camera/setting?setting=121&option=4"   # Linear FOV
  curl "http://$IP:8080/gopro/camera/setting?setting=135&option=0"   # HyperSmooth Off
  curl "http://$IP:8080/gopro/camera/setting?setting=59&option=0"    # Auto Power Down: Never
done
```

<details>
<summary><b>📋 Full Camera Settings Reference</b></summary>

| Setting | ID | Options |
|---|---|---|
| **Resolution** | 2 | `1`=4K, `9`=1080p, `12`=720p, `100`=5.3K |
| **Frame Rate** | 234 | `8`=30fps, `9`=25fps, `10`=24fps, `5`=60fps |
| **Video FOV** | 121 | `0`=Wide, `4`=Linear, `2`=Narrow, `9`=HyperView |
| **HyperSmooth** | 135 | `0`=Off, `1`=Low, `2`=High, `3`=Boost |
| **Anti-Flicker** | 134 | `2`=60Hz, `3`=50Hz |
| **Color Profile** | 184 | `0`=Standard, `2`=GP-Log |
| **Bit Rate** | 182 | `0`=Standard, `1`=High |
| **Bit Depth** | 183 | `0`=8-bit, `2`=10-bit |
| **Horizon Level** | 150 | `0`=Off, `2`=Locked |
| **Auto Power Down** | 59 | `0`=Never, `4`=5min, `7`=30min |
| **GPS** | 83 | `0`=Off, `1`=On |
| **LED** | 91 | `4`=All Off, `2`=On |

> **Note:** Not all setting combinations are valid. Query available options with `GET /gopro/camera/state`.

</details>

### 4. Launch

```bash
ros2 launch gopro_ros2 gopro_cameras.launch.py
```

### 5. Verify

```bash
# Check topics are publishing
ros2 topic list | grep gopro

# The node logs publish rate automatically every 5 seconds:
#   [gopro_front] publish rate: 14.8 Hz (total: 1482 frames)
#   [gopro_back]  publish rate: 15.0 Hz (total: 1500 frames)

# You can also measure externally:
ros2 topic hz /gopro/camera_0/image_raw
ros2 topic hz /gopro/camera_1/image_raw

# View in RViz
rviz2 -d $(ros2 pkg prefix gopro_ros2)/share/gopro_ros2/rviz/gopro_cameras.rviz
```

## ⚙️ Configuration

### `config/gopro_cameras.yaml`

```yaml
gopro_cameras:
  ros__parameters:
    target_fps: 15              # Target publish rate per camera
    use_hw_decode: true         # Use nvv4l2decoder (Jetson HW accel)
    use_webcam_api: true        # Try Webcam API first (H.264), fallback to Preview (H.265)
    webcam_resolution: 12       # Webcam API resolution: 4=480p, 7=720p, 12=1080p
    keepalive_interval_s: 2.0   # HTTP keepalive ping interval
    reconnect_delay_s: 3.0      # Wait before reconnecting after failure

    camera_0:
      name: "gopro_front"
      ip: "172.22.152.51"
      serial: "C3531350017152"
      udp_port: 8554

    camera_1:
      name: "gopro_back"
      ip: "172.21.106.51"
      serial: "C3531350027106"
      udp_port: 8556
```

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target_fps` | int | 15 | Target frame rate per camera |
| `use_hw_decode` | bool | true | Use NVIDIA HW decoder (falls back to SW) |
| `use_webcam_api` | bool | true | Try Webcam API before Preview Stream |
| `webcam_resolution` | int | 12 | Webcam API resolution (`4`=480p, `7`=720p, `12`=1080p) |
| `keepalive_interval_s` | double | 2.0 | Keepalive HTTP request interval |
| `reconnect_delay_s` | double | 3.0 | Delay before reconnection attempt |

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/gopro/camera_0/image_raw` | `sensor_msgs/Image` (BGR8) | First camera frames |
| `/gopro/camera_1/image_raw` | `sensor_msgs/Image` (BGR8) | Second camera frames |

> Topic naming follows the pattern `/gopro/camera_{N}/image_raw` where N is the camera index.

## 📊 Performance

Measured on **NVIDIA Jetson AGX Orin** with two GoPro Hero 13 Black cameras via USB-C:

| Resolution | Camera 0 Hz | Camera 1 Hz | Decode |
|---|---|---|---|
| 1920×1440 (mixed) | ~10 Hz | ~16 Hz | H.265 HW (nvv4l2decoder) |
| **1920×1080 (matched)** | **~14 Hz** | **~15 Hz** | **H.265 HW (nvv4l2decoder)** |

> Setting both cameras to the same resolution (1080p) yields the most balanced performance.

## 🔧 GStreamer Pipeline

The node uses the following GStreamer pipeline per camera:

```
udpsrc port={PORT}
  ! tsdemux name=demux demux.
  ! video/x-h265
  ! h265parse
  ! nvv4l2decoder          # NVIDIA HW decode (Jetson)
  ! nvvidconv
  ! video/x-raw,format=BGRx
  ! videoconvert
  ! video/x-raw,format=BGR
  ! appsink max-buffers=1 drop=true sync=false
```

Software fallback replaces `nvv4l2decoder ! nvvidconv ! BGRx` with `avdec_h265`.

## 📁 Package Structure

```
gopro_ros2/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── gopro_cameras.yaml        # Camera IPs, ports, parameters
├── include/gopro_ros2/
│   └── gopro_stream.hpp          # Stream manager + pipeline builder
├── launch/
│   └── gopro_cameras.launch.py
├── rviz/
│   └── gopro_cameras.rviz        # RViz config for both cameras
└── src/
    ├── gopro_camera_node.cpp     # ROS2 node with per-camera reader threads
    └── gopro_stream.cpp          # GoPro HTTP API (libcurl) + GStreamer pipelines
```

## 🐛 Troubleshooting

<details>
<summary><b>Webcam API returns HTTP 409</b></summary>

The camera may be in an incompatible state. The node automatically falls back to Preview Stream. To fix webcam mode:

```bash
curl "http://{IP}:8080/gopro/webcam/exit"
curl "http://{IP}:8080/gopro/camera/control/wired_usb?p=1"
curl "http://{IP}:8080/gopro/webcam/start?port=8554"
```

</details>

<details>
<summary><b>No UDP packets received</b></summary>

```bash
# Check the camera is reachable
ping -c1 172.22.152.51

# Manually start stream and check for packets
curl "http://172.22.152.51:8080/gopro/camera/stream/start?port=8554"
timeout 3 python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2.0)
s.bind(('0.0.0.0', 8554))
data, addr = s.recvfrom(4096)
print(f'Received {len(data)} bytes from {addr}')
"
```

</details>

<details>
<summary><b>GStreamer pipeline fails to open</b></summary>

Check GStreamer elements are installed:

```bash
gst-inspect-1.0 nvv4l2decoder   # HW decoder (Jetson only)
gst-inspect-1.0 h265parse       # H.265 parser
gst-inspect-1.0 tsdemux         # MPEG-TS demuxer
```

Verify OpenCV has GStreamer support:

```bash
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep GStreamer
```

</details>

<details>
<summary><b>Low frame rate</b></summary>

- Ensure `webcam_resolution: 12` (1080p) so all cameras stream at the same resolution
- Ensure `use_hw_decode: true` — software decode is significantly slower
- Check the built-in Hz logs to identify which camera is underperforming
- Check CPU/GPU load with `tegrastats` (Jetson) or `htop`

</details>

## 📝 License

MIT License — see [LICENSE](LICENSE) for details.

## 🙏 Acknowledgements

- [Open GoPro](https://github.com/gopro/OpenGoPro) — GoPro HTTP API documentation
- [gopro_as_webcam_on_linux](https://github.com/jschmid1/gopro_as_webcam_on_linux) — Community Linux GoPro streaming
- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) — Colorization algorithm reference
