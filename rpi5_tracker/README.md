# RPi 5 Lock-On Tracker

Standalone C++ port of the Python app's drone mode for the Raspberry Pi 5.
Click the video to lock a fixed-size box onto that point; a lightweight
tracker (CSRT / KCF / sparse optical flow) follows it every frame and the
HUD reports lock age and loss count for reliability testing. No neural
networks, no GPU — pure CPU, single core.

## Build (on the Pi 5, Raspberry Pi OS Bookworm)

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev
cd rpi5_tracker
cmake -B build
cmake --build build -j4
```

Debian's `libopencv-dev` includes the contrib `tracking` module (CSRT/KCF),
so no custom OpenCV build is needed.

## Run

```bash
./build/lockon                       # USB webcam at /dev/video0, 640x480, CSRT
./build/lockon -b=kcf -s=100        # KCF backend, 100 px lock box
./build/lockon -c=1 --width=1280 --height=720
```

| Key | Action |
| --- | ------ |
| left click | lock onto that point |
| `1` / `2` / `3` | switch backend: CSRT / KCF / optical flow (resets lock) |
| `r` | reset |
| `q` / ESC | quit |

HUD colors: **green** = locked, **orange** = coasting through a momentary
loss, **red** = lost (15 consecutive failed frames). `age` counts frames
tracked since lock; `losses` counts loss episodes the tracker recovered from.

## Expected performance (640x480)

| Backend | FPS on Pi 5 | Character |
| ------- | ----------- | --------- |
| KCF | 60+ | fastest, weaker on scale change / occlusion |
| CSRT | 30–60 | most accurate of the three |
| Optical flow | 60+ | forward-backward validated LK, handles partial occlusion well |

## Pi Camera Module note

`cv::VideoCapture(0, CAP_V4L2)` works out of the box for USB webcams. The
Pi Camera Module uses libcamera; either run
`libcamera-vid --inline -t 0 -o - | …` through a GStreamer pipeline, or use
a USB webcam for testing.
