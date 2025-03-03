# VisionSync-ROS2
VisionSync: Real-Time Webcam Image Processing and Color Filtering

## BananaCam
### Build
```bash
cd ~/ros2_ws
colcon build --packages-select banana_webcam_driver
source install/setup.bash

```
### Run
```bash
ros2 run banana_webcam_driver webcam_driver
```
## LimeFilter
### Build
```bash
cd ~/ros2_ws
colcon build --packages-select blue_color_filter
source install/setup.bash
```

### Run
```bash
ros2 run blue_color_filter color_filter --ros-args --param h_low:=35 --param h_high:=85 --param s_low:=50 --param s_high:=255 --param v_low:=50 --param v_high:=255
```
## Others
### Rebuild Package
```bash
colcon build --symlink-install
source install/setup.bash
```

### Restart Nodes
```bash
ros2 run blue_color_filter color_filter
ros2 run banana_webcam_driver webcam_driver
```
### RQT Image View
```bash
ros2 run rqt_image_view rqt_image_view
```
