# Data Pipeline

## Data Recording Package

### sync_image_saver

Recording synchronized image pairs and save to `</path/to/output>/cam0/` and `</path/to/output>/cam1/`

```bash
ros2 run data_recording sync_image_saver --ros-args -p cam0_topic:=<cam0_topic> -p cam1_topic:=<cam1_topic> -p output_dir:=</path/to/output/>

## Example
ros2 run data_recording sync_image_saver --ros-args -p cam0_topic:=/camera/realsense/color/image_raw -p cam1_topic:=/pmd_royale_ros_camera_node/gray_image_0 -p output_dir:=/home/yunjinli/camera_calibration/rel_calib_images/2 -p queue_size:=30 -p slop:=0.005 -p save_formet:=png
```
