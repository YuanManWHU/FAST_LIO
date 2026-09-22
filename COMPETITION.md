# LiDAR-SLAM Challenge 2026 adapter

This branch adapts FAST-LIO for the LiDAR-Inertial sub-track of the 10th National LiDAR Conference Multi-Sensor SLAM Challenge.

## Input

- LiDAR: `/rslidar_front_points` (`sensor_msgs/PointCloud2`, 10 Hz)
- IMU: `/rslidar_front_imu_data` (`sensor_msgs/Imu`, about 200 Hz)
- Airy point fields: `x/y/z/intensity/ring/timestamp/feature`
- `timestamp` is an absolute float64 timestamp in seconds.

The Airy preprocessor converts the absolute point timestamp to FAST-LIO's millisecond offset from the LiDAR header. The original IMU frame is retained so the FAST-LIO state directly represents the Airy internal IMU origin required by the competition.

The official LiDAR-to-IMU calibration from the challenge repository is used with online extrinsic estimation disabled.

## Build

```bash
cd ~/fast_lio_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Run

Scene 1:

```bash
roslaunch fast_lio mapping_competition.launch \
  scene_name:=scene_0001 \
  bagfile:=/media/ym/BLUE/Datasets/全国激光雷达大会数据处理大赛/sequence_01_0000_0550.bag
```

Scene 2:

```bash
roslaunch fast_lio mapping_competition.launch \
  scene_name:=scene_0002 \
  bagfile:=/media/ym/BLUE/Datasets/全国激光雷达大会数据处理大赛/sequence_02_0560_0970.bag
```

Use `output_dir:=...` if a different result directory is required.

## Output

The mapping node records every original LiDAR `header.stamp`. FAST-LIO estimates the IMU-origin state at scan end; the competition output path interpolates between adjacent processed scan-end posterior states to the original LiDAR header timestamps. It writes TUM format:

```text
timestamp tx ty tz qx qy qz qw
```

Results are written to:

```text
<output_dir>/trajectories/
  scene_0001.txt
  scene_0002.txt
```

The output logic rejects non-finite samples and duplicate/non-increasing timestamps. Initialization-prefix or final suffix timestamps outside the available posterior-state history are clamped to the nearest available pose and reported in the ROS log. For the provided competition bags the initialization prefix occurs during the long static start.

Before submission, place a competition-required team `README.md` next to the `trajectories` directory and validate the directory with the organizer's public validator.
