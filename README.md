# Zuppo — Map creation & LiDAR Calibration

This following covers two workflows:

1. **Map generation** — build a cost map from LiDAR log data using ROS 2.
2. **Calibration** — calibrate the horizontal and slanted LiDARs.

---

## Prerequisites

- ROS 2 Jazzy installed at `/opt/ros/jazzy`
- A built workspace with `install/setup.bash` available
- The `lidar_publisher` and `map_builder` packages present in your workspace

---

## 1. Generating the Map

This workflow uses three terminals running concurrently. Each command must be executed in the **root floder**

### Terminal 1 — Build the workspace

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select lidar_publisher map_builder
```

### Terminal 2 — Run the cost map node

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run map_builder map_builder --ros-args -p resolution:=0.02 -p max_range:=12.0
```

### Terminal 3 — Replay LiDAR data

Choose **one** of the following, depending on which dataset you want to replay:

```bash
ros2 run lidar_publisher lidar_publisher \
  --ros-args \
  -p map_path:=dataset/calibration/Hall/hall_log.txt \
  -p lidar_idx:=0 \
  -p halt_line:=33442
```

* **Note:**
  * `halt_line`: The last line before the robot starts moving.
  * `max_range`: The LiDAR maximum range in meters.

### After the replay finishes

Back in **Terminal 1**, save the generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f assets/map/my_map --fmt png
```

This produces `assets/map/my_map.png` (plus the accompanying YAML metadata file).

---

## 2. Calibration

### Step 1 — Configure parameters

If needed, edit the relevant parameters in `params.h` before building.

### Step 2 — Install dependencies

Make the setup script executable, then run it:

```bash
chmod +x setup.sh
./setup.sh
```

### Step 3 — Build the calibrator

```bash
cd src/zuppo_calibrator && mkdir build && cd build
cmake ../
make -j $(nproc)
```

### Step 4 — Run the calibrator

`zuppo_calibrator` takes the following arguments:

```
./zuppo_calibrator <path_to_image: str> <resolution: float> <path_to_data:str> <plot:bool>
```

Example runs (from /build):

```bash
./zuppo_calibrator ../../../assets/map/Hall/hall.png 0.02 ../../../dataset/calibration/Hall/hall_log.txt true
```

---

## Doxygen documentation

Doxygen documentation can be found [here]().