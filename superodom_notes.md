# Super Odometry — Notes & Analysis

> Notes gathered while debugging why Super Odometry works on the CMU-Scafie dataset but drifts on a custom dataset (`data/accuracy_test_free_1`) where the LiDAR is mounted upside down.

---

## 1. Pipeline Overview

Super Odometry is a LiDAR-inertial odometry (LIO) system with three ROS 2 nodes in the `super_odometry` package:

```mermaid
flowchart LR
    A[livox_ros_driver2<br>/livox/lidar<br>/livox/imu] --> B[feature_extraction_node]
    B -->|/feature_info| C[laser_mapping_node]
    C -->|/laser_odometry| D[imu_preintegration_node]
    D -->|/state_estimation| E[RViz / downstream]
```

| Node | Role |
|------|------|
| `feature_extraction_node` | Receives raw LiDAR (`CustomMsg`) + IMU, deskews the point cloud using IMU orientation, extracts features, publishes them with an initial pose estimate. |
| `laser_mapping_node` | Scan-to-map ICP matching (Ceres), maintains a local map, publishes laser odometry at LiDAR rate. |
| `imu_preintegration_node` | GTSAM IMU preintegration; fuses laser odometry with IMU to produce high-rate odometry (`/state_estimation`). |

---

## 2. Topics

### Inputs (from the driver / bag)
| Topic | Type | Notes |
|-------|------|-------|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Raw LiDAR scan (per-point `offset_time` used for deskewing). |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Raw IMU (Livox internal IMU, ~200 Hz). |

> The config defaults are `/imu/data` and `/lidar/scan` — remap or edit `config/livox_mid360.yaml` to match the bag topics.

### Outputs (all prefixed by `ProjectName`, empty by default)
| Topic | Type | Publisher | Purpose |
|-------|------|-----------|---------|
| `/velodyne_cloud_2` | `PointCloud2` | feature_extraction | Deskewed full-resolution cloud. |
| `/feature_info` | `super_odometry_msgs/msg/LaserFeature` | feature_extraction | Corner/surface features + initial IMU pose. |
| `/edge_points` | `PointCloud2` | feature_extraction | Edge (line) features. |
| `/planner_points` | `PointCloud2` | feature_extraction | Surface (plane) features. |
| `/bob_points` | `PointCloud2` | feature_extraction | Depth/Realsense points (unused for Livox). |
| `/laser_odometry` | `nav_msgs/msg/Odometry` | laser_mapping | Scan-to-map odometry (LiDAR rate). |
| `/aft_mapped_to_init_incremental` | `nav_msgs/msg/Odometry` | laser_mapping | Incremental pose. |
| `/laser_odom_path` | `nav_msgs/msg/Path` | laser_mapping | Trajectory. |
| `/laser_cloud_map` / `/laser_cloud_surround` | `PointCloud2` | laser_mapping | Local map visualization. |
| `/registered_scan` | `PointCloud2` | laser_mapping | Scan transformed to map frame. |
| `/super_odometry_stats` | `super_odometry_msgs/msg/OptimizationStats` | laser_mapping | Iteration/optimization stats. |
| `/prediction_source` | `std_msgs/msg/String` | laser_mapping | Which prediction source was used. |
| `/state_estimation` | `nav_msgs/msg/Odometry` | imu_preintegration | Fused high-rate odometry (main output). |
| `/state_estimation_health` | `std_msgs/msg/Bool` | imu_preintegration | Health flag. |
| `/imuodom_path` | `nav_msgs/msg/Path` | imu_preintegration | IMU-only path. |

---

## 3. Configuration Files

### `config/livox_mid360.yaml` (main parameters)
```yaml
imu_topic: "/imu/data"          # must match bag
laser_topic: "/lidar/scan"      # must match bag
sensor: "livox"
use_imu_roll_pitch: false       # Livox MID360 does not use roll/pitch from IMU orientation
save_ply: false

world_frame: "map"
world_frame_rot: "map_rot"
sensor_frame: "sensor"
sensor_frame_rot: "sensor_rot"

imu_acc_x_limit: 0.5
imu_acc_y_limit: 0.2
imu_acc_z_limit: 0.4

feature_extraction_node:
    scan_line: 4                # Livox MID360 has 4 scan lines
    min_range: 0.2
    filter_point_size: 3

laser_mapping_node:
    mapping_line_resolution: 0.1
    mapping_plane_resolution: 0.1
    max_iterations: 5
    max_surface_features: 4000
    localization_mode: false
    read_pose_file: false
    init_x/y/z/roll/pitch/yaw: 0.0

imu_preintegration_node:
    lidar_correction_noise: 0.01
    acc_n: 3.9939570888238808e-03   # accelerometer noise std
    gyr_n: 1.5636343949698187e-03   # gyroscope noise std
    acc_w: 6.4356659353532566e-05   # acc bias random walk
    gyr_w: 3.5640318696367613e-05   # gyro bias random walk
    g_norm: 9.80511                 # gravity magnitude for GTSAM preintegration
```

### `config/livox/livox_mid360_calibration.yaml` (extrinsics)
```yaml
# Rotation from laser frame to IMU frame: imu^R_laser
extrinsicRotation_imu_laser: [1,0,0, 0,1,0, 0,0,1]   # identity by default
# Translation from laser frame to IMU frame: imu^T_laser
extrinsicTranslation_imu_laser: [-0.011, -0.02329, 0.04412]
# Manual RPY offset (degrees), applied as: R_offset * R_extrinsic
imu_laser_rotation_offset: [0.0, 0.0, 0.0]
yaw_ratio: 0.0
```

---

## 4. Key Internal Concepts

### 4.1 Gravity-aligned frame (`imu_laser_R_Gravity`)
The pipeline works in a **gravity-aligned frame** (gravity along $-Z$), not the raw IMU frame. During IMU initialization (`imuInit()` in `include/super_odometry/sensor_data/imu/imu_data.h`):

```cpp
gravity = -acc_mean / acc_mean.norm() * Gravity_Norm;   // 9.81
Roll_Pitch_Gravity_Matrix = calculatePitchRollMatrix(acc_mean.x(), acc_mean.y(), acc_mean.z());
imu_laser_R_Gravity = Roll_Pitch_Gravity_Matrix.inverse() * imu_laser_R;
```

- `calculatePitchRollMatrix()` derives roll/pitch from the mean accelerometer reading (assumes ~1 s of stationary data).
- `imu_laser_R_Gravity` maps IMU/lidar-frame measurements into the gravity-aligned frame.
- It is used in:
  - `featureExtraction::livoxHandler()` — rotates raw LiDAR points.
  - `featureExtraction::createImuData()` — rotates accel (and should rotate gyro).
  - `imuPreintegration::imuConverter()` — rotates gyro/accel/orientation.

### 4.2 IMU initialization
- Requires **~1 second of stationary IMU data** before `imuInit()` runs.
- Computes `acc_mean`, `gyr_bias`, `acc_bias`, and the gravity-alignment matrix.
- If the bag starts with motion, the alignment is wrong → drift.

### 4.3 Deskewing (`removePointDistortion`)
- Interpolates IMU poses (`q_w_i`) at each point's timestamp.
- Transforms points from the start-of-scan frame to the end-of-scan frame using `T_l_i * T_original_current * T_i_l`.
- Requires IMU orientation (`q_w_i`) to be in the **same frame** as the LiDAR points (both rotated by `imu_laser_R_Gravity`).

### 4.4 Prediction sources (laser_mapping)
`determinePredictionSource()` picks, in order:
1. `LIO_ODOM` (if available)
2. `VIO_ODOM` (if degenerate)
3. `NEURAL_IMU_ODOM` (if degenerate)
4. `IMU_ORIENTATION` (roll/pitch from IMU)
5. `CONSTANT_VELOCITY` (fallback)

---

## 5. Known Bug Found (upside-down mounting)

### `featureExtraction::createImuData()` — gyro not rotated
```cpp
if(IMU_INIT && config_.sensor == SensorType::LIVOX) {
    double gravity = imu_Init->gravity_norm;
    Eigen::Vector3d gyr = imu_Init->imu_laser_R_Gravity * measurement.gyr;  // computed...
    Eigen::Vector3d accel = imu_Init->imu_laser_R_Gravity * measurement.accel;
    imudata->acc = accel * gravity / imu_Init->acc_mean.norm();
} else {
    imudata->acc = measurement.accel;
}
imudata->gyr = measurement.gyr;   // BUG: raw, unrotated gyro overwrites `gyr`
```

**Effect:**
- Right-side-up (CMU-Scafie): `imu_laser_R_Gravity ≈ I` → bug invisible.
- Upside-down: `imu_laser_R_Gravity` is a 180° rotation → gyro is 180° off → `updateImuOrientation()` integrates wrong orientation → corrupted deskewing → drift.

**Fix applied:**
```cpp
if(IMU_INIT && config_.sensor == SensorType::LIVOX) {
    double gravity = imu_Init->gravity_norm;
    Eigen::Vector3d gyr = imu_Init->imu_laser_R_Gravity * measurement.gyr;
    Eigen::Vector3d accel = imu_Init->imu_laser_R_Gravity * measurement.accel;
    imudata->acc = accel * gravity / imu_Init->acc_mean.norm();
    imudata->gyr = gyr;              // FIX: use rotated gyro
} else {
    imudata->acc = measurement.accel;
    imudata->gyr = measurement.gyr;
}
```

### Other potential issues
- `calculatePitchRollMatrix()` cannot distinguish roll-180° from pitch-180° (both give `acc_mean ≈ [0,0,-9.81]`); it always assumes roll 180°.
- Gravity magnitude inconsistency: `g_norm: 9.80511` (config) vs hardcoded `9.8105` (`imu_data.h`, `correctLivoxGravity()`).
- Extrinsic calibration is identity by default — if the IMU–LiDAR relationship is not truly identity, set `imu_laser_rotation_offset` (e.g. `[180, 0, 0]` for a roll-flipped mount).

---

## 6. Debugging Tips

1. **Check console output** from `imuInit()`:
   - `Roll Pitch Gravity Matrix` and `imu_laser_extrinsic_gravity` — if roll offset ≈ 180°, upside-down detection is working.
   - `Accelerometer Mean` — `[0, 0, -9.81]` means IMU is upside down.
2. **RViz**: if the deskewed cloud looks flipped/mirrored, the `imu_laser_R_Gravity` rotation isn't applied consistently.
3. **Topic check**: `ros2 topic list` and verify `/livox/lidar` + `/livox/imu` are remapped to `/lidar/scan` + `/imu/data`.
4. **Bag start**: ensure the bag begins with ~1 s of stationary IMU data for correct gravity alignment.

---

## 7. Build / Run

```bash
# Build
colcon build --symlink-install

# Source
source install/setup.bash

# Launch (bringup package)
ros2 launch bringup super.launch.py

# Play bag
ros2 bag play data/accuracy_test_free_1
```
