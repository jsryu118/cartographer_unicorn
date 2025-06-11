# 📌 Cartographer Configuration Overview

[🚀 **Go to Cartographer ROS Parameters**](#🔹-cartographer_ros)

[🚀 **Go to Cartographer Parameters**](#🔹-cartographer)

---
## 🔹 **Cartographer_ROS** 
```lua
options = {
  map_builder = MAP_BUILDER,
  map_frame = "map",
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 1e-2,
  trajectory_publish_period_sec = 30e-3,
  publish_to_tf = true, -- default:true
  publish_tracked_pose = true, -- default:false
  -- use_pose_extrapolator = true, -- default:true
  trajectory_builder = TRAJECTORY_BUILDER,
  tracking_frame = "imu_enu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 0,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  rangefinder_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}
````

* `use_pose_extrapolator` : Instead of using the local trajectory results, the pose at the current time is taken from the `pose_extrapolator`.
* `*sampling_ratio` : Sampling ratio at the ROS level.
* `tracking_frame` : The frame actually estimated by Cartographer; should not have translation relative to imu\_data frame but rotation is allowed.
* `published_frame` : If `publish_to_tf` is true, publishes tf between `map_frame` and `published_frame`.
* `publish_tracked_pose` : The `tracked_pose` topic represents the pose of the tracking frame.
* `use_odometry` : Does not use twist velocity values from the `use_odometry` topic; only uses relative pose information.
* `imu_sampling_ratio` : Only uses 6-axis info from the IMU (linear\_acceleration, angular\_velocity).

## 🔹 **Cartographer**

### 🔹 **Map Builder Options**

```lua
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4
MAP_BUILDER.pose_graph = POSE_GRAPH
MAP_BUILDER.collate_by_trajectory = false
```

* `collate_by_trajectory` : If true, collate sensor data by each trajectory.

### 🔹 **Trajectory Builder 2d - Laser Preprocess**

Input is subdivided according to `num_subdivisions_per_laser_scan`.

```lua
TRAJECTORY_BUILDER_2D.min_range = 0.
TRAJECTORY_BUILDER_2D.max_range = 30.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
```

* Points closer than `min_range` are discarded.
* Points closer than `max_range` are converted to global position by taking pose from `pose_extrapolator` and marked as hits.
* Points farther than `max_range` are converted to global position according to `missing_data_ray_length` using pose from `pose_extrapolator` and marked as misses, which results in only white areas being filled in rviz, with black unaffected.

```lua
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_z = -0.8
TRAJECTORY_BUILDER_2D.max_z = 2.
```

* Collect subdivisions equal to `num_accumulated_range_data`. For example, if divided into 10 and you need to collect 10, this is equivalent to not subdividing and using a single scan.
* Once enough data is collected, get the predicted gravity direction from the extrapolator (estimated using imu\_linear\_acceleration).
* Transform so that gravity direction aligns with the z-axis.
* Crop points based on `min_z` and `max_z`.

```lua
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.
```

* Filtering is applied depending on the size of `voxel_filter_size`.
* Adaptive voxel filter is applied to maintain at least `min_num_points`.

  * `max_range` is filtering based on distance from map origin, not sensor origin.
  * If points are fewer than `min_num_points` after filtering, returns immediately.
  * First applies voxel filtering with `max_length`; if number of points is sufficient, returns.
  * Otherwise, performs a binary search on voxel size to keep at least `min_num_points`.

### 🔹 **Trajectory Builder 2d - Scan Matching**

First Stage (Optional) - Real-Time Correlative Scan Matching (RTC-SM)
This step improves the initial guess of the Ceres ScanMatcher.

```lua
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
```

* Executed only if `use_online_correlative_scan_matching` is true.

Second Stage (Optional) - Ceres ScanMatcher

```lua
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1
```

* The `*weight` values are dimensionless and relative ratios.

### 🔹 **Trajectory Builder 2d - Submap**

**Motion Filter**

```lua
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5. 
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.) -- math.rad(0.1)
```

* Compares to previous two results, and if the difference is small, does not add to submap.
* If time difference between last added submap pose and current pose is less than `max_time_seconds`, position difference less than `max_distance_meters`, and orientation difference less than `max_angle_radians`, it is considered similar and is ignored.

**Add Submap**

```lua
TRAJECTORY_BUILDER_2D.submaps.num_range_data=90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49
```

* If motion filter decides not similar, add to submap.
* When the most recent submap collects `num_range_data` pieces of data, a new submap is added.
* If active submaps are more than 2, the oldest submap is finished.
* Or, if the oldest active submap has collected `num_range_data * 2` data, finish the submap.
* For example, with a 40Hz lidar, `num_subdivisions_per_laser_scan` = 10 and `num_accumulated_range_data` = 10, it will operate roughly at lidar Hz. Assuming the motion filter works well, a new submap is generated approximately every 90/40 seconds.

### 🔹 **Pose Graph Options**

If scan matching results exist, `AddNode` is called on the pose graph:

1. Add a new node to the trajectory
2. Add a new submap to the trajectory if available
3. Finally, add `ComputeConstraintsForNode` to the workqueue
4. Add the new submap to the optimization problem if there is one
5. Add the new node to the optimization problem

```lua
POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3
```

6. Add constraints between the new node and new submap to the optimization problem

* `matcher_translation_weight`: weight of the initial constraint between the newly added node and submap for translation
* `matcher_rotation_weight`: weight of the initial constraint between the newly added node and submap for rotation

7. Find constraints between the new node and existing submaps
8. Find constraints between existing nodes and the new submap

```lua
POSE_GRAPH.optimize_every_n_nodes = 90
```

9. Increment by +1 each time a new node is added, reset to 0 after optimization runs

* `optimize_every_n_nodes`: triggers optimization after this many new nodes have been added since the last optimization; for localization purposes, this should be set lower, e.g., 3

---

### 🔹 **Constraints Builder**

```lua
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.
```

1. First, decide whether the constraint is local or global (Global constraints do not use the pose\_extrapolator's initial guess; instead, matching is done across all areas!)

* In very fast racing scenarios, global constraints may cause localization to jump incorrectly.
* Therefore, it is recommended to set `global_sampling_ratio` to 0:

```lua
POSE_GRAPH.global_sampling_ratio = 0.0
```

* If the node and submap belong to the same trajectory → find Local Constraint
* If node and submap belong to different trajectories:

  * If connected within `global_constraint_search_after_n_seconds`, find local constraint. To maintain stable localization by matching against frozen `.pbstream` data continuously, increase this value (but too large values may cause zero recognition issues...)

```lua
POSE_GRAPH.global_constraint_search_after_n_seconds = 1000.
```

2. Matching process
   **First Stage - Fast Correlative Scan Matching (FC-SM)**
   This step improves the initial guess for the Ceres ScanMatcher.
   It is optional in trajectory building but mandatory here.

Local Constraint

```lua
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
POSE_GRAPH.constraint_builder.min_score = 0.55
```

1. If the translation norm of the initial relative guess is greater than `max_constraint_distance`, terminate.
2. Execute with probability `sampling_ratio` since local constraints are called frequently.
3. Run `fast_correlative_scan_matcher` within the specified `linear_search_window` and `angular_search_window` using the initial guess.
4. If the resulting score is above `min_score`, proceed to the Ceres scan matcher.

Global Constraint

```lua
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
```

1. The initial guess is the center of the submap.
2. `linear_search_window` is set to `1e6 * (submap resolution)`, and `angular_search_window` is set to `M_PI` for `fast_correlative_scan_matcher`.
3. If the resulting score is above `global_localization_min_score`, proceed to the Ceres scan matcher.

Ceres Scan Matcher Parameters:

```lua
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1
```

Loop Closure Constraint Weights:

```lua
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
```

If `fast_correlative_scan_matcher` succeeds, Ceres does not verify success.
Weights for the newly added constraints.

---

### 🔹 **Optimization Problem**

```lua
POSE_GRAPH = {
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1.1e2,
    rotation_weight = 1.6e4,
    local_slam_pose_translation_weight = 1e5,
    local_slam_pose_rotation_weight = 1e5,
    odometry_translation_weight = 1e5,
    odometry_rotation_weight = 1e5,
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    fixed_frame_pose_use_tolerant_loss = false,
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,
    log_solver_summary = false,
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
}
```

1. Odometry information is included in the optimization problem; IMU information is not.
2. Add new submaps to the optimization problem if available.

---

### 🔹 **Localization Trimmer**

```lua
TRAJECTORY_BUILDER.pure_localization_trimmer.max_submaps_to_keep = 3
```

This trimmer manages the maximum number of submaps in a single trajectory.

* `max_submaps_to_keep`: maximum number of submaps a trajectory can keep (minimum 2)

---

### 🔹 **Overlap Trimmer**

```lua
-- Default OFF
-- POSE_GRAPH.overlapping_submaps_trimmer_2d.min_added_submaps_count = 5
-- POSE_GRAPH.overlapping_submaps_trimmer_2d.fresh_submaps_count = 1
-- POSE_GRAPH.overlapping_submaps_trimmer_2d.min_covered_area = 2
```

This trimmer deletes overlapping submaps from the finished (`kFinished`) submap list across all trajectories. It is OFF by default.

* `min_added_submaps_count`: minimum number of submaps required to activate the trimmer
* `fresh_submaps_count`: when multiple submaps cover the same area, keep this many most recent submaps and delete older ones
* `min_covered_area`: if a submap overlaps too much and its independent covered area is smaller than this value, it gets deleted

---

### 🔹 **Trajectory Builder 2D - Pose Extrapolator**

```lua
TRAJECTORY_BUILDER_2D = {
  use_imu_data = true, -- IMU is mandatory for 3D
  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,  -- ROS only
  pose_extrapolator = {
    use_imu_based = false,
    constant_velocity = {
      imu_gravity_time_constant = 10.,
      pose_queue_duration = 0.001,
    },
  },
}
-- TRAJECTORY_BUILDER.pose_graph_odometry_motion_filter = { 
--   max_time_seconds = 3,
--   max_distance_meters = 3,
--   max_angle_radians = 3,
-- }
```

* `pose_graph_odometry_motion_filter` prevents odometry data from being added too frequently, but is currently unused in the code.

* It is mentioned that odometry data might later be marginalized within the pose graph optimization process to reduce data size.

* `use_imu_based`: IMU-based extrapolator is not supported in 2D.

* `pose_queue_duration`: Only poses within this duration are stored. With a small value and 40Hz pose updates, only 2 poses are kept in the extrapolator.

  * The two most recent poses are used to calculate linear acceleration and angular velocity. If IMU data is unavailable, this is used.

* Linear and angular velocity estimation is based on the difference between previous and current odometry poses.

* For angular velocity from IMU, only the most recent value is used.

* Linear velocity x delta\_t, angular velocity x delta\_t:

  * Linear velocity estimation:

    1. If odometry is used, differentiate pose between odometries.
    2. Otherwise, differentiate pose results from the trajectory builder.
  * Angular velocity estimation:

    1. Use IMU angular velocity if IMU is available.
    2. If no IMU but odometry exists, differentiate between odometry poses.
    3. Otherwise, differentiate trajectory builder pose results.

* If both IMU and odometry are used, linear velocity is from odometry, angular velocity from IMU.

* IMU usage process:

* The main purpose of `imu_tracker` is to estimate the gravity vector direction (gravity vector and orientation).

  * The most recent angular velocity from IMU measurements is used and multiplied by delta\_t to correct the gravity vector.
  * When linear acceleration is received from IMU, it also corrects the gravity vector direction.
  * `constant_velocity.imu_gravity_time_constant` assumes incoming IMU data corresponds to a stationary state, controlling the gravity vector update speed.
    A larger value means slower change; a smaller value means more sensitive to noise.
    For a stable vehicle, a larger value is recommended; for vehicles with lots of roll/pitch motion (weak suspension), a smaller value is better.

---