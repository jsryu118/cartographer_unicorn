# 📌 Cartographer Configuration Overview

[🚀 **Cartographer ROS Parameters 바로가기**](#🔹-cartographer_ros)

[🚀 **Cartographer Parameters 바로가기**](#🔹-cartographer)

[🚀 **파라미터 튜닝 전략 바로가기**](#🔥-📈-파라미터-튜닝-전략)

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
```
- `use_pose_extrapolator` : Local trajectory의 결과를 쓰는 것이 아니라 현재 시간의 포즈를 `pose_extrapolator` 에서 가져와서 사용
- `*sampling_ratio` : ROS Level sampling
- `tracking_frame` : 실제로 Cartographer에서 추정하는 프레임, imu_data의 프레임과 translation이 있으면 안됨. rotation은 가능
- `published_frame` : `publish_to_tf`가 true라면 `map_frame`과 `published_frame`사이의 tf publish함
- `publish_tracked_pose` : `tracked_pose`토픽은 tracking frame의 포즈를 나타냄
- `use_odometry` : `use_odometry` 토픽 내의 twist 속도값을 사용안함. relative pose 정보만 사용함
- `imu_sampling_ratio` : imu에서 6축 정보만 사용(linear_acceleration, angular_velocity)


## 🔹 **Cartographer**
### 🔹 **Map Builder Options**
```lua
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4
MAP_BUILDER.pose_graph = POSE_GRAPH
MAP_BUILDER.collate_by_trajectory = false
```
- `collate_by_trajectory` : if true, collate sensor data for each trajectory

### 🔹 **Trajectory Builder 2d - Laser Preprocess**
`num_subdivisions_per_laser_scan`에 따라 나누어진 subdivision이 input
```lua
TRAJECTORY_BUILDER_2D.min_range = 0.
TRAJECTORY_BUILDER_2D.max_range = 30.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
```
- `min_range` 보다 작은 포인트는 버림
- `max_range` 보다 작은 포인트는 `pose_extrapolator`에서 포즈를 받아서 global position으로 변환 후 HIT 했다고 저장
- `max_range` 보다 큰 포인트는 `pose_extrapolator`에서 포즈를 받아서 `missing_data_ray_length`에 따라 global position으로 변환 후 MISS 했다고 저장, 즉 rviz에서 흰색만 채워지고 검정색은 영향안주는것.
```lua
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_z = -0.8
TRAJECTORY_BUILDER_2D.max_z = 2.
```
- `num_accumulated_range_data`만큼 subdivision 수집, 즉 10개로 나누고, 10개를 모아야한다면 안나누고 한개를 바로 사용하는것과 같다.
- 충분히 모였다면 `extrapolator`에서 예측된 중력 방향 가져오기(imu_linear_acceleration 사용 추정)
- 중력방향이 z축이 되도록 tranform하기
- `min_z` 와 `max_z`에 따라서 crop하기
```lua
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.
```
- `voxel_filter_size` 크기에 따라 필터링 적용
- 최소한 `min_num_points`포인트를 가지도록 adaptive_voxel_filter 적용
  - `max_range` 이 값은 센서의 원점이 아니라 맵의 원점과의 거리를 기준으로 필터링 !!!!
  - 필터링 안해도 `min_num_points`보다 작다면 return
  - 먼저 `max_length`크기의 복셀을 적용해보고 `min_num_points`보다 크다면 return
  - 너무 많이 필터링이 되었다면 최대한 `min_num_points` 가지도록 적절한 voxel size binary Search

### 🔹 **Trajectory Builder 2d - Scan Matcing**
First Stage(Optional) - Real-Time Correlative Scan Matching (RTC-SM)
Initial Guess of Ceres ScanMatcher를 Improve하기 위한 과정
```lua
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
```
- `use_online_correlative_scan_matching` true일때만 수행됨
Second Stage(Optional) - Ceres ScanMatcher
```lua
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1
```
- `*weight`들은 차원이 없고 서로간의 비율임.


### 🔹 **Trajectory Builder 2d - Submap**
**Motion Filter**
```lua
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5. 
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.) -- math.rad(0.1)
```
- 두 이전의 결과와 비교하여 motion filter에 넣어서 그차이가 작다면 서브맵에 넣지 않는다.
- 이전에 서브맵에 추가된 포즈와 현재 계산된 포즈 데이터 사이에 시간 차이가 `max_time_seconds`보다 작으면서 position의 차이가 `max_distance_meters`보다 작으면서 orientation의 차이가 `max_angle_radians`보다 작으면 Similar한것으로 return

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
- Motion Filter 에서 similar하지 않다고 한다면 서브맵에 추가
- 가장 최근의 서브맵에 `num_range_data` 만큼의 데이터가 모이면 새로운 서브맵 추가
- Active 서브맵이 2개 보다 많다면 오래된 서브맵 FINISH
- 또는 가장 오래된 Active submap이 `num_range_data`x2 만큼의 데이터가 모이면 서브맵 FINISH
- ex. 40hz 라이다 `num_subdivisions_per_laser_scan`=10개로 나누고 `num_accumulated_range_data`=10 개 모으면 라이다 hz와 거의 동일하게 작동할것이다. 또한 계속 움직이며 모션 필터를 잘 거친다고 가정하면 90/40 초에 한번씩 서브맵이 생성된다.



### 🔹 **Pose Graph Options**
Scan Matching의 결과가 있다면 pose_grpah에 AddNode호출
1. 새로운 노드를 Trajectory에 추가
2. 새로운 서브맵이 있다면 Trajectory에 추가
3. 마지막으로 ComputeConstraintsForNode를 workqueue 대기줄에 추가
4. 새로운 서브맵이 있다면 Optimization problem에 추가
5. 새로운 노드를 Optimization problem에 추가
```lua
POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3
```
6. 새로운 노드와 새로운 서브맵사이의 Contraint를 Optimization problem에 추가
  - `matcher_translation_weight` : 새롭게 추가되는 노드와 서브맵 사이의 초기 Constraint에서의 가중치
  - `matcher_rotation_weight` : 새롭게 추가되는 노드와 서브맵 사이의 초기 Constraint에서의 가중치

7. 새로운 노드와 기존 서브맵들사이의 Contraint찾기
8. 기존 노드들과 새로운 서브맵사이의 Contraint찾기
```lua
POSE_GRAPH.optimize_every_n_nodes = 90
```
9. 새로운 노드가 추가될때 마다 +1, optimization실행되면 다시 0 초기화
- `optimize_every_n_nodes` : 이전 optimization기준 `optimize_every_n_nodes` 만큼 추가되었다면 최적화실행, localization을 위해서는 많이 낮춰줘야함 ex. 3

### 🔹 **Constraints Builder**
```lua
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.
```
1. 먼저 Local Constraint인지 Global contraint인지 나누기(Global constaint는 pose_extrapolator의 initial guess 사용하는 것이 아니라 실제 모든 구역에서 매칭을 함!!)
  - 실제 아주 빠른 레이싱에서는 Global contraint는 다른 곳으로 localization 결과가 날아갈 수 있음.
  - 그래서 global_sampling_ratio를 0으로 하는게 좋음
  ```lua
  POSE_GRAPH.global_sampling_ratio = 0.0
  ```
  - 노드와 서브맵이 같은 trajectory에 있다면 -> Local Constraint 찾기
  - 노드와 서브맵이 다른 trajectory에 있다면
    - `global_constraint_search_after_n_seconds`안에 다른 trajectory와 연결이 되었었다면 Local Constraint 찾기, 따라서 freeze 된 .pbstream데이터와 계속 매칭을 시켜서 안정적인 localization을 하고 싶다면 값을 키우자(하지만 너무 값을 키우면 0으로 인식되는거 같음...)
  ```lua
  POSE_GRAPH.global_constraint_search_after_n_seconds = 1000.
  ```

2. Matching 하기
First Stage - Fast Correlative Scan Matching (FC-SM)
Initial Guess of Ceres ScanMatcher를 Improve하기 위한 과정
Trajectory 빌드에서는 optional이엿지만 여기서는 필수로 실행이 된다.

Local Constraint
```lua
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.,
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
POSE_GRAPH.constraint_builder.min_score = 0.55
```
1. initial relative Guess의 translation norm이 `max_constraint_distance`보다 크다면 종료
2. `sampling_ratio`의 확률로 실행, local contraint는 자주 호출 되기때문에 샘플링함
3. initial relative Guess와 함께 `linear_search_window`와 `angular_search_window`범위 만큼 지정하여 `fast_correlative_scan_matcher`를 실행한다.
4. 만약 결과 점수가 `min_score`보다 크다면 다음의 ceres scan matcher로 넘어간다.


Global Contraint
```lua
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
```
1. initial guess가 서브맵의 중앙이다.
2. `linear_search_window`는 1e6 * (서브맵의 resolution), `angular_search_window`는 M_PI로 지정이 되어 `fast_correlative_scan_matcher`를 실행한다.
4. 만약 결과 점수가 `global_localization_min_score`보다 크다면 다음의 ceres scan matcher로 넘어간다.

```lua
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1
```

```lua
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
```
fast_correlative_scan_matcher가 성공했다면 ceres는 성공여부 확인안함.
새롭게 추가되는 contraint의 가중치

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
1. Odometry정보는 최적화 문제에 포함됨, Imu 정보는 최적화문제에 포함되지 않음.
4. 새로운 서브맵이 있다면 Optimization problem에 추가

### 🔹 **Localization Trimmer**
```lua
TRAJECTORY_BUILDER.pure_localization_trimmer.max_submaps_to_keep = 3
```
이 트리머는 한개의 trajectory 내에서 서브맵리스트를 가져와서 max 개수를 관리한다.
- `max_submaps_to_keep` : 한 trajectory가 유지할 수 있는 최대 서브맵 개수 (최소 2개)

### 🔹 **overlap Trimmer**
```lua
-- Default OFF
-- POSE_GRAPH.overlapping_submaps_trimmer_2d.min_added_submaps_count = 5
-- POSE_GRAPH.overlapping_submaps_trimmer_2d.fresh_submaps_count = 1
-- POSE_GRAPH.overlapping_submaps_trimmer_2d.min_covered_area = 2
```
이 트리머는 모든 trajectory 에서 완료된(`kFinished`) 서브맵 리스트를 가져와서 오버랩 되는 맵을 삭제한다. 디폴트로 꺼져있음.
- `min_added_submaps_count` : 트리머가 작동하기 위한 최소 서브맵의 개수
- `fresh_submaps_count` : 한 영역을 덮고 있는 서브맵이 여러 개 있을 때, 가장 최신 서브맵을 fresh_submaps_count 개수만큼 유지하고, 그보다 오래된 서브맵은 삭제됨.
- `min_covered_area` : 만약 특정 서브맵이 겹치는 정도가 너무 크고, 그 서브맵의 독립적인 커버 면적이 min_covered_area보다 작다면 삭제됨.

### 🔹 **Trajectory Builder 2d - Pose Extrapolator**
```lua
TRAJECTORY_BUILDER_2D = {
  use_imu_data = true, -- 3d는 Imu 필수
  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,  -- only for ROS
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

- `pose_graph_odometry_motion_filter` 오도메트리 데이터가 너무 자주 추가되는 것을 막는것, 하지만 현재 코드에서는 사용되지 않고 있음.
나중에는 오도메트리 데이터를 포즈 그래프(Pose Graph) 내에서 마지날라이징(Marginalizing, 최적화 과정에서 일부 데이터를 제거) 하는 방식으로 개선할 수도 있다는 내용적혀있음. 
- `use_imu_based` 2d에서는 imu기반 exptrapolator 지원 안함.
- `pose_queue_duration` 이 시간내의 포즈만 저장하고 잇음., 작은 값과 40hz의 포즈 업데이트가 있다면 extrapolator에는 2개의 포즈만 유지될것이다.
  - 가장 최신의 포즈 2개를 통해서 linear_acc, angular_vel을 계산하여, 만약 imu 데이터가 없다면 이것을 사용한다.
- linear 속도와 각속도 추정은 단순히 이전 포즈(odometry)와 최근 포즈(odometry) 간의 차이를 이용해서 한다.
- imu측정값에서 angular_velocity는 가장 최신값만 사용한다.

- linear_vel x del_t, angular_vel x del_t
- linear 속도추정
  1. odometry사용하면 odometry사이의 포즈를 미분해서 사용함.
  2. 아니라면 trajectory builder의 결과의 포즈를 미분해서 사용함.
- angular 속도추정
  1. Imu 사용하면 IMU angular velocity 사용함
  2. Imu 없고 odometry사용하면 odometry사이의 포즈를 미분해서 사용함.
  3. 아니라면 trajectory builder의 결과의 포즈를 미분해서 사용함.

IMU랑 Odometry둘다 사용하면 linear 속도추정은 odometry로 부터, angular는 Imu로 부터 얻게 된다.

- IMU 사용되는 과정
- `imu_tracker`의 메인 목적은 중력벡터 방향을 추정하는것임!!!(중력벡터와 orientation를)
  - imu측정값에서 angular_velocity는 가장 최신값만 사용한다. dt와 곱해서 gravity vector 보정
  - imu측정값에서 Linear Acceleration이 들어오면 중력벡터 방향 보정.
  - `constant_velocity.imu_gravity_time_constant`는 중력 벡터를 새로 들어오는 IMU를 정지 상태라고 가정하기 처리하기때문에 처리하는 정도를 결정
    값이 크다면 천천히 변함, 작으면 노이즈에 민감
    만약에 차량이 튼튼하다면 조금 크게하면 좋음, 차량의 서스펜션이 약해서 롤, 피치 움직임이 많다면 조금 작게하면 좋음.

### 🔹 **Not used in code**
```lua
TRAJECTORY_BUILDER_2D = {
  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },
}

POSE_GRAPH.max_num_final_iterations = 200
POSE_GRAPH.log_residual_histograms = true
```
-  `max_num_final_iterations` : ros node가 종료될 때, 마지막으로 최적화를 실행하는 횟수, 만약 노드가 종료될때 맵을 저장하도록 해두었다면(`-save_state_filename map.pbstream`) 최적화를 시키고 맵을 저장시킴
- `log_residual_histograms` : 3d에서만 사용됨...

### 🔹 **Ceres Scan Matcher Settings**
```lua
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 5
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 0.1
```


## 🔹 **Range Data Inserter Settings**
```lua
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49
```

---
# 🔥 **📈 파라미터 튜닝 전략**
## 📌 **Parameter Study: Optimization Execution Conditions**
### ✅ `RunOptimization()` 실행 조건:
1. `optimize_every_n_nodes` 만큼 노드가 추가되었을 때
2. `FinishTrajectory()` 호출 시
3. `RunFinalOptimization()` 실행 시
4. `DrainWorkQueue`에서 최적화 필요성이 감지될 때
5. Loop Closure가 감지되었을 때
6. 특정 노드와 서브맵 간의 제약조건이 추가되었을 때

### ❌ `RunOptimization()` 실행되지 않는 경우:
1. IMU 데이터 추가 시
2. Odometry 데이터 추가 시
3. Fixed Frame Pose 데이터 추가 시
4. Landmark 데이터 추가 시
5. Frozen된 Trajectory 추가 시
6. Submap 추가 시
7. 새로운 노드 추가 시
8. 특정 Submap을 Trim(삭제)할 때
9. Trajectory 삭제 예약 시

---

## 🔥 **📌 Node 및 Submap 추가 조건**
### 📌 **Node가 추가되는 경우**
- 새로운 Laser Scan 데이터가 수신되었을 때
- Odometry 데이터에 변화가 있을 때

### 📌 **Submap이 추가되는 경우**
1. **서브맵이 없는 경우** → 새로운 서브맵 생성
2. **기존 서브맵이 가득 찼을 경우** → 새로운 서브맵 추가

---

## 🔥 **📌 `num_subdivisions_per_laser_scan`의 역할**
- Lidar 속도가 느리고, 차량 속도가 빠를 때 높이는 것이 좋음.
- 한 개의 Laser Scan을 여러 개의 Subdivision으로 나눔.
- 각 Subdivision 마다 `HandleRangefinder()` 실행.
- Subdivision별 `TF (Laser_frame <-> tracking_frame)`을 `lookup_transform_timeout_sec` 내에 찾지 못하면 오류 발생.
- 서브디비전마다 `AddSensorData()` 실행됨 → `cartographer_ros`에서 `cartographer`로 데이터 전달됨.

---
<!-- ## 🚀 **📌 최적화 전략 요약** -->
✔ `global_sampling_ratio` 값을 조정하여 **Loop Closure 감지 빈도를 최적화**.
✔ `global_constraint_search_after_n_seconds` 값을 낮춰 **Local Constraint 생성 빈도를 증가**.
✔ `max_constraint_distance` 값을 조정하여 **Local Constraint 범위를 최적화**.
✔ `motion_filter`를 적절히 조정하여 **불필요한 노드 생성을 방지**.

---
📌 **이 설정들은 레이싱 환경에서 SLAM/Localization 최적화를 위해 조정 가능! 🚗💨**


## 🔥 **📌 laserscan이 처리되는 과정**
1. `N`개 라이다 포인트는 laser handler에서 `N / num_subdivisions_per_laser_scan`로 나뉘어진다.
2.

## 🔥 **📌 Global Matching과 Local Matching**
노드가 추가될때 마다 서브맵의 개수만큼 실행이 됨.(Global Matching은 initial guess없이 일어난다. 따라서 `POSE_GRAPH.constraint_builder.global_localization_min_score`을 충분히 크게 잡거나 )