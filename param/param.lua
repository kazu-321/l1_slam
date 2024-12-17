include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",  -- マップの固定フレーム
  tracking_frame = "unilidar_imu",  -- トラッキングに使用するIMUフレーム
  published_frame = "base_link",  -- 公開されるフレーム
  odom_frame = "odom",  -- オドメトリフレーム
  provide_odom_frame = false,  -- オドメトリフレームを提供するかどうか
  publish_frame_projected_to_2d = false,  -- フレームを2Dに投影して公開するかどうか
  use_odometry = false,  -- オドメトリを使用するかどうか
  use_nav_sat = false,  -- ナビゲーション衛星を使用するかどうか
  use_landmarks = false,  -- ランドマークを使用するかどうか
  num_laser_scans = 0,  -- 使用するレーザースキャンの数
  num_multi_echo_laser_scans = 0,  -- 使用するマルチエコーレーザースキャンの数
  num_subdivisions_per_laser_scan = 1,  -- レーザースキャンごとのサブディビジョンの数
  num_point_clouds = 1,  -- 使用するポイントクラウドの数
  lookup_transform_timeout_sec = 0.2,  -- トランスフォームのタイムアウト時間
  submap_publish_period_sec = 0.3,  -- サブマップの公開周期
  pose_publish_period_sec = 5e-3,  -- ポーズの公開周期
  trajectory_publish_period_sec = 30e-3,  -- トラジェクトリの公開周期
  rangefinder_sampling_ratio = 1.,  -- レンジファインダーのサンプリング比率
  odometry_sampling_ratio = 1.,  -- オドメトリのサンプリング比率
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定フレームポーズのサンプリング比率
  imu_sampling_ratio = 1.,  -- IMUのサンプリング比率
  landmarks_sampling_ratio = 1.,  -- ランドマークのサンプリング比率
}

MAP_BUILDER.use_trajectory_builder_3d = true  -- 3Dトラジェクトリビルダーを使用する
MAP_BUILDER.num_background_threads = 4  -- バックグラウンドスレッドの数
MAP_BUILDER.pose_graph = POSE_GRAPH  -- ポーズグラフの設定
POSE_GRAPH.optimize_every_n_nodes = 20  -- ノードごとに最適化を行う頻度

-- 追加設定: IMUとオドメトリの使用を有効にする
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true  -- オンライン相関スキャンマッチングを使用する
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.1  -- リアルタイム相関スキャンマッチャの線形検索ウィンドウ
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.  -- 平行移動のコスト重み
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.  -- 回転のコスト重み


TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = false  -- IMUベースの使用
TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.imu_gravity_time_constant = 10.  -- IMU重力時間定数
TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.pose_queue_duration = 0.001  -- ポーズキューの持続時間

-- TODO: これらのパラメータをサンプルデータセットで調整します。
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.pose_queue_duration = 5.  -- ポーズキューの持続時間
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.gravity_constant = 9.806  -- 重力定数
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.pose_translation_weight = 1.  -- 平行移動の重み
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.pose_rotation_weight = 1.  -- 回転の重み
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight = 1.  -- IMU加速度の重み
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_rotation_weight = 1.  -- IMU回転の重み
-- TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.odometry_translation_weight = 1.  -- オドメトリ平行移動の重み
-- TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.odometry_rotation_weight = 1.  -- オドメトリ回転の重み

TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.solver_options.use_nonmonotonic_steps = false  -- 非単調ステップの使用
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.solver_options.max_num_iterations = 10  -- 最大反復回数
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.solver_options.num_threads = 1  -- スレッド数

TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10  -- high_resolutionマップの解像度
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.  -- high_resolutionマップの最大範囲
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45  -- low_resolutionマップの解像度
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160  -- 新しいサブマップを追加する前の範囲データの数

TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55  -- hitの確率変化
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49  -- missの確率変化
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels = 2  -- フリースペースボクセルの最大数
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.intensity_threshold = INTENSITY_THRESHOLD  -- 強度閾値

TRAJECTORY_BUILDER_3D.use_intensities = false  -- 強度の使用


return options