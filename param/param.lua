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

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- [double] / 追加される制約の割合がこの数値を下回ると制約が追加されます。
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.  -- [double] / サブマップに近いと見なされるポーズの閾値。
POSE_GRAPH.constraint_builder.min_score = 0.55  -- [double] / スキャンマッチスコアがこの値を下回ると一致と見なされません。低いスコアはスキャンとマップが似ていないことを示します。
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6  -- [double] / グローバルローカライゼーションが信頼されない閾値。
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4  -- [double] / ループクロージャ制約の平行移動成分の最適化問題で使用される重み。
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5  -- [double] / ループクロージャ制約の回転成分の最適化問題で使用される重み。
POSE_GRAPH.constraint_builder.log_matches = true  -- [bool] / 有効にすると、デバッグのためにループクロージング制約の情報がログに記録されます。

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.  -- [double] / 最良のスキャンアライメントが見つかる最小線形検索ウィンドウ。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)  -- [double] / 最良のスキャンアライメントが見つかる最小角度検索ウィンドウ。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7  -- [int32] / 使用する事前計算グリッドの数。

POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.  -- [double] / 各コストファンクタのスケーリングパラメータ。
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.  -- [double] / 未記載。
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.  -- [double] / 未記載。

POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true  -- [bool] / Ceresソルバーを設定します。詳細はCeresドキュメントを参照してください: https://code.google.com/p/ceres-solver/
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10  -- [int32] / 未記載。
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1  -- [int32] / 未記載。

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 8  -- [int32] / 使用する事前計算グリッドの数。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth = 3  -- [int32] / 使用するフル解像度グリッドの数。追加のグリッドは解像度を半分に減らします。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.77  -- [double] / 回転スキャンマッチャの最小スコア。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.55  -- [double] / 低解像度グリッドのスコアがこの値を下回ると一致と見なされません。3Dでのみ使用されます。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5.  -- [double] / 最良のスキャンアライメントが見つかる重力に直交する平面内の線形検索ウィンドウ。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.  -- [double] / 最良のスキャンアライメントが見つかる重力方向の線形検索ウィンドウ。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(15.)  -- [double] / 最良のスキャンアライメントが見つかる最小角度検索ウィンドウ。

POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_0 = 5.  -- [double] / 各コストファンクタのスケーリングパラメータ。
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_1 = 30.  -- [double] / 各コストファンクタのスケーリングパラメータ。
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 10.  -- [double] / 未記載。
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 1.  -- [double] / 未記載。
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = false  -- [bool] / ロール/ピッチを固定し、ヨーのみを変更するかどうか。

POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.use_nonmonotonic_steps = false  -- [bool] / Ceresソルバーを設定します。詳細はCeresドキュメントを参照してください: https://code.google.com/p/ceres-solver/
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 10  -- [int32] / 未記載。
POSE_GRAPH.constraint_builder.ceres_solver_options.num_threads = 1  -- [int32] / 未記載。

POSE_GRAPH.matcher_translation_weight = 5e2  -- [double] / ループクロージャ以外のスキャンマッチャ制約の平行移動成分の最適化問題で使用される重み。
POSE_GRAPH.matcher_rotation_weight = 1.6e3  -- [double] / ループクロージャ以外のスキャンマッチャ制約の回転成分の最適化問題で使用される重み。

POSE_GRAPH.optimization_problem.huber_scale = 1e1  -- [double] / Huber損失関数のスケーリングパラメータ。
POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2  -- [double] / IMU加速度項のスケーリングパラメータ。
POSE_GRAPH.optimization_problem.rotation_weight = 1.6e4  -- [double] / IMU回転項のスケーリングパラメータ。
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5  -- [double] / ローカルSLAMポーズに基づく連続ノード間の平行移動のスケーリングパラメータ。
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5  -- [double] / ローカルSLAMポーズに基づく連続ノード間の回転のスケーリングパラメータ。
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5  -- [double] / オドメトリに基づく連続ノード間の平行移動のスケーリングパラメータ。
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5  -- [double] / オドメトリに基づく連続ノード間の回転のスケーリングパラメータ。
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1  -- [double] / 固定フレームポーズの平行移動のスケーリングパラメータ。
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2  -- [double] / 固定フレームポーズの回転のスケーリングパラメータ。
POSE_GRAPH.optimization_problem.log_solver_summary = false  -- [bool] / 有効にすると、最適化ごとにCeresソルバーのサマリーがログに記録されます。
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false  -- [bool] / Ceresソルバーを設定します。詳細はCeresドキュメントを参照してください: https://code.google.com/p/ceres-solver/
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50  -- [int32] / 未記載。
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 7  -- [int32] / 未記載。

POSE_GRAPH.max_num_final_iterations = 200  -- [int32] / 最終最適化のために使用する反復回数。
POSE_GRAPH.global_sampling_ratio = 0.003  -- [double] / グローバルローカライゼーションのために単一のトラジェクトリのノードをサンプリングする割合。
POSE_GRAPH.log_residual_histograms = true  -- [bool] / ポーズ残差のヒストグラムを出力するかどうか。

POSE_GRAPH.global_constraint_search_after_n_seconds = 10.  -- [double] / 2つのトラジェクトリ間で指定された期間グローバル制約が追加されない場合、ループクロージャ検索は小さな検索ウィンドウではなくグローバルに実行されます。

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