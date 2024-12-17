# SLAM with Unitree LiDAR L1, Google Cartographer, ROS2
L1, L2 LiDARではROS 2のLIO等で自己位置推定やマップ生成ができませんでした  
このリポジトリはCartographerを使って3D SLAMをするものです  
パラメーターの調整等はできていませんが、ゆっくり動かせば問題なく動作します  

# Build
略(git clone colcon build)

# 実行
## 一括起動
```bash
ros2 launch l1_slam all.launch.xml
```

## 1. LiDAR起動
```bash
ros2 lunahc l1_slam l1.launch.py
```

## 2. TF発行
```bash
ros2 launch l1_slam tf.launch.xml
```

## 3. Cartographer起動
```bash
ros2 launch l1_slam cartographer.launch.py
```

## 4.rviz2起動
```bash
ros2 launch l1_slam rviz2.launch.xml
```
