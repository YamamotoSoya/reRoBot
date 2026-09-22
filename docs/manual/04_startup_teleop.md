<!-- claude: 運用手引き 第4章 テンプレ (2026-09-22) -->

# 第4章 起動と手動操作

---
## 1. docker up
すべて
```
docker compose up 
```
対象dockerコンテナのみ (例：main, glim)
```
docker compose up main main glim
```
コンテナに入る (例：main)
```
docker exec -it rerobot_env bash
```

## 2. colcon build
```
colcon build --symlink-install
source install/setup.bash
```

## 3. rerobotlaunch
```
ros2 launch rerobot_bringup rerobot_bringup.launch.py
```

## 4. joy_teleope
```
ros2 launch rerobot_bringup joy_teleop.launch.py
```

← [第3章 基本パラメータ](03_parameters.md) | → [第5章 bag記録](05_bag_recording.md)
