# reRoBot
Autonomous navigation robot for the Tsukuba Challenge

## Setup
### clone
Please add '--recursive' option when you clone this repository
```bash
$ git clone --recursive https://github.com/YamamotoSoya/reRoBot.git
```

If the symbolic link is broken, please recreate it manually
<!-- claude_opt: 旧記載の external/maxon_epos4_ros2_repo/... は実在しないパスだった -->
```bash
$ cd ~/reRoBot/src
$ ln -s external/epos4compact50-5can/maxon_epos4_ros2 .
```
### docker
```bash
$ xhost +local:docker # set host before activate docker
$ docker compose up --build # activate
$ docker exec -it rerobot_env bash # enter
$ docker compose down # deactivate
```
### colcon build
```bash
$ cd /workspace
$ rosdep update
$ rosdep install --from-paths src --ignore-src --simulate # check dependencies

$ colcon build --symlink-install --executor sequential
$ source install/setup.bash
```

## Run
<!-- claude_opt: 起動手順が README に無かったので最小限を追記。詳細は CLAUDE.md 参照 -->
```bash
# CAN interface (host or container, requires privileged)
$ sudo ip link set can0 up type can bitrate 1000000

# 2D LiDAR (HOKUYO) bringup
$ ros2 launch rerobot_bringup rerobot_bringup_2d.launch.py
# 3D LiDAR (R-Fans) bringup
$ ros2 launch rerobot_bringup rerobot_bringup_3d.launch.py

# keyboard teleop
$ ros2 run epos4_teleop teleop_keyboard --ros-args --params-file src/epos4_teleop/config/params.yaml
```

### Hardware setup
* **ubuntu22.04にcanusbを認識させる手順**:
1. can-utilsのインストール
`sudo apt-get update`
`sudo apt-get install can-utils`
2. SLcanカーネルモジュールのロードの設定ファイル作成
`sudo vim /etc/modules-load.d/can.conf`
このファイルに以下を記述
`can`
`can_raw`
`slcan`
3. USB-CANアダプタの接続と初期確認
`ls /dev/ttyACM*`または
`ls /dev/ttyUSB*`
4. CANインターフェースの作成と起動
`sudo slcand -o -c -s8 /dev/tty<<hoge>> can0`
`sudo ip link set can0 up`
