# reRoBot
Autonomous navigation robot for the Tsukuba Challenge

## Setup
### clone
Please add '--recursive' option when you clone this repository
```bash
$ git clone --recursive https://github.com/YamamotoSoya/reRoBot.git
```

If the symbolic link is broken, please recreate it manually
```bash
$ cd ~/reRoBot/src
$ ln -s external/maxon_epos4_ros2_repo/maxon_epos4_ros2 .
```
### docker
```bash
$ xhost +local:docker # set host before activate docker
$ docker compose up --build # activate
$ docker exec -it rerobot_env bash # enter
$ docker copose down # deactivate
```
### colcon build
```bash
$ cd /workspace
$ rosdep update
$ rosdep install --from-paths src --ignore-src --simulate # check dependencies

$ colcon build --symlink-install --executor sequential
$ source install/setup.bash
```
### Hardwere setup
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