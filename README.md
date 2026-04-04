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
$ docker exec -it reRoBot_env bash # enter
$ docker copose down # deactivate
```
### colcon build
```bash
$ cd /workspace
$ rosdep update
$ rosdep rosdep install --from-paths src --ignore-src --simulate # check dependencies

$ colcon build --symlink-install --executor sequential
$ source install/setup.bash
```