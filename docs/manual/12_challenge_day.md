<!-- claude: 運用手引き 第12章 テンプレ (2026-09-22) -->

# 第12章 つくチャレ当日の実行手順

---

## 1. docker起動->launch
* 本体電源を入れる。
```
xhost +local:docker
docker compose up glim main
docker exec -it ~~~~~_env bash

ros2 launch rerobot_bringup rerobot_bringup.launch.py
ros2 launch rerobot_bringup wt901_imu.launch.py
ros2 launch rerobot_bringup joy_releop.launch.py
``` 
topicの確認
```
ros2 topic list
以下 topicの存在を確認

```

## 2. bagの記録
<<<リンク　05>>>
```

```

## 3. 3D SLAM

## 4. 2D 圧縮

## 5. map加工

## 6. Nav2,slamtoolbox反映

## 7. 自律移動

← [第11章 amcl](11_amcl.md) | → [第13章 トラブルシューティング](13_troubleshooting.md)
