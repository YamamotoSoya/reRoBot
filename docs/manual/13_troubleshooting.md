<!-- claude: 運用手引き 第13章 テンプレ (2026-09-22) -->

# 第13章 トラブルシューティング

---

## (GLIM) IMU-RFans間の相対位置補正
**bag取得後の編集でOK. 専用bagを撮る必要がある**
1. URDF(大きな位置変更の時のみ)
ros2_ws_main/src/bringup/rerobot_bringup/urdf/rerobot.urdf
* laser_joint  L91  : 2D LiDAR。上下反転取付なら rpy="π 0 0" のまま、正立に戻したら "0 0 0"
* rfans_joint  L115 : 3D LiDAR。xyz 実測、rpy は「前方障害物が +X に出る向き」
* imu_joint    L144 : BNO086。xyz 実測、rpy は軸対応 (現行 Rz(+90°))

(9/24 現在)

| joint | 現行 xyz [m] | 現行 rpy | rpy の決め方 |
|---|---|---|---|
| laser | (0.067, 0, 0.09406) | (π, 0, 0) | 天地逆さ取付なら roll=π。RViz で /scan の左右が反転していないか確認 |
| rfans | (0, 0, 0.80246) | (0, 0, 0) | RViz で /rfans_driver/rfans_points を見て、前方障害物が +X に出れば 0。ケーブル向きが変わると yaw=π |
| imu_link | (0, 0, 0.74196) | (0, 0, π/2) | 静止 accel z ≈ +9.8 なら正立。yaw は基板の刻印 X が車体のどちらを向くか (現行は imu_X → base_Y) |

2. GLIM の LiDAR–IMU 相対姿勢 (URDF から計算し直す派生値)
GLIMにはIMUの姿勢より3DLidarの姿勢を求めるための変換行列の値を設定するファイルがある。
* ros2_ws_glim/config/config_sensors.json       (live) の "T_lidar_imu"
* ros2_ws_glim/config/config_sensors.flat.json  (同上、同じ値)

2-1. **bagからLidar-IMU間の角度差計測**
 bag を撮る。 車体を水平な床に置き、静止 20 s 以上。これを場所を変えて 2〜3 箇所 (理由は下)。必要トピックは /rfans_driver/rfans_points・/imu/data・/odom の 3 つだけなので、通常の bringup 中の記録で足ります。

2-2. **GLIM の T_lidar_imu を再計算する**

定義は「IMU 座標の点を LiDAR 座標へ移す変換」で、URDF の 2 つの joint から

T_lidar_imu = (T_base_rfans)⁻¹ · T_base_imu
  並進 t = R_rfansᵀ · (t_imu − t_rfans)
  回転 R = R_rfansᵀ · R_imu

コンテナに入らずホストの Python で計算できます (numpy のみ):

import numpy as np
from math import cos, sin
def R(r,p,y):
    Rx=np.array([[1,0,0],[0,cos(r),-sin(r)],[0,sin(r),cos(r)]])
    Ry=np.array([[cos(p),0,sin(p)],[0,1,0],[-sin(p),0,cos(p)]])
    Rz=np.array([[cos(y),-sin(y),0],[sin(y),cos(y),0],[0,0,1]])
    return Rz@Ry@Rx                       # URDF の rpy は固定軸 X→Y→Z の順
def quat(M):                              # 回転行列 → (qx,qy,qz,qw)
    w=np.sqrt(max(0,1+M[0,0]+M[1,1]+M[2,2]))/2
    return ((M[2,1]-M[1,2])/(4*w),(M[0,2]-M[2,0])/(4*w),(M[1,0]-M[0,1])/(4*w),w)
#### ↓ URDF の値をここに写す
t_L,rpy_L = np.array([0,0,0.80246]), (0,0,0)                 # rfans_joint
t_I,rpy_I = np.array([0,0,0.74196]), (0,0,np.pi/2)           # imu_joint
RL,RI = R(*rpy_L),R(*rpy_I)
t = RL.T@(t_I-t_L); q = quat(RL.T@RI)
print("T_lidar_imu:", [*np.round(t,5), *np.round(q,10)])     # [x,y,z,qx,qy,qz,qw]

現行値で回すと [0, 0, -0.0605, 0, 0, 0.7071, 0.7071] が出るので、まずこれで自己検証してから新しい値に差し替えてください。出た 7 要素を config_sensors.json と config_sensors.flat.json の T_lidar_imu に貼り、直上のコメント (算出元の xyz/rpy) も更新します。書式は TUM 形式※1 で quaternion が最後、qw が末尾です。


← [第12章 つくチャレ当日の実行手順](12_challenge_day.md) | ↑ [目次](00_index.md)
