# claude: 2026-10-01 新設。
# 3D 点群 (R-Fans) を LaserScan に落とすとき、同じ方位ビンに入った複数リングの点から
# 「どれを代表にするか」を切り替える実験用ノード。pointcloud_to_laserscan は最近点固定で
# 選び方を変えられないため自作した。`all` (全点) はビン幅を R-Fans の方位刻みまで細かくした
# pointcloud_to_laserscan で実現できるので、ここでは持たない。
#
#   mode = nearest  : ビン内の最短距離 (pointcloud_to_laserscan と同じ。実装一致確認の基準)
#   mode = farthest : ビン内の最遠距離 (手前の植え込みでなく奥の壁を見る)
#   mode = nth      : 近い方から nth_k 番目 (0 始まり。k=1 で 2 番目に近い点)
#
# 帯 [min_height, max_height] は target_frame (base_link) の z で切る。rfans→base_link は
# 静的 TF なので初回取得後はキャッシュする。空ビンは inf (use_inf 相当)。
import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2

# PointField.datatype → numpy dtype
_DT = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}


def _quat_to_mat(x, y, z, w):
    """クォータニオン → 3x3 回転行列 (scipy 非依存)。"""
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class ScanModes(Node):
    def __init__(self):
        super().__init__("rfans_scan_modes")
        self.declare_parameters(
            "",
            [
                ("cloud_topic", "/rfans_driver/rfans_points"),
                ("scan_topic", "/scan_modes"),
                ("target_frame", "base_link"),
                ("mode", "nearest"),
                ("nth_k", 1),
                ("min_height", 0.3),
                ("max_height", 1.5),
                ("angle_increment", 0.0035),
                ("range_min", 0.5),
                ("range_max", 30.0),
                ("use_laserids", [-1]),  # [-1] = 全リング
                ("log_every_frames", 100),
            ],
        )
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.mode = str(g("mode"))
        if self.mode not in ("nearest", "farthest", "nth"):
            raise ValueError(f"unknown mode: {self.mode}")
        self.k = int(g("nth_k"))
        self.zmin, self.zmax = float(g("min_height")), float(g("max_height"))
        self.inc = float(g("angle_increment"))
        self.rmin, self.rmax = float(g("range_min")), float(g("range_max"))
        self.frame = str(g("target_frame"))
        ids = [int(v) for v in g("use_laserids")]
        self.ids = None if ids == [-1] else np.array(ids)
        self.log_every = int(g("log_every_frames"))
        # pointcloud_to_laserscan と同じ配列長・添字にする (ranges = ceil((max-min)/inc)、idx = (θ-min)/inc)
        self.amin, amax = -3.14159265, 3.14159265
        self.nb = int(math.ceil((amax - self.amin) / self.inc))
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self.rot = None
        self.trans = None
        self.dtype = None
        self.frames = 0
        self.filled = 0
        self.pub = self.create_publisher(LaserScan, str(g("scan_topic")), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, str(g("cloud_topic")), self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"mode={self.mode} k={self.k} band=[{self.zmin},{self.zmax}] inc={self.inc} "
            f"bins={self.nb} → {g('scan_topic')}"
        )

    def _lookup(self, cloud_frame):
        if self.rot is not None:
            return
        tr = self.tfbuf.lookup_transform(
            self.frame, cloud_frame, Time(), timeout=Duration(seconds=0.2)
        ).transform
        q, v = tr.rotation, tr.translation
        self.rot = _quat_to_mat(q.x, q.y, q.z, q.w)
        self.trans = np.array([v.x, v.y, v.z])
        self.get_logger().info(f"TF {cloud_frame}→{self.frame} cached: t={self.trans.round(3).tolist()}")

    def cb(self, msg: PointCloud2):
        try:
            self._lookup(msg.header.frame_id)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"TF not available yet: {e}", throttle_duration_sec=5.0)
            return
        if self.dtype is None:
            self.dtype = np.dtype(
                {
                    "names": [f.name for f in msg.fields],
                    "formats": [_DT[f.datatype] for f in msg.fields],
                    "offsets": [f.offset for f in msg.fields],
                    "itemsize": msg.point_step,
                }
            )
        a = np.frombuffer(bytes(msg.data), dtype=self.dtype)
        pts = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float64)
        ok = np.isfinite(pts).all(axis=1)
        if self.ids is not None and "laserid" in a.dtype.names:
            ok &= np.isin(a["laserid"], self.ids)
        pts = pts[ok] @ self.rot.T + self.trans  # → target_frame
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        r = np.hypot(x, y)
        sel = (z >= self.zmin) & (z <= self.zmax) & (r >= self.rmin) & (r <= self.rmax)
        r = r[sel]
        b = np.clip(((np.arctan2(y[sel], x[sel]) - self.amin) / self.inc).astype(np.int64), 0, self.nb - 1)

        nb = self.nb
        if self.mode == "nearest":
            out = np.full(nb, np.inf)
            np.minimum.at(out, b, r)
        elif self.mode == "farthest":
            out = np.zeros(nb)
            np.maximum.at(out, b, r)
            out[out == 0.0] = np.inf
        else:  # nth
            order = np.lexsort((r, b))  # ビン → 距離 の順
            bs, rs = b[order], r[order]
            first = np.searchsorted(bs, np.arange(nb))
            cnt = np.bincount(bs, minlength=nb)
            out = np.full(nb, np.inf)
            has = cnt > self.k
            out[has] = rs[first[has] + self.k]

        s = LaserScan()
        s.header.stamp = msg.header.stamp
        s.header.frame_id = self.frame
        s.angle_min = self.amin
        s.angle_increment = self.inc
        s.angle_max = self.amin + self.inc * (nb - 1)
        s.scan_time = 0.1
        s.time_increment = 0.0
        s.range_min = float(self.rmin)
        s.range_max = float(self.rmax)
        s.ranges = out.astype(np.float32).tolist()
        self.pub.publish(s)

        self.frames += 1
        self.filled += int(np.isfinite(out).sum())
        if self.log_every > 0 and self.frames % self.log_every == 0:
            self.get_logger().info(
                f"{self.mode}: bin fill {100.0 * self.filled / (self.frames * nb):.1f}% "
                f"(points in band {sel.sum()}/{len(sel)})"
            )
            self.frames = 0
            self.filled = 0


def main():
    rclpy.init()
    node = ScanModes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
