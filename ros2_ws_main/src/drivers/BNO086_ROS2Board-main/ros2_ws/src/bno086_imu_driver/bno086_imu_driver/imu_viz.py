"""Browser-based live view of the ROS 2 IMU topics.

Subscribes to /imu/data, /imu/mag and /diagnostics and serves a single page
that renders the orientation in 3D, updating over Server-Sent Events.

    ros2 run bno086_imu_driver imu_viz
    # then open http://localhost:8080

RViz2 is the usual answer here, but it needs an X server, which Docker
Desktop on macOS does not provide. This node only needs a browser, and it
reads the same ROS 2 topics any other subscriber would.
"""

from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, MagneticField

ACCURACY_NAMES = ("unreliable", "low", "medium", "high")

PAGE = r"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>BNO086 IMU - ROS 2 live</title>
<style>
  :root {
    --bg: #0e1116; --panel: #171b22; --line: #262c36;
    --fg: #e6edf3; --dim: #8b949e; --accent: #4aa3ff;
    --ok: #3fb950; --warn: #d29922; --err: #f85149;
  }
  @media (prefers-color-scheme: light) {
    :root {
      --bg: #f6f8fa; --panel: #ffffff; --line: #d8dee4;
      --fg: #1f2328; --dim: #636c76; --accent: #0969da;
    }
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--fg); font: 14px/1.5
      ui-sans-serif, -apple-system, "Hiragino Sans", "Noto Sans JP", sans-serif;
    padding: 20px;
  }
  header { display: flex; align-items: baseline; gap: 14px; flex-wrap: wrap; margin-bottom: 18px; }
  h1 { font-size: 18px; margin: 0; font-weight: 650; }
  .badge {
    font: 600 11px/1 ui-monospace, monospace; padding: 5px 9px; border-radius: 999px;
    border: 1px solid var(--line); color: var(--dim);
  }
  .badge.live { color: var(--ok); border-color: var(--ok); }
  .badge.dead { color: var(--err); border-color: var(--err); }
  .grid { display: grid; grid-template-columns: minmax(280px, 420px) 1fr; gap: 16px; }
  @media (max-width: 760px) { .grid { grid-template-columns: 1fr; } }
  .panel { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 16px; }
  .panel h2 {
    font-size: 11px; letter-spacing: .04em;
    color: var(--dim); margin: 0 0 12px; font-weight: 600;
  }
  /* --- 3D orientation box --- */
  .stage {
    height: 300px; display: grid; place-items: center; perspective: 700px;
    cursor: grab; user-select: none; touch-action: none;
  }
  .stage:active { cursor: grabbing; }
  .hint { text-align: center; color: var(--dim); font-size: 11px; margin-top: 4px; }
  /* The sensor frame is Z-up right-handed. CSS is X-right, Y-DOWN,
     Z-toward-viewer, so feeding sensor axes straight into CSS puts +Z at the
     viewer and makes yaw run backwards on screen. quatToCss() rotates the
     quaternion into CSS axes instead; +Z then stands up and this is just a
     camera tilt to look down on the top face. */
  /* Camera only - driven by dragging, see the script. Which side you view
     the board from decides whether roll and yaw appear to run forwards or
     backwards (a turntable seen from behind spins the other way), so this is
     adjustable rather than guessed. */
  .view { transform-style: preserve-3d; }
  .box {
    width: 120px; height: 120px; position: relative; transform-style: preserve-3d;
    transition: transform .05s linear;
  }
  .face {
    position: absolute; inset: 0; border: 1px solid rgba(255,255,255,.45);
    display: grid; place-items: center; font: 700 15px ui-monospace, monospace;
    color: #fff; opacity: .88; text-shadow: 0 1px 3px rgba(0,0,0,.6);
  }
  /* Faces sit where the sensor axis lands after the basis change:
       +X -> CSS +X (right)   +Y -> CSS -Z (away)   +Z -> CSS -Y (up) */
  .fx  { background: #2f6feb; transform: rotateY(  90deg) translateZ(60px); }
  .fx2 { background: #1b4899; transform: rotateY( -90deg) translateZ(60px); }
  .fy  { background: #3fb950; transform: rotateY(180deg) translateZ(60px); }
  .fy2 { background: #26702f; transform: translateZ(60px); }
  .fz  { background: #f0883e; transform: rotateX(  90deg) translateZ(60px); }
  .fz2 { background: #9c5426; transform: rotateX( -90deg) translateZ(60px); }
  /* --- readouts --- */
  .rows { display: grid; gap: 10px; }
  .row { display: grid; grid-template-columns: 78px 1fr; align-items: center; gap: 10px; }
  .row span { color: var(--dim); font-size: 12px; }
  .vals { font: 13px/1.4 ui-monospace, monospace; }
  table { width: 100%; border-collapse: collapse; font: 13px ui-monospace, monospace; }
  td { padding: 5px 0; border-bottom: 1px solid var(--line); }
  td:first-child { color: var(--dim); font-family: inherit; font-size: 12px; }
  td:last-child { text-align: right; }
  tr:last-child td { border-bottom: 0; }
  .bar { height: 6px; background: var(--line); border-radius: 3px; overflow: hidden; }
  .bar i { display: block; height: 100%; background: var(--accent); width: 0; transition: width .1s; }
  .acc-0 { color: var(--err); } .acc-1 { color: var(--warn); }
  .acc-2 { color: var(--fg); }  .acc-3 { color: var(--ok); }
  footer { margin-top: 16px; color: var(--dim); font-size: 12px; }
  code { font-family: ui-monospace, monospace; background: var(--line); padding: 1px 5px; border-radius: 4px; }
</style>
</head>
<body>
<header>
  <h1>BNO086 IMU</h1>
  <span class="badge" id="conn">接続中…</span>
  <span class="badge" id="rate">-- Hz</span>
  <span class="badge" id="acc">精度 --</span>
</header>

<div class="grid">
  <div class="panel">
    <h2>姿勢 (sensor_msgs/Imu.orientation)</h2>
    <div class="stage">
     <div class="view">
      <div class="box" id="box">
        <div class="face fx">+X</div><div class="face fx2">−X</div>
        <div class="face fy">+Y</div><div class="face fy2">−Y</div>
        <div class="face fz">+Z</div><div class="face fz2">−Z</div>
      </div>
     </div>
    </div>
    <div class="hint">ドラッグで視点を回転 / ダブルクリックで初期位置</div>
    <div class="rows" id="rpy"></div>
  </div>

  <div class="panel">
    <h2>ROS 2 トピック</h2>
    <table id="tbl"></table>
  </div>
</div>

<footer>
  ROS 2 の <code>/imu/data</code> / <code>/imu/mag</code> / <code>/diagnostics</code> を購読して表示しています。
</footer>

<script>
const $ = id => document.getElementById(id);
const fmt = (v, n = 3) => (v >= 0 ? "+" : "") + v.toFixed(n);

function quatToMatrix(x, y, z, w) {
  const n = Math.hypot(x, y, z, w) || 1;
  x /= n; y /= n; z /= n; w /= n;
  const m = [
    1 - 2 * (y * y + z * z), 2 * (x * y + z * w),     2 * (x * z - y * w),     0,
    2 * (x * y - z * w),     1 - 2 * (x * x + z * z), 2 * (y * z + x * w),     0,
    2 * (x * z + y * w),     2 * (y * z - x * w),     1 - 2 * (x * x + y * y), 0,
    0, 0, 0, 1,
  ];
  return "matrix3d(" + m.join(",") + ")";
}

/* Sensor frame -> CSS frame.
 *
 * CSS is x-right, y-DOWN, z-toward-viewer. That triple is right-handed as
 * numbers, but because y runs down the screen it is LEFT-handed as seen by
 * the viewer: right x up = toward-viewer, yet CSS +X x +Y = +Z points at the
 * viewer while +Y is down. Feeding a right-handed rotation straight in
 * therefore renders a mirror image - rotations about every axis except the
 * viewing axis appear to run backwards.
 *
 * The compensating map sends sensor (x,y,z) -> CSS (x,-z,-y): +X right,
 * +Z up, +Y away from the viewer. Its determinant is -1, and conjugating a
 * rotation by a reflection negates the rotation angle, so the quaternion
 * vector part picks up an extra minus sign on top of the axis remap.
 */
function quatToCss(x, y, z, w) { return [-x, z, y, w]; }

function rpy(x, y, z, w) {
  const sinp = 2 * (w * y - z * x);
  return [
    Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180 / Math.PI,
    Math.asin(Math.max(-1, Math.min(1, sinp))) * 180 / Math.PI,
    Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180 / Math.PI,
  ];
}

function bar(label, value, max, text) {
  const pct = Math.min(100, Math.abs(value) / max * 100);
  return `<div class="row"><span>${label}</span><div>
    <div class="vals">${text}</div><div class="bar"><i style="width:${pct}%"></i></div>
  </div></div>`;
}

/* --- draggable camera ------------------------------------------------- */
const CAM0 = { yaw: -32, pitch: -24 };   // three-quarter view
let cam = { ...CAM0 };
const view = document.querySelector(".view");
function applyCam() {
  view.style.transform = `rotateX(${cam.pitch}deg) rotateY(${cam.yaw}deg)`;
}
applyCam();

const stage = document.querySelector(".stage");
let drag = null;
stage.addEventListener("pointerdown", e => {
  drag = { x: e.clientX, y: e.clientY, ...cam };
  stage.setPointerCapture(e.pointerId);
});
stage.addEventListener("pointermove", e => {
  if (!drag) return;
  cam.yaw = drag.yaw + (e.clientX - drag.x) * 0.5;
  cam.pitch = Math.max(-89, Math.min(89, drag.pitch - (e.clientY - drag.y) * 0.5));
  applyCam();
});
stage.addEventListener("pointerup", () => { drag = null; });
stage.addEventListener("dblclick", () => { cam = { ...CAM0 }; applyCam(); });

const es = new EventSource("/events");
es.onopen = () => { $("conn").textContent = "接続済み"; $("conn").className = "badge live"; };
es.onerror = () => { $("conn").textContent = "切断"; $("conn").className = "badge dead"; };

es.onmessage = ev => {
  const d = JSON.parse(ev.data);
  if (!d.have_imu) { $("conn").textContent = "データ待ち"; return; }

  const q = d.orientation;
  $("box").style.transform = quatToMatrix(...quatToCss(q[0], q[1], q[2], q[3]));

  const [r, p, yw] = rpy(q[0], q[1], q[2], q[3]);
  const a = d.linear_acceleration, g = d.angular_velocity;
  const amag = Math.hypot(a[0], a[1], a[2]), gmag = Math.hypot(g[0], g[1], g[2]);

  $("rpy").innerHTML =
    bar("roll",  r,  180, `${fmt(r, 2)}°`) +
    bar("pitch", p,   90, `${fmt(p, 2)}°`) +
    bar("yaw",   yw, 180, `${fmt(yw, 2)}°`);

  $("rate").textContent = d.rate.toFixed(1) + " Hz";
  $("acc").textContent = "精度 " + d.accuracy_name;
  $("acc").className = "badge acc-" + d.accuracy;

  const mag = d.magnetic_field;
  $("tbl").innerHTML = `
    <tr><td>orientation (x,y,z,w)</td><td>${q.map(v => fmt(v, 4)).join("  ")}</td></tr>
    <tr><td>roll / pitch / yaw</td><td>${fmt(r,1)}°  ${fmt(p,1)}°  ${fmt(yw,1)}°</td></tr>
    <tr><td>angular_velocity [rad/s]</td><td>${g.map(v => fmt(v, 3)).join("  ")}</td></tr>
    <tr><td>|ω|</td><td>${gmag.toFixed(3)} rad/s</td></tr>
    <tr><td>linear_acceleration [m/s²]</td><td>${a.map(v => fmt(v, 2)).join("  ")}</td></tr>
    <tr><td>|a|</td><td>${amag.toFixed(3)} m/s²  (重力 9.807)</td></tr>
    <tr><td>magnetic_field [µT]</td><td>${mag ? mag.map(v => fmt(v, 1)).join("  ") : "-"}</td></tr>
    <tr><td>/imu/data 受信数</td><td>${d.imu_count}</td></tr>
    <tr><td>/imu/mag 受信数</td><td>${d.mag_count}</td></tr>
    <tr><td>frame_id</td><td>${d.frame_id}</td></tr>
    <tr><td>診断</td><td>${d.diagnostic}</td></tr>`;
};
</script>
</body>
</html>
"""


class VizNode(Node):
    def __init__(self) -> None:
        super().__init__("bno086_imu_viz")
        self.declare_parameter("http_port", 8080)

        self._lock = threading.Lock()
        self._state = {
            "have_imu": False,
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "angular_velocity": [0.0, 0.0, 0.0],
            "linear_acceleration": [0.0, 0.0, 0.0],
            "magnetic_field": None,
            "accuracy": 0,
            "accuracy_name": "unreliable",
            "frame_id": "-",
            "imu_count": 0,
            "mag_count": 0,
            "rate": 0.0,
            "diagnostic": "-",
        }
        self._last_count = 0
        self._last_tick = time.monotonic()

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Imu, "imu/data", self._on_imu, qos)
        self.create_subscription(MagneticField, "imu/mag", self._on_mag, qos)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.create_timer(0.5, self._tick)

    def _on_imu(self, msg: Imu) -> None:
        o = msg.orientation
        with self._lock:
            s = self._state
            s["have_imu"] = True
            s["orientation"] = [o.x, o.y, o.z, o.w]
            s["angular_velocity"] = [msg.angular_velocity.x,
                                     msg.angular_velocity.y,
                                     msg.angular_velocity.z]
            s["linear_acceleration"] = [msg.linear_acceleration.x,
                                        msg.linear_acceleration.y,
                                        msg.linear_acceleration.z]
            s["frame_id"] = msg.header.frame_id
            s["imu_count"] += 1
            # The driver encodes the sensor's own confidence as the isotropic
            # orientation covariance; -1 means it declined to supply one.
            var = msg.orientation_covariance[0]
            if var < 0:
                s["accuracy"] = 0
            else:
                sd = math.sqrt(var)
                s["accuracy"] = 3 if sd < 0.05 else 2 if sd < 0.2 else 1 if sd < 0.8 else 0
            s["accuracy_name"] = ACCURACY_NAMES[s["accuracy"]]

    def _on_mag(self, msg: MagneticField) -> None:
        with self._lock:
            self._state["magnetic_field"] = [msg.magnetic_field.x * 1e6,
                                             msg.magnetic_field.y * 1e6,
                                             msg.magnetic_field.z * 1e6]
            self._state["mag_count"] += 1

    def _on_diag(self, msg: DiagnosticArray) -> None:
        for st in msg.status:
            if st.name.startswith("bno086"):
                with self._lock:
                    self._state["diagnostic"] = st.message
                return

    def _tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            dt = now - self._last_tick
            if dt > 0:
                self._state["rate"] = (self._state["imu_count"] - self._last_count) / dt
            self._last_count = self._state["imu_count"]
        self._last_tick = now

    def snapshot(self) -> str:
        with self._lock:
            return json.dumps(self._state)


def make_handler(node: VizNode):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args) -> None:
            pass  # keep the ROS console readable

        def do_GET(self) -> None:
            if self.path.startswith("/events"):
                self._stream()
            elif self.path in ("/", "/index.html"):
                body = PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    payload = node.snapshot()
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(1.0 / 30.0)   # 30 fps is plenty for a display
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VizNode()
    port = node.get_parameter("http_port").value

    server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(node))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"visualiser on http://localhost:{port}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
