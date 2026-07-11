// claude: R-Fans (/sdk_could) → LIO-SAM 互換 (/points) 点群変換ノード。
//
// LIO-SAM は入力 PointCloud2 に "ring"(uint16) と "time"(float32, スキャン先頭
// からの相対秒) フィールドを要求する。R-Fans (rfans_driver) の出力は
// "laserid"(int32) / "timeflag"(float32, 絶対秒) で名前も型も異なるため、
// フィールド名ベースで読み替えて再発行する。
//
// LIO-SAM velodyne モードの ring 逆算 (縦角の等間隔仮定) は R-Fans-16 GM の
// 不等間隔縦角では誤分類するため、実機の laserid をそのまま ring に使うのが正。
//
// QoS: rfans_driver は reliable で publish、LIO-SAM は best-effort で subscribe。
//      本ノードは reliable(10) で受けて reliable(10) で出す (reliable pub →
//      best-effort sub は DDS 互換)。
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class RfansRingConverterNode : public rclcpp::Node
{
public:
  RfansRingConverterNode()
  : Node("rfans_ring_converter")
  {
    declare_parameter("input_topic", std::string("/sdk_could"));
    declare_parameter("output_topic", std::string("/points"));
    // claude: R-Fans-16 のチャネル数。laserid がこの範囲外の点は捨てる
    declare_parameter("n_scan", 16);

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    n_scan_ = static_cast<int>(get_parameter("n_scan").as_int());

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::QoS(10));
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { convert(*msg); });

    RCLCPP_INFO(
      get_logger(), "rfans_ring_converter: %s -> %s (n_scan=%d)",
      input_topic.c_str(), output_topic.c_str(), n_scan_);
  }

private:
  static bool has_field(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
  {
    for (const auto & f : msg.fields) {
      if (f.name == name) {return true;}
    }
    return false;
  }

  void convert(const sensor_msgs::msg::PointCloud2 & in)
  {
    // claude: 必須フィールドが無い入力 (別センサ・再生データ等) は変換不能。
    // 毎周期 ERROR を吐かないよう 5 秒スロットルで知らせて捨てる。
    if (!has_field(in, "x") || !has_field(in, "laserid") || !has_field(in, "timeflag")) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "input cloud lacks x/laserid/timeflag fields; is this really rfans_driver output?");
      return;
    }
    const bool has_intensity = has_field(in, "intensity");

    // 1 パス目: スキャン先頭時刻 (有効点の timeflag 最小値) を求める。
    // rfans の点順は時刻昇順とは限らない前提で min を取る。
    float t0 = std::numeric_limits<float>::max();
    size_t valid = 0;
    {
      sensor_msgs::PointCloud2ConstIterator<float> it_x(in, "x"), it_y(in, "y"), it_z(in, "z");
      sensor_msgs::PointCloud2ConstIterator<float> it_t(in, "timeflag");
      sensor_msgs::PointCloud2ConstIterator<int32_t> it_l(in, "laserid");
      for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_t, ++it_l) {
        if (!point_ok(*it_x, *it_y, *it_z, *it_l)) {continue;}
        ++valid;
        if (*it_t < t0) {t0 = *it_t;}
      }
    }
    if (valid == 0) {return;}  // 全点無効 (起動直後等) は黙って捨てる
    if (!std::isfinite(t0)) {t0 = 0.0f;}

    sensor_msgs::msg::PointCloud2 out;
    out.header = in.header;
    sensor_msgs::PointCloud2Modifier mod(out);
    // claude: LIO-SAM の VelodynePointXYZIRT と同名・同型のフィールド構成。
    // pcl::fromROSMsg は名前でマッチするため offset の詰め方は任意。
    mod.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(valid);
    out.height = 1;
    out.width = static_cast<uint32_t>(valid);
    out.is_dense = true;

    // 2 パス目: 転記。
    sensor_msgs::PointCloud2ConstIterator<float> it_x(in, "x"), it_y(in, "y"), it_z(in, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_t(in, "timeflag");
    sensor_msgs::PointCloud2ConstIterator<int32_t> it_l(in, "laserid");
    // intensity は rfans 側に無いケースに備えて別扱い (実機は "intensity" あり)
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_i;
    if (has_intensity) {
      it_i = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(in, "intensity");
    }

    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
    sensor_msgs::PointCloud2Iterator<float> oi(out, "intensity"), ot(out, "time");
    sensor_msgs::PointCloud2Iterator<uint16_t> oring(out, "ring");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_t, ++it_l) {
      const float intensity = it_i ? *(*it_i) : 0.0f;
      if (it_i) {++(*it_i);}
      if (!point_ok(*it_x, *it_y, *it_z, *it_l)) {continue;}
      *ox = *it_x;
      *oy = *it_y;
      *oz = *it_z;
      *oi = intensity;
      *oring = static_cast<uint16_t>(*it_l);
      // claude: timeflag は絶対秒 → スキャン先頭からの相対秒に変換 (deskew 用)
      const float rel = *it_t - t0;
      *ot = std::isfinite(rel) && rel >= 0.0f ? rel : 0.0f;
      ++ox; ++oy; ++oz; ++oi; ++oring; ++ot;
    }

    pub_->publish(out);
  }

  bool point_ok(float x, float y, float z, int32_t laserid) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {return false;}
    if (x == 0.0f && y == 0.0f && z == 0.0f) {return false;}  // 無効点 (no return)
    if (laserid < 0 || laserid >= n_scan_) {return false;}
    return true;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  int n_scan_ = 16;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RfansRingConverterNode>());
  rclcpp::shutdown();
  return 0;
}
