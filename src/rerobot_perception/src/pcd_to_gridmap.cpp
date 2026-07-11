// claude: LIO-SAM が保存した PCD 地図を Nav2 (map_server) 用の 2D 占有格子
// (pgm + yaml) に投影するオフライン CLI ツール。ROS ノードではない。
//
//   使い方:
//     ros2 run rerobot_perception pcd_to_gridmap <in.pcd> <out_prefix>
//         [--resolution 0.05] [--obstacle-zmin 0.2] [--obstacle-zmax 1.5]
//         [--ground-zmin -0.3] [--ground-zmax 0.1] [--min-points 2]
//   出力: <out_prefix>.pgm / <out_prefix>.yaml
//
// 判定則 (セル単位):
//   - 障害物帯 (obstacle-zmin..zmax) に min-points 個以上 → 占有 (0, 黒)
//   - 占有でなく、地面帯 (ground-zmin..zmax) に点がある → 自由 (254, 白)
//   - どちらでもない → 未知 (205, 灰)
// z の基準は LIO-SAM の map フレーム原点 (= 開始時の lidar 位置) なので、
// 地面帯の既定値はロボットの LiDAR 高 0.714 m を想定した -0.9..-0.5 ではなく、
// まず PCD を CloudCompare 等で眺めて z 帯を決め直すこと (下記は base_link
// 原点相当を仮定した仮値)。生成物は GIMP で手直しして使う下絵という位置付け。
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace
{

struct Options
{
  std::string input;
  std::string out_prefix;
  double resolution = 0.05;      // [m/cell]
  double obstacle_zmin = 0.2;    // [m] この帯の点を障害物とみなす
  double obstacle_zmax = 1.5;
  double ground_zmin = -0.3;     // [m] この帯の点があれば自由空間の根拠
  double ground_zmax = 0.1;
  int min_points = 2;            // 占有判定に必要な障害物点数 (ノイズ除け)
};

bool parse_args(int argc, char ** argv, Options & opt)
{
  if (argc < 3) {return false;}
  opt.input = argv[1];
  opt.out_prefix = argv[2];
  for (int i = 3; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    const std::string val = argv[i + 1];
    if (key == "--resolution") {opt.resolution = std::stod(val);} else
    if (key == "--obstacle-zmin") {opt.obstacle_zmin = std::stod(val);} else
    if (key == "--obstacle-zmax") {opt.obstacle_zmax = std::stod(val);} else
    if (key == "--ground-zmin") {opt.ground_zmin = std::stod(val);} else
    if (key == "--ground-zmax") {opt.ground_zmax = std::stod(val);} else
    if (key == "--min-points") {opt.min_points = std::stoi(val);} else {
      std::cerr << "unknown option: " << key << "\n";
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  if (!parse_args(argc, argv, opt)) {
    std::cerr <<
      "usage: pcd_to_gridmap <in.pcd> <out_prefix>"
      " [--resolution 0.05] [--obstacle-zmin 0.2] [--obstacle-zmax 1.5]"
      " [--ground-zmin -0.3] [--ground-zmax 0.1] [--min-points 2]\n";
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(opt.input, cloud) < 0) {
    std::cerr << "failed to load " << opt.input << "\n";
    return 1;
  }
  if (cloud.empty()) {
    std::cerr << "empty cloud: " << opt.input << "\n";
    return 1;
  }

  // 対象帯 (障害物帯 ∪ 地面帯) の xy 範囲から格子サイズを決める
  double min_x = 1e18, min_y = 1e18, max_x = -1e18, max_y = -1e18;
  size_t in_band = 0;
  for (const auto & p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    const bool obstacle = p.z >= opt.obstacle_zmin && p.z <= opt.obstacle_zmax;
    const bool ground = p.z >= opt.ground_zmin && p.z <= opt.ground_zmax;
    if (!obstacle && !ground) {continue;}
    ++in_band;
    min_x = std::min(min_x, static_cast<double>(p.x));
    max_x = std::max(max_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_y = std::max(max_y, static_cast<double>(p.y));
  }
  if (in_band == 0) {
    std::cerr << "no points in obstacle/ground z-bands; check --obstacle-z*/--ground-z*\n";
    return 1;
  }

  // 端の点がセル境界に乗らないよう 1 セル分の余白を取る
  min_x -= opt.resolution;
  min_y -= opt.resolution;
  max_x += opt.resolution;
  max_y += opt.resolution;
  const int width = static_cast<int>((max_x - min_x) / opt.resolution) + 1;
  const int height = static_cast<int>((max_y - min_y) / opt.resolution) + 1;
  if (width <= 0 || height <= 0 || static_cast<long long>(width) * height > 200000000LL) {
    std::cerr << "grid too large: " << width << "x" << height << "\n";
    return 1;
  }

  std::vector<uint16_t> obstacle_count(static_cast<size_t>(width) * height, 0);
  std::vector<uint8_t> ground_hit(static_cast<size_t>(width) * height, 0);
  for (const auto & p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    const int cx = static_cast<int>((p.x - min_x) / opt.resolution);
    const int cy = static_cast<int>((p.y - min_y) / opt.resolution);
    if (cx < 0 || cx >= width || cy < 0 || cy >= height) {continue;}
    const size_t idx = static_cast<size_t>(cy) * width + cx;
    if (p.z >= opt.obstacle_zmin && p.z <= opt.obstacle_zmax) {
      if (obstacle_count[idx] < UINT16_MAX) {++obstacle_count[idx];}
    } else if (p.z >= opt.ground_zmin && p.z <= opt.ground_zmax) {
      ground_hit[idx] = 1;
    }
  }

  // pgm (P5)。map_server の既定解釈: 0=占有, 254=自由, 205=未知。
  // 画像の行 0 は「上」= y 最大側なので y を反転して書く。
  const std::string pgm_path = opt.out_prefix + ".pgm";
  std::ofstream pgm(pgm_path, std::ios::binary);
  if (!pgm) {
    std::cerr << "cannot write " << pgm_path << "\n";
    return 1;
  }
  pgm << "P5\n" << width << " " << height << "\n255\n";
  size_t occupied = 0, free_cells = 0;
  for (int row = 0; row < height; ++row) {
    const int cy = height - 1 - row;
    for (int cx = 0; cx < width; ++cx) {
      const size_t idx = static_cast<size_t>(cy) * width + cx;
      uint8_t v = 205;  // unknown
      if (obstacle_count[idx] >= opt.min_points) {
        v = 0;
        ++occupied;
      } else if (ground_hit[idx]) {
        v = 254;
        ++free_cells;
      }
      pgm.put(static_cast<char>(v));
    }
  }
  pgm.close();

  // yaml (map_server 用)。origin は格子左下セルの map 座標。
  const std::string yaml_path = opt.out_prefix + ".yaml";
  std::ofstream yaml(yaml_path);
  if (!yaml) {
    std::cerr << "cannot write " << yaml_path << "\n";
    return 1;
  }
  // image は yaml と同じディレクトリ想定の相対パスにする
  std::string image_name = pgm_path;
  const auto slash = image_name.find_last_of('/');
  if (slash != std::string::npos) {image_name = image_name.substr(slash + 1);}
  yaml << "image: " << image_name << "\n"
       << "resolution: " << opt.resolution << "\n"
       << "origin: [" << min_x << ", " << min_y << ", 0.0]\n"
       << "negate: 0\n"
       << "occupied_thresh: 0.65\n"
       << "free_thresh: 0.196\n";
  yaml.close();

  std::cout << "wrote " << pgm_path << " (" << width << "x" << height
            << " @ " << opt.resolution << " m/cell), " << yaml_path << "\n"
            << "cells: occupied=" << occupied << " free=" << free_cells
            << " unknown=" << (static_cast<size_t>(width) * height - occupied - free_cells)
            << "\n";
  return 0;
}
