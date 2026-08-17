<!-- claude: 2026-08-17 作成 -->
# tools/

コンテナ内で使う補助ツール置き場 (ROS ワークスペース外)。glim コンテナに
`/workspace/tools` として bind mount される (docker-compose.yml)。

## pointcloud_to_2dmap (submodule)

GLIM の 3D 点群地図 (PLY→PCD) を Nav2 map_server 用の 2D 占有格子 (png + yaml) に
変換する既製ツール (GLIM と同作者 koide3 製)。使い方・パラメータの意味は
`docs/features/2026-08-17_glim_map_to_nav2.md` と `docs/text/map3d_to_nav2/` を参照。

### ビルド (glim コンテナ内)

```bash
mkdir -p /workspace/tools/pointcloud_to_2dmap/build
cd /workspace/tools/pointcloud_to_2dmap/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-include boost/make_shared.hpp'
make
```

⚠️ `-include boost/make_shared.hpp` は必須。ソースが `boost::make_shared` を
ヘッダ include なしで使っており、旧 Boost では他ヘッダ経由で偶然通っていたが
Boost 1.83 (Jazzy) では未宣言エラーになる。**submodule のソースを直接パッチせず**
(pristine な上流 commit を gitlink に保つため)、コンパイラの強制 include で解決している。

依存 (`libpcl-dev`, `pcl-tools`, OpenCV, Boost) は Dockerfile_glim で導入済み。
build/ はホスト側に永続化されるが、`.gitignore` 対象 (バイナリはコミットしない)。
