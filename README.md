# rabcl_ros2
[![CI](https://github.com/rabbits-robotics/rabcl_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/rabbits-robotics/rabcl_ros2/actions/workflows/ci.yml)

Rabbits Common Library の ROS 2 向けラッパー。ROS 2 のメッセージと [rabcl](https://github.com/rabbits-robotics/rabcl) の共通プロトコル（`rabcl::protocol`）を相互に変換する。
rabcl 自体は ROS 2 に依存しないので、ROS 2 固有の変換だけをここに置く。機体の情報（関節名の並び）は持たず、呼び出し側（bridge の YAML）が渡す。

## 構成

| ファイル | 内容 |
|---|---|
| `include/rabcl_ros2/convert.hpp` | `Convert` クラス。`JointStatesToMsg`（`JOINT_STATE` → `sensor_msgs/JointState`、関節番号 → 名前）、`ImuToMsg`（`IMU` → `sensor_msgs/Imu`）、`ReferenceIsConsistent` / `ReferenceToChassisVel` / `ReferenceToJointCmds` / `ReferenceToModes`（`rabbits_msgs/ReferenceCommand` → 下りメッセージ、名前 → 番号） |
| `include/rabcl_ros2/utils.hpp` | `Utils` クラス。beta 用の旧変換（`AttractsCommand` ⇔ `rabcl::Info`）。beta を共通プロトコルに移行したあと削除する |
| `test/` | gtest |

依存: `rabcl`、`rabbits_msgs`、`sensor_msgs`。

## 使い方

`rabbits_ws` の `機体名.repos` に rabcl と一緒に追加する。パッケージからは `package.xml` と `CMakeLists.txt` で `rabcl_ros2` に依存し、`#include <rabcl_ros2/convert.hpp>` で使う（[rabbits_bridge](https://github.com/rabbits-robotics/rabbits_bridge) が例）。

同じ位置づけのライブラリとして、ESP32 ファームウェア向けの [rabcl_esp32](https://github.com/rabbits-robotics/rabcl_esp32) がある（こちらは rabcl に依存せず、ESP-IDF のドライバを包む）。

## ビルド・テスト

```bash
colcon build --packages-select rabcl_ros2
colcon test --packages-select rabcl_ros2
colcon test-result --verbose
```

テストは `test/` に gtest で置いている。rabcl 系のリポジトリはユニットテストを必ず書き、CI で実行する（handbook の [common-lib.md](https://github.com/rabbits-robotics/.github/blob/main/docs/common-lib.md)）。

## ライセンス

MIT。[LICENSE](./LICENSE) を参照。
