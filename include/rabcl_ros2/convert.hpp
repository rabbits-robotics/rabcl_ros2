// rabcl::protocol（PC ⇔ ファームの共通プロトコル）の構造体と ROS 2 メッセージの相互変換。
// 機体の情報は持たない。関節名・モード名の並び（YAML）は呼び出し側（bridge）が渡す。
#ifndef RABCL_ROS2__CONVERT_HPP_
#define RABCL_ROS2__CONVERT_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <rabbits_msgs/msg/reference_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "rabcl/protocol/protocol.hpp"

namespace rabcl_ros2
{
class Convert
{
public:
  // JOINT_STATE → JointState。id を names の添字として名前を引く。
  // names に無い id は読み飛ばし、その場合 false を返す（詰めた分は msg に入る）
  static bool JointStatesToMsg(
    const rabcl::protocol::JointState * joints, uint8_t count,
    const std::vector<std::string> & names, sensor_msgs::msg::JointState & msg);

  // IMU → Imu。orientation はオイラー角（heading, roll, pitch）からクォータニオン
  static void ImuToMsg(const rabcl::protocol::Imu & imu, sensor_msgs::msg::Imu & msg);

  // joint_* の 6 配列が同じ長さ、mode_* の 2 配列が同じ長さなら true
  static bool ReferenceIsConsistent(const rabbits_msgs::msg::ReferenceCommand & msg);

  static void ReferenceToChassisVel(
    const rabbits_msgs::msg::ReferenceCommand & msg, rabcl::protocol::ChassisVel & vel);

  // body_height / body_roll / body_pitch → BodyPose
  static void ReferenceToBodyPose(
    const rabbits_msgs::msg::ReferenceCommand & msg, rabcl::protocol::BodyPose & pose);

  // joint_name を names の添字（関節番号）に変換して out に詰める。無い名前は読み飛ばす。
  // 戻り値は詰めた数（max まで）
  static uint8_t ReferenceToJointCmds(
    const rabbits_msgs::msg::ReferenceCommand & msg, const std::vector<std::string> & names,
    rabcl::protocol::JointCmd * out, uint8_t max);

  // mode_name を mode_names の添字（モード番号）に変換して out に詰める。無い名前は読み飛ばす
  static uint8_t ReferenceToModes(
    const rabbits_msgs::msg::ReferenceCommand & msg,
    const std::vector<std::string> & mode_names, rabcl::protocol::ModeCmd * out, uint8_t max);
};
}  // namespace rabcl_ros2

#endif  // RABCL_ROS2__CONVERT_HPP_
