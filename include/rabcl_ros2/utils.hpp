#ifndef RABCL_ROS2__UTILS_HPP_
#define RABCL_ROS2__UTILS_HPP_

#include <cmath>

#include <attracts_msgs/msg/attracts_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "rabcl/utils/type.hpp"

namespace rabcl_ros2
{
class Utils
{
public:
  static void CmdMsgToInfo(
    const attracts_msgs::msg::AttractsCommand & msg, rabcl::Info & info);

  static void InfoToImuMsg(
    const rabcl::Info & info, sensor_msgs::msg::Imu & msg);

  static void InfoToJointStateMsg(
    const rabcl::Info & info, sensor_msgs::msg::JointState & msg);
};
}  // namespace rabcl_ros2

#endif  // RABCL_ROS2__UTILS_HPP_
