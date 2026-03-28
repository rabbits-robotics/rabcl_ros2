#ifndef RABCL_ROS2__UTILS_HPP_
#define RABCL_ROS2__UTILS_HPP_

#include <cmath>

#include <attracts_msgs/msg/attracts_command.hpp>
#include "rabcl/utils/type.hpp"

namespace rabcl_ros2
{
class Utils
{
public:
  static void CmdMsgToInfo(
    const attracts_msgs::msg::AttractsCommand & msg, rabcl::Info & info);
};
}  // namespace rabcl_ros2

#endif  // RABCL_ROS2__UTILS_HPP_
