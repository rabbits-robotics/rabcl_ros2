#ifndef RABCL_ROS2_UTILS_HPP
#define RABCL_ROS2_UTILS_HPP

#include <cstdint>
#include <math.h>

#include <attracts_msgs/msg/attracts_command.hpp>
#include <rabcl/utils/type.hpp>

namespace rabcl_ros2
{
class Utils
{
public:
    static void CmdMsgToInfo(const attracts_msgs::msg::AttractsCommand& msg, rabcl::Info& info);

};
} // namespace rabcl

#endif
