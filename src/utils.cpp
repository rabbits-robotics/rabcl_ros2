#include "rabcl_ros2/utils.hpp"

namespace rabcl_ros2
{
void Utils::CmdMsgToInfo(const attracts_msgs::msg::AttractsCommand& msg, rabcl::Info& info)
{
    info.chassis_vel_x_ = msg.chassis_vel.x;
    info.chassis_vel_y_ = msg.chassis_vel.y;
    info.chassis_vel_z_ = msg.chassis_vel.z;
    info.pitch_pos_ = msg.pitch_pos;
    info.yaw_pos_ = msg.yaw_pos;

    info.load_mode_ = msg.load_mode;
    info.fire_mode_ = msg.fire_mode;
    info.speed_mode_ = msg.speed_mode;
    info.chassis_mode_ = msg.chassis_mode;
}
} // namespace rabcl
