#include "rabcl_ros2/utils.hpp"

namespace rabcl_ros2
{
void Utils::CmdMsgToInfo(const attracts_msgs::msg::AttractsCommand& msg, rabcl::Info& info)
{
    info.chassis_vel_x_ = msg.chassis_vel_x.data;
    info.chassis_vel_y_ = msg.chassis_vel_y.data;
    info.chassis_vel_z_ = msg.chassis_vel_z.data;
    info.pitch_vel_ = msg.pitch_vel.data;
    info.yaw_vel_ = msg.yaw_vel.data;

    info.load_mode_ = msg.load_mode.data;
    info.fire_mode_ = msg.fire_mode.data;
    info.speed_mode_ = msg.speed_mode.data;
    info.chassis_mode_ = msg.chassis_mode.data;
}
} // namespace rabcl
