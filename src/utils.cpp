#include "rabcl_ros2/utils.hpp"

namespace rabcl_ros2
{
void Utils::CmdMsgToInfo(
  const attracts_msgs::msg::AttractsCommand & msg, rabcl::Info & info)
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

void Utils::InfoToImuMsg(
  const rabcl::Info & info, sensor_msgs::msg::Imu & msg)
{
  // Linear acceleration [m/s^2]
  msg.linear_acceleration.x = info.imu_.acc_x_;
  msg.linear_acceleration.y = info.imu_.acc_y_;
  msg.linear_acceleration.z = info.imu_.acc_z_;

  // Angular velocity [rad/s]
  msg.angular_velocity.x = info.imu_.gyro_x_;
  msg.angular_velocity.y = info.imu_.gyro_y_;
  msg.angular_velocity.z = info.imu_.gyro_z_;

  // Orientation: euler (heading, roll, pitch) → quaternion
  double h = static_cast<double>(info.imu_.euler_heading_);
  double r = static_cast<double>(info.imu_.euler_roll_);
  double p = static_cast<double>(info.imu_.euler_pitch_);

  double cy = std::cos(h * 0.5);
  double sy = std::sin(h * 0.5);
  double cr = std::cos(r * 0.5);
  double sr = std::sin(r * 0.5);
  double cp = std::cos(p * 0.5);
  double sp = std::sin(p * 0.5);

  msg.orientation.w = cy * cr * cp + sy * sr * sp;
  msg.orientation.x = cy * sr * cp - sy * cr * sp;
  msg.orientation.y = cy * cr * sp + sy * sr * cp;
  msg.orientation.z = sy * cr * cp - cy * sr * sp;
}

void Utils::InfoToJointStateMsg(
  const rabcl::Info & info, sensor_msgs::msg::JointState & msg)
{
  msg.name = {"yaw", "pitch", "chassis_fr", "chassis_fl", "chassis_br", "chassis_bl"};

  msg.position = {
    static_cast<double>(info.yaw_act_.position_),
    static_cast<double>(info.pitch_act_.position_),
    static_cast<double>(info.chassis_fr_act_.position_),
    static_cast<double>(info.chassis_fl_act_.position_),
    static_cast<double>(info.chassis_br_act_.position_),
    static_cast<double>(info.chassis_bl_act_.position_),
  };

  msg.velocity = {
    static_cast<double>(info.yaw_act_.velocity_),
    static_cast<double>(info.pitch_act_.velocity_),
    static_cast<double>(info.chassis_fr_act_.velocity_),
    static_cast<double>(info.chassis_fl_act_.velocity_),
    static_cast<double>(info.chassis_br_act_.velocity_),
    static_cast<double>(info.chassis_bl_act_.velocity_),
  };

  // effort: yaw/pitch = current [A], chassis = torque [Nm]
  msg.effort = {
    static_cast<double>(info.yaw_act_.current_),
    static_cast<double>(info.pitch_act_.current_),
    static_cast<double>(info.chassis_fr_act_.torque_),
    static_cast<double>(info.chassis_fl_act_.torque_),
    static_cast<double>(info.chassis_br_act_.torque_),
    static_cast<double>(info.chassis_bl_act_.torque_),
  };
}
}  // namespace rabcl_ros2
