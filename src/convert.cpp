#include "rabcl_ros2/convert.hpp"

#include <algorithm>
#include <cmath>

namespace rabcl_ros2
{

namespace
{

// names の中の name の添字。無ければ -1
int IndexOf(const std::vector<std::string> & names, const std::string & name)
{
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end()) {
    return -1;
  }
  return static_cast<int>(it - names.begin());
}

}  // namespace

bool Convert::JointStatesToMsg(
  const rabcl::protocol::JointState * joints, uint8_t count,
  const std::vector<std::string> & names, sensor_msgs::msg::JointState & msg)
{
  bool all_known = true;
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();
  for (uint8_t i = 0; i < count; ++i) {
    const auto & j = joints[i];
    if (j.id >= names.size()) {
      all_known = false;
      continue;
    }
    msg.name.push_back(names[j.id]);
    msg.position.push_back(static_cast<double>(j.position));
    msg.velocity.push_back(static_cast<double>(j.velocity));
    msg.effort.push_back(static_cast<double>(j.effort));
  }
  return all_known;
}

void Convert::ImuToMsg(const rabcl::protocol::Imu & imu, sensor_msgs::msg::Imu & msg)
{
  msg.linear_acceleration.x = imu.acc[0];
  msg.linear_acceleration.y = imu.acc[1];
  msg.linear_acceleration.z = imu.acc[2];

  msg.angular_velocity.x = imu.gyro[0];
  msg.angular_velocity.y = imu.gyro[1];
  msg.angular_velocity.z = imu.gyro[2];

  // euler = (heading, roll, pitch) → quaternion
  const double h = static_cast<double>(imu.euler[0]);
  const double r = static_cast<double>(imu.euler[1]);
  const double p = static_cast<double>(imu.euler[2]);
  const double cy = std::cos(h * 0.5);
  const double sy = std::sin(h * 0.5);
  const double cr = std::cos(r * 0.5);
  const double sr = std::sin(r * 0.5);
  const double cp = std::cos(p * 0.5);
  const double sp = std::sin(p * 0.5);
  msg.orientation.w = cy * cr * cp + sy * sr * sp;
  msg.orientation.x = cy * sr * cp - sy * cr * sp;
  msg.orientation.y = cy * cr * sp + sy * sr * cp;
  msg.orientation.z = sy * cr * cp - cy * sr * sp;
}

bool Convert::ReferenceIsConsistent(const rabbits_msgs::msg::ReferenceCommand & msg)
{
  const size_t n = msg.joint_name.size();
  const bool joints_ok = msg.joint_position.size() == n && msg.joint_velocity.size() == n &&
    msg.joint_effort.size() == n && msg.joint_kp.size() == n && msg.joint_kd.size() == n;
  const bool modes_ok = msg.mode_value.size() == msg.mode_name.size();
  return joints_ok && modes_ok;
}

void Convert::ReferenceToChassisVel(
  const rabbits_msgs::msg::ReferenceCommand & msg, rabcl::protocol::ChassisVel & vel)
{
  vel.vx = static_cast<float>(msg.chassis_vel.x);
  vel.vy = static_cast<float>(msg.chassis_vel.y);
  vel.wz = static_cast<float>(msg.chassis_vel.z);
}

void Convert::ReferenceToBodyPose(
  const rabbits_msgs::msg::ReferenceCommand & msg, rabcl::protocol::BodyPose & pose)
{
  pose.height = msg.body_height;
  pose.roll = msg.body_roll;
  pose.pitch = msg.body_pitch;
}

uint8_t Convert::ReferenceToJointCmds(
  const rabbits_msgs::msg::ReferenceCommand & msg, const std::vector<std::string> & names,
  rabcl::protocol::JointCmd * out, uint8_t max)
{
  uint8_t n = 0;
  for (size_t i = 0; i < msg.joint_name.size() && n < max; ++i) {
    const int id = IndexOf(names, msg.joint_name[i]);
    if (id < 0) {
      continue;
    }
    out[n].id = static_cast<uint8_t>(id);
    out[n].position = msg.joint_position[i];
    out[n].velocity = msg.joint_velocity[i];
    out[n].effort = msg.joint_effort[i];
    out[n].kp = msg.joint_kp[i];
    out[n].kd = msg.joint_kd[i];
    ++n;
  }
  return n;
}

uint8_t Convert::ReferenceToModes(
  const rabbits_msgs::msg::ReferenceCommand & msg,
  const std::vector<std::string> & mode_names, rabcl::protocol::ModeCmd * out, uint8_t max)
{
  uint8_t n = 0;
  for (size_t i = 0; i < msg.mode_name.size() && n < max; ++i) {
    const int id = IndexOf(mode_names, msg.mode_name[i]);
    if (id < 0) {
      continue;
    }
    out[n].id = static_cast<uint8_t>(id);
    out[n].value = msg.mode_value[i];
    ++n;
  }
  return n;
}

}  // namespace rabcl_ros2
