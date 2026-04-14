#include <gtest/gtest.h>

#include "attracts_msgs/msg/attracts_command.hpp"
#include "rabcl/utils/type.hpp"
#include "rabcl_ros2/utils.hpp"

namespace rabcl_ros2
{

TEST(UtilsTest, CmdMsgToInfoAllFields)
{
  attracts_msgs::msg::AttractsCommand msg;
  msg.chassis_vel.x = 1.0;
  msg.chassis_vel.y = 2.0;
  msg.chassis_vel.z = 3.0;
  msg.pitch_pos = 0.5;
  msg.yaw_pos = 1.5;
  msg.load_mode = 1;
  msg.fire_mode = 2;
  msg.speed_mode = 0;
  msg.chassis_mode = 1;

  rabcl::Info info;
  Utils::CmdMsgToInfo(msg, info);

  EXPECT_FLOAT_EQ(info.chassis_vel_x_, 1.0f);
  EXPECT_FLOAT_EQ(info.chassis_vel_y_, 2.0f);
  EXPECT_FLOAT_EQ(info.chassis_vel_z_, 3.0f);
  EXPECT_FLOAT_EQ(info.pitch_pos_, 0.5f);
  EXPECT_FLOAT_EQ(info.yaw_pos_, 1.5f);
  EXPECT_EQ(info.load_mode_, 1);
  EXPECT_EQ(info.fire_mode_, 2);
  EXPECT_EQ(info.speed_mode_, 0);
  EXPECT_EQ(info.chassis_mode_, 1);
}

TEST(UtilsTest, CmdMsgToInfoZeroValues)
{
  attracts_msgs::msg::AttractsCommand msg;
  rabcl::Info info;
  Utils::CmdMsgToInfo(msg, info);

  EXPECT_FLOAT_EQ(info.chassis_vel_x_, 0.0f);
  EXPECT_FLOAT_EQ(info.chassis_vel_y_, 0.0f);
  EXPECT_FLOAT_EQ(info.chassis_vel_z_, 0.0f);
  EXPECT_EQ(info.load_mode_, 0);
}

// --- InfoToImuMsg tests ---

TEST(UtilsTest, InfoToImuMsgAccelGyro)
{
  rabcl::Info info{};
  info.imu_.acc_x_ = 1.0f;
  info.imu_.acc_y_ = -2.0f;
  info.imu_.acc_z_ = 9.81f;
  info.imu_.gyro_x_ = 0.1f;
  info.imu_.gyro_y_ = -0.2f;
  info.imu_.gyro_z_ = 0.3f;

  sensor_msgs::msg::Imu msg;
  Utils::InfoToImuMsg(info, msg);

  EXPECT_DOUBLE_EQ(msg.linear_acceleration.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.y, -2.0);
  EXPECT_FLOAT_EQ(msg.linear_acceleration.z, 9.81);
  EXPECT_FLOAT_EQ(msg.angular_velocity.x, 0.1);
  EXPECT_FLOAT_EQ(msg.angular_velocity.y, -0.2);
  EXPECT_FLOAT_EQ(msg.angular_velocity.z, 0.3);
}

TEST(UtilsTest, InfoToImuMsgOrientationIdentity)
{
  rabcl::Info info{};
  info.imu_.euler_heading_ = 0.0f;
  info.imu_.euler_roll_ = 0.0f;
  info.imu_.euler_pitch_ = 0.0f;

  sensor_msgs::msg::Imu msg;
  Utils::InfoToImuMsg(info, msg);

  EXPECT_NEAR(msg.orientation.w, 1.0, 1e-6);
  EXPECT_NEAR(msg.orientation.x, 0.0, 1e-6);
  EXPECT_NEAR(msg.orientation.y, 0.0, 1e-6);
  EXPECT_NEAR(msg.orientation.z, 0.0, 1e-6);
}

TEST(UtilsTest, InfoToImuMsgOrientationNonZero)
{
  rabcl::Info info{};
  info.imu_.euler_heading_ = M_PI / 2.0f;
  info.imu_.euler_roll_ = 0.0f;
  info.imu_.euler_pitch_ = 0.0f;

  sensor_msgs::msg::Imu msg;
  Utils::InfoToImuMsg(info, msg);

  // quaternion norm should be 1
  double norm = std::sqrt(
    msg.orientation.w * msg.orientation.w +
    msg.orientation.x * msg.orientation.x +
    msg.orientation.y * msg.orientation.y +
    msg.orientation.z * msg.orientation.z);
  EXPECT_NEAR(norm, 1.0, 1e-6);

  // yaw = pi/2 → z component should be ~sin(pi/4) = 0.707
  EXPECT_NEAR(msg.orientation.z, std::sin(M_PI / 4.0), 1e-6);
}

// --- InfoToJointStateMsg tests ---

TEST(UtilsTest, InfoToJointStateMsgNames)
{
  rabcl::Info info{};
  sensor_msgs::msg::JointState msg;
  Utils::InfoToJointStateMsg(info, msg);

  ASSERT_EQ(msg.name.size(), 6u);
  EXPECT_EQ(msg.name[0], "yaw");
  EXPECT_EQ(msg.name[1], "pitch");
  EXPECT_EQ(msg.name[2], "chassis_fr");
  EXPECT_EQ(msg.name[3], "chassis_fl");
  EXPECT_EQ(msg.name[4], "chassis_br");
  EXPECT_EQ(msg.name[5], "chassis_bl");
}

TEST(UtilsTest, InfoToJointStateMsgValues)
{
  rabcl::Info info{};
  info.yaw_act_.position_ = 1.0f;
  info.yaw_act_.velocity_ = 2.0f;
  info.yaw_act_.current_ = 0.5f;
  info.pitch_act_.position_ = -1.0f;
  info.pitch_act_.velocity_ = -2.0f;
  info.pitch_act_.current_ = 0.3f;
  info.chassis_fr_act_.position_ = 10.0f;
  info.chassis_fr_act_.velocity_ = 20.0f;
  info.chassis_fr_act_.torque_ = 1.5f;
  info.chassis_fl_act_.position_ = 11.0f;
  info.chassis_fl_act_.velocity_ = 21.0f;
  info.chassis_fl_act_.torque_ = 1.6f;
  info.chassis_br_act_.position_ = 12.0f;
  info.chassis_br_act_.velocity_ = 22.0f;
  info.chassis_br_act_.torque_ = 1.7f;
  info.chassis_bl_act_.position_ = 13.0f;
  info.chassis_bl_act_.velocity_ = 23.0f;
  info.chassis_bl_act_.torque_ = 1.8f;

  sensor_msgs::msg::JointState msg;
  Utils::InfoToJointStateMsg(info, msg);

  ASSERT_EQ(msg.position.size(), 6u);
  ASSERT_EQ(msg.velocity.size(), 6u);
  ASSERT_EQ(msg.effort.size(), 6u);

  EXPECT_DOUBLE_EQ(msg.position[0], 1.0);
  EXPECT_DOUBLE_EQ(msg.velocity[0], 2.0);
  EXPECT_DOUBLE_EQ(msg.effort[0], 0.5);   // yaw current

  EXPECT_DOUBLE_EQ(msg.position[1], -1.0);
  EXPECT_NEAR(msg.effort[1], 0.3, 1e-6);   // pitch current

  EXPECT_DOUBLE_EQ(msg.position[2], 10.0);
  EXPECT_DOUBLE_EQ(msg.effort[2], 1.5);   // chassis_fr torque

  EXPECT_DOUBLE_EQ(msg.position[5], 13.0);
  EXPECT_NEAR(msg.effort[5], 1.8, 1e-6);   // chassis_bl torque
}

TEST(UtilsTest, InfoToJointStateMsgZeroValues)
{
  rabcl::Info info{};
  sensor_msgs::msg::JointState msg;
  Utils::InfoToJointStateMsg(info, msg);

  ASSERT_EQ(msg.position.size(), 6u);
  for (size_t i = 0; i < 6; i++) {
    EXPECT_DOUBLE_EQ(msg.position[i], 0.0);
    EXPECT_DOUBLE_EQ(msg.velocity[i], 0.0);
    EXPECT_DOUBLE_EQ(msg.effort[i], 0.0);
  }
}

}  // namespace rabcl_ros2

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
