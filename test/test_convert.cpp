#include <gtest/gtest.h>
#include <string>
#include <vector>
#include "rabcl_ros2/convert.hpp"

namespace rabcl_ros2
{

using rabcl::protocol::BodyPose;
using rabcl::protocol::ChassisVel;
using rabcl::protocol::Imu;
using rabcl::protocol::JointCmd;
using rabcl::protocol::JointState;
using rabcl::protocol::ModeCmd;

TEST(Convert, JointStatesToMsgUsesNamesByIndex)
{
  const JointState joints[2] = {
    {2, 1.0f, 2.0f, 3.0f, 40.0f, 0},
    {0, -1.0f, -2.0f, -3.0f, 30.0f, 1},
  };
  const std::vector<std::string> names = {"a", "b", "c"};
  sensor_msgs::msg::JointState msg;
  EXPECT_TRUE(Convert::JointStatesToMsg(joints, 2, names, msg));
  ASSERT_EQ(msg.name.size(), 2u);
  EXPECT_EQ(msg.name[0], "c");
  EXPECT_EQ(msg.name[1], "a");
  EXPECT_DOUBLE_EQ(msg.position[0], 1.0);
  EXPECT_DOUBLE_EQ(msg.velocity[0], 2.0);
  EXPECT_DOUBLE_EQ(msg.effort[0], 3.0);
  EXPECT_DOUBLE_EQ(msg.position[1], -1.0);
}

TEST(Convert, JointStatesToMsgSkipsUnknownId)
{
  const JointState joints[2] = {{5, 1.0f, 0, 0, 0, 0}, {1, 2.0f, 0, 0, 0, 0}};
  const std::vector<std::string> names = {"a", "b", "c"};
  sensor_msgs::msg::JointState msg;
  EXPECT_FALSE(Convert::JointStatesToMsg(joints, 2, names, msg));
  ASSERT_EQ(msg.name.size(), 1u);
  EXPECT_EQ(msg.name[0], "b");
  EXPECT_DOUBLE_EQ(msg.position[0], 2.0);
}

TEST(Convert, ImuToMsgQuaternionIdentityForZeroEuler)
{
  Imu imu = {{1.0f, 2.0f, 9.81f}, {0.1f, 0.2f, 0.3f}, {0.0f, 0.0f, 0.0f}};
  sensor_msgs::msg::Imu msg;
  Convert::ImuToMsg(imu, msg);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.z, 9.81f);
  EXPECT_DOUBLE_EQ(msg.angular_velocity.y, 0.2f);
  EXPECT_DOUBLE_EQ(msg.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(msg.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(msg.orientation.z, 0.0);
}

TEST(Convert, ReferenceIsConsistentDetectsMismatch)
{
  rabbits_msgs::msg::ReferenceCommand msg;
  EXPECT_TRUE(Convert::ReferenceIsConsistent(msg));  // 全部空
  msg.joint_name = {"a"};
  msg.joint_position = {1.0f};
  msg.joint_velocity = {0.0f};
  msg.joint_effort = {0.0f};
  msg.joint_kp = {0.0f};
  msg.joint_kd = {0.0f};
  msg.mode_name = {"m"};
  msg.mode_value = {1};
  EXPECT_TRUE(Convert::ReferenceIsConsistent(msg));
  msg.mode_value = {};
  EXPECT_FALSE(Convert::ReferenceIsConsistent(msg));
}

TEST(Convert, ReferenceWithMismatchedArraysIsRejected)
{
  rabbits_msgs::msg::ReferenceCommand msg;
  msg.joint_name = {"a", "b"};
  msg.joint_position = {1.0f};
  msg.joint_velocity = {0.0f, 0.0f};
  msg.joint_effort = {0.0f, 0.0f};
  msg.joint_kp = {0.0f, 0.0f};
  msg.joint_kd = {0.0f, 0.0f};
  EXPECT_FALSE(Convert::ReferenceIsConsistent(msg));
}

TEST(Convert, ReferenceToChassisVelCopiesVector)
{
  rabbits_msgs::msg::ReferenceCommand msg;
  msg.chassis_vel.x = 0.5;
  msg.chassis_vel.y = -0.25;
  msg.chassis_vel.z = 1.5;
  ChassisVel vel = {};
  Convert::ReferenceToChassisVel(msg, vel);
  EXPECT_FLOAT_EQ(vel.vx, 0.5f);
  EXPECT_FLOAT_EQ(vel.vy, -0.25f);
  EXPECT_FLOAT_EQ(vel.wz, 1.5f);
}

TEST(Convert, ReferenceToBodyPoseCopiesFields)
{
  rabbits_msgs::msg::ReferenceCommand msg;
  msg.body_height = 0.5;
  msg.body_roll = 0.1;
  msg.body_pitch = -0.2;
  BodyPose pose = {};
  Convert::ReferenceToBodyPose(msg, pose);
  EXPECT_FLOAT_EQ(pose.height, 0.5f);
  EXPECT_FLOAT_EQ(pose.roll, 0.1f);
  EXPECT_FLOAT_EQ(pose.pitch, -0.2f);
}

TEST(Convert, ReferenceToJointCmdsMapsNames)
{
  rabbits_msgs::msg::ReferenceCommand msg;
  msg.joint_name = {"pitch_joint"};
  msg.joint_position = {0.3f};
  msg.joint_velocity = {0.1f};
  msg.joint_effort = {0.2f};
  msg.joint_kp = {5.0f};
  msg.joint_kd = {0.5f};
  const std::vector<std::string> names = {"yaw_joint", "pitch_joint"};
  JointCmd out[4] = {};
  ASSERT_EQ(Convert::ReferenceToJointCmds(msg, names, out, 4), 1);
  EXPECT_EQ(out[0].id, 1);
  EXPECT_FLOAT_EQ(out[0].position, 0.3f);
  EXPECT_FLOAT_EQ(out[0].velocity, 0.1f);
  EXPECT_FLOAT_EQ(out[0].effort, 0.2f);
  EXPECT_FLOAT_EQ(out[0].kp, 5.0f);
  EXPECT_FLOAT_EQ(out[0].kd, 0.5f);
}

TEST(Convert, ReferenceToJointCmdsSkipsUnknownName)
{
  rabbits_msgs::msg::ReferenceCommand msg;
  msg.joint_name = {"unknown", "yaw_joint"};
  msg.joint_position = {1.0f, 2.0f};
  msg.joint_velocity = {0.0f, 0.0f};
  msg.joint_effort = {0.0f, 0.0f};
  msg.joint_kp = {0.0f, 0.0f};
  msg.joint_kd = {0.0f, 0.0f};
  const std::vector<std::string> names = {"yaw_joint", "pitch_joint"};
  JointCmd out[4] = {};
  ASSERT_EQ(Convert::ReferenceToJointCmds(msg, names, out, 4), 1);
  EXPECT_EQ(out[0].id, 0);
  EXPECT_FLOAT_EQ(out[0].position, 2.0f);
}

TEST(Convert, ReferenceToJointCmdsRespectsMax)
{
  rabbits_msgs::msg::ReferenceCommand msg;
  msg.joint_name = {"a", "b", "c"};
  msg.joint_position = {1.0f, 2.0f, 3.0f};
  msg.joint_velocity = {0.0f, 0.0f, 0.0f};
  msg.joint_effort = {0.0f, 0.0f, 0.0f};
  msg.joint_kp = {0.0f, 0.0f, 0.0f};
  msg.joint_kd = {0.0f, 0.0f, 0.0f};
  const std::vector<std::string> names = {"a", "b", "c"};
  JointCmd out[2] = {};
  EXPECT_EQ(Convert::ReferenceToJointCmds(msg, names, out, 2), 2);
  EXPECT_EQ(out[1].id, 1);
}

TEST(Convert, ReferenceToModesMapsNames)
{
  rabbits_msgs::msg::ReferenceCommand msg;
  msg.mode_name = {"fire", "nope", "load"};
  msg.mode_value = {2, 9, 1};
  const std::vector<std::string> mode_names = {"load", "fire", "speed", "chassis"};
  ModeCmd out[4] = {};
  ASSERT_EQ(Convert::ReferenceToModes(msg, mode_names, out, 4), 2);
  EXPECT_EQ(out[0].id, 1);
  EXPECT_EQ(out[0].value, 2);
  EXPECT_EQ(out[1].id, 0);
  EXPECT_EQ(out[1].value, 1);
}

}  // namespace rabcl_ros2

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
