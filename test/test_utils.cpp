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

}  // namespace rabcl_ros2

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
