#include <rclcpp/rclcpp.hpp>

#include "gtest/gtest.h"

int main(int argc, char ** argv)
{
  // rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  // rclcpp::shutdown();
  return result;
}
