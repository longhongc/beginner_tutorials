#include <cstdint>
#include <memory>
#include <thread>

#include "gtest/gtest.h"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "beginner_tutorials/pub.hpp"

using namespace std::chrono_literals;

TEST(DummyTests, dummy1) {
  EXPECT_TRUE(true);
}

class TestPub : public ::testing::Test
{
public:
  TestPub() {}

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    minimal_pub_ = std::make_shared<MinimalPublisher>();
    clock_ = std::make_unique<rclcpp::Clock>(RCL_ROS_TIME);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(clock_);
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<MinimalPublisher> minimal_pub_;
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<std::thread> pub_thread_;
};

TEST_F(TestPub, testTF) {
  // Wait for publisher 3 seconds
  auto start = clock_->now();
  double duration_sec = 0;
  while (duration_sec < 3) {
    rclcpp::spin_some(minimal_pub_);
    duration_sec = (clock_->now() - start).seconds();
  }

  geometry_msgs::msg::TransformStamped t;

  // lookupTransform
  try {
    t = tf_buffer_->lookupTransform(
      "world", "talk",
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("rclcpp"),
      "Could not find transform from world to talk");
    FAIL();
  }

  // Received transform
  auto x = t.transform.translation.x;
  auto y = t.transform.translation.y;
  auto z = t.transform.translation.z;
  auto rx = t.transform.rotation.x;
  auto ry = t.transform.rotation.y;
  auto rz = t.transform.rotation.z;
  auto rw = t.transform.rotation.w;

  // Correct transform
  auto x2 = minimal_pub_->world_to_talk_tf.translation.x;
  auto y2 = minimal_pub_->world_to_talk_tf.translation.y;
  auto z2 = minimal_pub_->world_to_talk_tf.translation.z;
  auto rx2 = minimal_pub_->world_to_talk_tf.rotation.x;
  auto ry2 = minimal_pub_->world_to_talk_tf.rotation.y;
  auto rz2 = minimal_pub_->world_to_talk_tf.rotation.z;
  auto rw2 = minimal_pub_->world_to_talk_tf.rotation.w;

  EXPECT_FLOAT_EQ(x, x2);
  EXPECT_FLOAT_EQ(y, y2);
  EXPECT_FLOAT_EQ(z, z2);
  EXPECT_FLOAT_EQ(rx, rx2);
  EXPECT_FLOAT_EQ(ry, ry2);
  EXPECT_FLOAT_EQ(rz, rz2);
  EXPECT_FLOAT_EQ(rw, rw2);
}
