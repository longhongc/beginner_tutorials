// Copyright 2022 Chang-Hong Chen
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "beginner_tutorials/srv/get_count.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using GetCount = beginner_tutorials::srv::GetCount;

/**
 * @Brief  A publisher node
 */
class MinimalPublisher : public rclcpp::Node
{
public:
  /**
   * @Brief The constructor
   */
  MinimalPublisher();

private:
  /**
   * @Brief The callback function that will be called and publish message
   *        for a given frequency
   */
  void timer_callback();

  /**
   * @Brief This function uses different logger level based on
   *        the remainder of the count divided by 5.
   *        0) DEBUG
   *        1) INFO
   *        2) WARN
   *        3) ERROR
   *        4) FATAL
   *
   * @Param msg The count message in string
   */
  void logger(const std_msgs::msg::String & msg);

  /**
   * @Brief The callback function for the service server
   *        that returns the current count value
   *
   * @Param request None
   * @Param response Return the count value
   */
  void get_count_callback(
    const std::shared_ptr<GetCount::Request> request,
    std::shared_ptr<GetCount::Response> response);

  /**
   * @Brief  The callback function for the tf broadcaster timer
   */
  void broadcast_timer_callback();

  /**
   * @Brief  Timer for periodically publishing message
   */
  rclcpp::TimerBase::SharedPtr timer_;
  /**
   * @Brief  Publisher
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  /**
   * @Brief Service Server for getting count
   */
  rclcpp::Service<GetCount>::SharedPtr get_count_service_;

  /**
   * @Brief  Tf broadcaster
   */
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /**
   * @Brief  Timer for preiodically publishing tf info
   */
  rclcpp::TimerBase::SharedPtr tf_timer_;

  int count_;
};
