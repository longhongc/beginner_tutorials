// Copyright 2022 Chang-Hong Chen
#include "beginner_tutorials/sub.hpp"

MinimalSubscriber::MinimalSubscriber()
: Node("minimal_subscriber") {
  // Set default logger level to DEBUG
  if (rcutils_logging_set_logger_level(this->get_logger().get_name(),
                  RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG)
      == RCUTILS_RET_OK) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Set logger level DEBUG success.");
  } else {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Set logger level DEBUG fails.");
  }

  // Create a subscriber for count
  subscription_ = this->create_subscription<std_msgs::msg::String>(
  "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
}

void MinimalSubscriber::topic_callback(const std_msgs::msg::String & msg) const {
  // Determine which level of logger to use
  this->logger(msg);
}

void MinimalSubscriber::logger(const std_msgs::msg::String& msg) const {
  int count = stoi(msg.data);
  switch (count % 5) {
    case 0:
      RCLCPP_DEBUG_STREAM(this->get_logger(),
        "I heard count: " << msg.data);
      break;
    case 1:
      RCLCPP_INFO_STREAM(this->get_logger(),
        "I heard count: " << msg.data);
      break;
    case 2:
      RCLCPP_WARN_STREAM(this->get_logger(),
        "I heard count: " << msg.data);
      break;
    case 3:
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "I heard count: " << msg.data);
      break;
    case 4:
      RCLCPP_FATAL_STREAM(this->get_logger(),
        "I heard count: " << msg.data);
      break;
    default:
      break;

    return;
  }
}
