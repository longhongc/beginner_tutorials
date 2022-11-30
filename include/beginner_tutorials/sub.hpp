// Copyright 2022 Chang-Hong Chen
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

/**
 * @Brief A subscriber node
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
    /**
    * @Brief The constructor
    */
    MinimalSubscriber();

 private:
    /**
     * @Brief The callback function for the count subscriber
     *
     * @Param msg
     */
    void topic_callback(const std_msgs::msg::String & msg) const;  

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
    void logger(const std_msgs::msg::String& msg) const;
      
    /**
     * @Brief Subscriber
     */
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
