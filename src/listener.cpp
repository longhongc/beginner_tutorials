// Copyright 2022 Chang-Hong Chen
#include "beginner_tutorials/sub.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
       "The logger level will depends on the "
       "remainder of received count divided by 5.");

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
              "\nThe logger level of each remainder value:"
              "\n 0) Debug \n 1) INFO \n 2) WARN \n 3) ERROR \n 4) FATAL \n");

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
