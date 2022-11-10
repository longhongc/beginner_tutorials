// Copyright 2022 Chang-Hong Chen

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
    MinimalSubscriber()
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

 private:
    /**
     * @Brief The callback function for the count subscriber
     *
     * @Param msg
     */
    void topic_callback(const std_msgs::msg::String & msg) const {
      // Determine which level of logger to use
      this->logger(msg);
    }

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

    void logger(const std_msgs::msg::String& msg) const {
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

    /**
     * @Brief Subscriber
     */
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

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
