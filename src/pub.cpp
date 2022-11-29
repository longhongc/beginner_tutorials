// Copyright 2022 Chang-Hong Chen

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
class MinimalPublisher : public rclcpp::Node {
 public:
   /**
    * @Brief The constructor
    */
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0) {
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

      // Set ros parameter "count"
      auto param_desc =
           rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description =
        "\nThis parameter is the count value passed between "
        "the minimal publisher and the minimal subscriber."
        "\n\n It also determines the logger level of both nodes.";

      this->declare_parameter("count", count_, param_desc);

      count_ = this->get_parameter("count").get_parameter_value().get<int>();
      RCLCPP_INFO_STREAM(this->get_logger(), "Count starts from " << count_);


      // Create a publisher for count
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
          1s, std::bind(&MinimalPublisher::timer_callback, this));

      // Create a service for modifying count
      std::string get_count_service_name =
                  "/" + std::string(this->get_name()) + "/" + "GetCount";
      get_count_service_ = this->create_service<GetCount>(
          get_count_service_name,
          std::bind(&MinimalPublisher::get_count_callback, this, _1, _2));

      // Create tf broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // Timer constantly publishing tf info
      tf_timer_ = this->create_wall_timer(
          200ms, std::bind(&MinimalPublisher::broadcast_timer_callback, this));
    }

 private:
    /**
     * @Brief The callback function that will be called and publish message
     *        for a given frequency
     */
    void timer_callback() {
      auto message = std_msgs::msg::String();
      // Receive the count parameter
      count_ = this->get_parameter("count").get_parameter_value().get<int>();

      message.data = std::to_string(count_);

      // Determine which level of logger to use
      this->logger(message);
      publisher_->publish(message);

      count_++;
      // Set the count param with the new count
      this->set_parameter(rclcpp::Parameter("count", count_));
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
    void logger(const std_msgs::msg::String& msg) {
      int count = stoi(msg.data);
      switch (count % 5) {
        case 0:
          RCLCPP_DEBUG_STREAM(this->get_logger(),
            "Count: " << msg.data);
          break;
        case 1:
          RCLCPP_INFO_STREAM(this->get_logger(),
            "Count: " << msg.data);
          break;
        case 2:
          RCLCPP_WARN_STREAM(this->get_logger(),
            "Count: " << msg.data);
          break;
        case 3:
          RCLCPP_ERROR_STREAM(this->get_logger(),
            "Count: " << msg.data);
          break;
        case 4:
          RCLCPP_FATAL_STREAM(this->get_logger(),
            "Count: " << msg.data);
          break;
        default:
          break;
      }

      return;
    }

    /**
     * @Brief The callback function for the service server
     *        that returns the current count value
     *
     * @Param request None
     * @Param response Return the count value
     */
    void get_count_callback(const std::shared_ptr<GetCount::Request> request,
                            std::shared_ptr<GetCount::Response> response) {
      (void) request;
      response->count = count_;
    }

    /**
     * @Brief  The callback function for the tf broadcaster timer
     */
    void broadcast_timer_callback() {
      tf2::Quaternion tf2_quat;
      tf2_quat.setRPY(0, 0, 1.57);

      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "world";
      t.child_frame_id = "talk";
      t.transform.translation.x = 1.0;
      t.transform.translation.y = 2.0;
      t.transform.translation.z = 3.0;
      t.transform.rotation = tf2::toMsg(tf2_quat);
      
      tf_broadcaster_->sendTransform(t);
    }

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

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
       "The logger level will depends on the "
       "remainder of current count divided by 5.");

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
              "\nThe logger level of each remainder value:"
              "\n 0) Debug \n 1) INFO \n 2) WARN \n 3) ERROR \n 4) FATAL \n");

  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
