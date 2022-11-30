// Copyright 2022 Chang-Hong Chen
#include "beginner_tutorials/pub.hpp"

MinimalPublisher::MinimalPublisher()
: Node("minimal_publisher"), count_(0)
{
  // Set default logger level to DEBUG
  if (rcutils_logging_set_logger_level(
      this->get_logger().get_name(),
      RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG) ==
    RCUTILS_RET_OK)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Set logger level DEBUG success.");
  } else {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
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

/**
 * @Brief The callback function that will be called and publish message
 *        for a given frequency
 */
void MinimalPublisher::timer_callback()
{
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
void MinimalPublisher::logger(const std_msgs::msg::String & msg)
{
  int count = stoi(msg.data);
  switch (count % 5) {
    case 0:
      RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "Count: " << msg.data);
      break;
    case 1:
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Count: " << msg.data);
      break;
    case 2:
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Count: " << msg.data);
      break;
    case 3:
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Count: " << msg.data);
      break;
    case 4:
      RCLCPP_FATAL_STREAM(
        this->get_logger(),
        "Count: " << msg.data);
      break;
    default:
      break;
  }
}

/**
 * @Brief The callback function for the service server
 *        that returns the current count value
 *
 * @Param request None
 * @Param response Return the count value
 */
void MinimalPublisher::get_count_callback(
  const std::shared_ptr<GetCount::Request> request,
  std::shared_ptr<GetCount::Response> response)
{

  (void) request;
  response->count = count_;
}

/**
 * @Brief  The callback function for the tf broadcaster timer
*/
void MinimalPublisher::broadcast_timer_callback()
{
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
}
