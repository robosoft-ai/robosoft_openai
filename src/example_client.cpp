#include "ai_msgs/srv/string_image_prompt.hpp"
#include "rclcpp/rclcpp.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cstdlib>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("open_ai_client");
  rclcpp::Client<ai_msgs::srv::StringImagePrompt>::SharedPtr client =
      node->create_client<ai_msgs::srv::StringImagePrompt>("openai_server");

  auto request = std::make_shared<ai_msgs::srv::StringImagePrompt::Request>();
  request->prompt = "Is this a wooden table? Please respond in one word, yes or no.";

  // Add an image of a wooden table to the OpenAI request
  std::string pkg_share_directory = ament_index_cpp::get_package_share_directory("ros2_openai_server");
  std::string image_path = pkg_share_directory + "/test_data/wood_table.jpg";
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  request->image = *msg;

  while (!client->wait_for_service(5s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %b", result.get()->bool_response);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  return 0;
}
