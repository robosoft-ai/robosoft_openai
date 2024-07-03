#include "ros2_openai_server/convert_image_to_base_64.hpp"

#include "ai_msgs/srv/bool_response.hpp"
#include "ai_msgs/srv/string_response.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stdlib.h>

namespace openai_server
{
class OpenAIServer : public rclcpp::Node
{
public:
  OpenAIServer() : Node("service_client")
  {
    bool_response_srv_ = create_service<ai_msgs::srv::BoolResponse>(
        "openai_bool_response",
        std::bind(&OpenAIServer::boolResponseCallback, this, std::placeholders::_1, std::placeholders::_2));

    string_response_srv_ = create_service<ai_msgs::srv::StringResponse>(
        "openai_string_response",
        std::bind(&OpenAIServer::stringResponseCallback, this, std::placeholders::_1, std::placeholders::_2));

    curl_ = curl_easy_init();
    const std::string base_url = "https://api.openai.com/v1/chat/completions";
    curl_easy_setopt(curl_, CURLOPT_URL, base_url.c_str());
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, curlWriteCallback);

    // Get OpenAI key from environment variable
    const char* openai_key = std::getenv("OPENAI_API_KEY");
    openai_key_string_ = std::string(openai_key);
    if (openai_key_string_.empty())
    {
      RCLCPP_FATAL_STREAM(this->get_logger(), "An OpenAI key environment variable was not found.");
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Awaiting a prompt");
  }

  ~OpenAIServer()
  {
    curl_easy_cleanup(curl_);
  }

private:
  /**
   * @brief A callback needed by curl to handle the OpenAI response
   * @return size_t response size
   */
  static size_t curlWriteCallback(void* contents, size_t size, size_t nmemb, std::string* openai_response)
  {
    size_t total_size = size * nmemb;
    openai_response->append(reinterpret_cast<char*>(contents), total_size);
    return total_size;
  }

  /**
   * @brief Send a prompt which may include an image to OpenAI
   *
   * @tparam T The response type may be bool or string
   * @param prompt Input prompt
   * @param image Input image
   * @param response_filter How to filter the response from OpenAI. Search for a boolean yes/no, for example.
   * @param response The response, which will be returned from the service
   * @return true if successful
   */
  template <typename T>
  bool sendOpenAIPrompt(const std::string& prompt, const sensor_msgs::msg::Image& image, T& response)
  {
    std::string openai_response;

    if (curl_)
    {
      // Prepare the request data
      nlohmann::json request_data;
      request_data["model"] = "gpt-4o";
      request_data["messages"][0]["role"] = "user";
      // Attach the image, if any
      // See https://platform.openai.com/docs/guides/vision
      if (image.height > 0)
      {
        std::string encoded_image;
        try
        {
          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, image.encoding);
          encoded_image = convert_image_to_base_64(std::nullopt, std::optional<const cv::Mat>(cv_ptr->image));
        }
        catch (cv_bridge::Exception& e)
        {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return false;
        }
        request_data["messages"][0]["content"][0]["type"] = "text";
        request_data["messages"][0]["content"][0]["text"] = prompt;
        request_data["messages"][0]["content"][1]["type"] = "image_url";
        request_data["messages"][0]["content"][1]["image_url"]["url"] = ("data:image/jpg;base64," + encoded_image);
      }
      else  // No image
      {
        request_data["messages"][0]["content"] = prompt;
      }

      std::string formatted_request = request_data.dump();

      struct curl_slist* headers = NULL;
      headers = curl_slist_append(headers, "Content-Type: application/json");
      headers = curl_slist_append(headers, ("Authorization: Bearer " + openai_key_string_).c_str());

      // Set CURL options
      curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, formatted_request.c_str());
      curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, formatted_request.length());
      curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);
      curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &openai_response);

      // Perform the request
      CURLcode res = curl_easy_perform(curl_);
      if (res != CURLE_OK)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "curl_easy_perform() failed: " << curl_easy_strerror(res));
        return false;
      }

      curl_slist_free_all(headers);
    }
    else
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to initialize CURL");
      return false;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), openai_response);
    nlohmann::json jresponse = nlohmann::json::parse(openai_response);
    std::string string_response = jresponse["choices"][0]["message"]["content"].get<std::string>();

    if constexpr (std::is_same_v<T, std::string>)
    {
      // Pass the OpenAI string response forward
      response = string_response;
    }
    else if constexpr (std::is_same_v<T, bool>)
    {
      // Parse for a one-word Yes/No reply
      RCLCPP_INFO_STREAM(this->get_logger(), string_response);
      if ((string_response.find("Yes") != std::string::npos) || (string_response.find("yes") != std::string::npos))
      {
        response = true;
      }
      else
      {
        response = false;
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unexpected type");
      return false;
    }

    return true;
  }

  /**
   * @brief Callback for a BoolResponse service type
   *
   * @param request service request
   * @param response bool response, parsed from OpenAI's string response
   */
  void boolResponseCallback(const std::shared_ptr<ai_msgs::srv::BoolResponse::Request> request,
                            std::shared_ptr<ai_msgs::srv::BoolResponse::Response> response)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Incoming BoolResponse request: " << request->prompt);
    bool bool_response;
    bool result = sendOpenAIPrompt<bool>(request->prompt, request->image, bool_response);

    RCLCPP_INFO_STREAM(this->get_logger(), "Received response: " << bool_response);
    response->response = bool_response;
  }

  /**
   * @brief Callback for a StringResponse service type
   *
   * @param request service request
   * @param response string response
   */
  void stringResponseCallback(const std::shared_ptr<ai_msgs::srv::StringResponse::Request> request,
                              std::shared_ptr<ai_msgs::srv::StringResponse::Response> response)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Incoming StringResponse request: " << request->prompt);
    // No filter on the response, just pass the string through
    std::string string_response;
    bool result = sendOpenAIPrompt<std::string>(request->prompt, request->image, string_response);

    RCLCPP_INFO_STREAM(this->get_logger(), "Received response: " << string_response);
    response->response = string_response;
  }

  // Currently we provide servers for 2 types
  rclcpp::Service<ai_msgs::srv::BoolResponse>::SharedPtr bool_response_srv_;
  rclcpp::Service<ai_msgs::srv::StringResponse>::SharedPtr string_response_srv_;
  std::string openai_key_string_;
  CURL* curl_;
};
}  // namespace openai_server

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto service_client = std::make_shared<openai_server::OpenAIServer>();
  while (rclcpp::ok())
  {
    rclcpp::spin_some(service_client);
  }

  rclcpp::shutdown();
  return 0;
}
