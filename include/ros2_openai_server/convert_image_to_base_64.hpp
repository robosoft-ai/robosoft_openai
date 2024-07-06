#pragma once

#include <b64/encode.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>

namespace openai_server
{
/**
 * @brief Convert jpeg to base64 encoding. There are 2 possible input types.
 *
 * @param image_path path to image on disk
 * @param image_cv_mat an image
 * @return image encoded as string. If there was an error, it will be empty.
 */
std::string convert_image_to_base_64(const std::optional<const std::string>& image_path,
                                     const std::optional<const cv::Mat>& image_cv_mat)
{
  if (!image_path && !image_cv_mat)
  {
    std::cerr << "Both inputs to convert_image_to_base_64() are nullopt. One valid input is required." << std::endl;
    return "";
  }

  cv::Mat image;
  if (image_path)
  {
    image = cv::imread(*image_path, cv::IMREAD_COLOR);
    if (image.empty())
    {
      std::cerr << "Could not read the image: " << *image_path << std::endl;
      return "";
    }
  }
  else
  {
    image = image_cv_mat.value();
    if (image.empty())
    {
      std::cerr << "Passed image was empty" << std::endl;
      return "";
    }
  }

  // Encode image to JPEG format
  std::vector<uchar> buf;
  cv::imencode(".jpg", image, buf);

  // Convert the buffer to a char array
  std::string encoded_data(buf.begin(), buf.end());

  // Encode to Base64 using libb64
  base64::encoder E;
  std::ostringstream os;
  std::istringstream is(encoded_data);
  E.encode(is, os);

  return os.str();
}
}  // namespace openai_server
