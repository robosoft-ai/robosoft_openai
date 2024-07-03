cmake_minimum_required(VERSION 3.8)
project(ros2_openai_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS ai_msgs cv_bridge rclcpp sensor_msgs)

find_package(ament_cmake REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)

add_executable(openai_server src/openai_server.cpp)
ament_target_dependencies(openai_server ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(openai_server b64 CURL::libcurl nlohmann_json
                      ${OpenCV_LIBS})

install(
  TARGETS openai_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/ros2_openai_server)

install(
  DIRECTORY test_data
  DESTINATION DESTINATION
  share/ros2_openai_server)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
