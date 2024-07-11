cmake_minimum_required(VERSION 3.8)
project(robosoft_openai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS cv_bridge rclcpp robosoft_ai_msgs sensor_msgs)

find_package(ament_cmake REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)

include_directories(include)

add_executable(openai_server src/openai_server.cpp)
ament_target_dependencies(openai_server ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(openai_server b64 CURL::libcurl nlohmann_json
                      ${OpenCV_LIBS})

add_executable(example_client src/example_client.cpp)
ament_target_dependencies(example_client ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(example_client ${OpenCV_LIBS})

install(
  TARGETS example_client openai_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/robosoft_openai)

install(DIRECTORY include DESTINATION include/robosoft_openai)
install(
  DIRECTORY test_data
  DESTINATION DESTINATION
  share/robosoft_openai)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
