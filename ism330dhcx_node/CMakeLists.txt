cmake_minimum_required(VERSION 3.8)
project(ism330dhcx_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

set(NODE_NAME ism330dhcx_node)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(ism330dhcx_driver REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/ism330dhcx_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  rclcpp::rclcpp
  ism330dhcx_driver::ism330dhcx_driver
  ${sensor_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater)

add_executable(${NODE_NAME}
  src/ism330dhcx_main.cpp
)

target_link_libraries(${NODE_NAME}
  ${PROJECT_NAME}_lib
)

install(TARGETS ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_ism330dhcx_node.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_lib)
endif()

ament_package()
