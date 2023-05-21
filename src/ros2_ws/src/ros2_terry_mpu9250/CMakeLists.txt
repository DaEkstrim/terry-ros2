cmake_minimum_required(VERSION 3.8)
project(ros2_terry_mpu9250)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add internal libs folder
add_subdirectory(lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/ros2_terry_mpu9250.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs)
target_link_libraries(${PROJECT_NAME} mpu9250 terry_i2c_communicator)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME}/)
ament_package()
