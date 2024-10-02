cmake_minimum_required(VERSION 2.8.3)
project(vzense_demo)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  dynamic_reconfigure
  std_msgs
)

find_package(OpenCV REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")


# ------------------------------------------------------------------------------------
# Download
# ------------------------------------------------------------------------------------

# download and install trained data
add_custom_target(${PROJECT_NAME}_install_trained_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)

catkin_python_setup()

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/DrawRects.cfg
  cfg/InstanceSegmentation.cfg
)

catkin_package(
  # CATKIN_DEPENDS message_runtime
  # sensor_msgs
  # DEPENDS
  # # INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  # LIBRARIES ${PROJECT_NAME}
)
