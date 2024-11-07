cmake_minimum_required(VERSION 3.5)
project(line_detection)

# ROS2와 OpenCV 관련 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 노드 cpp 파일 추가
add_executable(line_detection_node src/line_detection_node.cpp)

# 타겟에 의존성 추가
ament_target_dependencies(line_detection_node rclcpp sensor_msgs OpenCV)

install(TARGETS
  line_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
