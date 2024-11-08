cmake_minimum_required(VERSION 3.5)
project(ball_detection)

# ROS2 및 OpenCV 패키지 의존성 설정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# OpenCV include 디렉토리 설정
include_directories(${OpenCV_INCLUDE_DIRS})

# 실행 파일 추가
add_executable(ball_detection_node src/ball_detection_node.cpp)
ament_target_dependencies(ball_detection_node rclcpp sensor_msgs)
target_link_libraries(ball_detection_node ${OpenCV_LIBRARIES})

# 설치 설정
install(TARGETS
  ball_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# ament 매크로 호출
ament_package()
