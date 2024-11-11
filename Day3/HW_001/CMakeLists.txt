cmake_minimum_required(VERSION 3.8)
project(contour_extraction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

# add executable
add_executable(contour_extraction_node src/contour_extraction.cpp)

# link libraries
target_link_libraries(contour_extraction_node
  ${OpenCV_LIBS}
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
)

# install the executable
install(TARGETS contour_extraction_node
  DESTINATION lib/${PROJECT_NAME}
)

# install image files to the share directory
install(DIRECTORY include/contour_extraction
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
