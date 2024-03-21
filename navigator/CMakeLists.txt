cmake_minimum_required(VERSION 3.8)
project(navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
    rclcpp
    irobot_create_msgs
    visualization_msgs
    geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/navigator/NavigatorNode.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(navigation src/navigator_main.cpp)
ament_target_dependencies(navigation ${dependencies})

target_link_libraries(navigation ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  navigation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
