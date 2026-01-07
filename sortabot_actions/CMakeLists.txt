cmake_minimum_required(VERSION 3.8)
project(sortabot_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoTo.action"
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  scripts/action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
