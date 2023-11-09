cmake_minimum_required(VERSION 3.8)
project(ros2_worm_multiplayer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-sign-compare)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Curses REQUIRED)
find_package(std_msgs REQUIRED) # only needed for testing purposes - mario
find_package(rosidl_default_generators REQUIRED) # testing - Joschie
find_package(ros2_worm_multiplayer REQUIRED)

# Display Node
add_executable(worm_display_node
	src/worm_display_node.cpp
  src/prep.c
	src/messages.c
	src/board_model.c
	src/worm_model.c
	src/worm.c
)
ament_target_dependencies(worm_display_node rclcpp std_msgs)

# Navigation Node
add_executable(worm_navigation_node
  src/worm_navigation_node.cpp
)
ament_target_dependencies(worm_navigation_node rclcpp ros2_worm_multiplayer)

# Messages test - Joshi
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Direction.msg"
)

#######################################
target_include_directories(worm_display_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 
install(TARGETS 
  worm_display_node
  worm_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(worm_display_node ncurses)

ament_package()