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

add_executable(worm_test
	src/worm_test.cpp
	src/prep.c
	src/messages.c
	src/board_model.c
	src/worm_model.c
	src/worm.c
)

#add_executable(worm_display_node
#  src/worm_display_node.cpp
#)
#
#add_executable(worm_grid_node
#  src/worm_grid_node.cpp
#)
#
#add_executable(worm_navigation_node
#  src/worm_navigation_node.cpp
#)

target_include_directories(worm_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(worm_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  worm_test
  "rclcpp"
  Curses
)

install(TARGETS 
  worm_test
#  worm_display_node
#  worm_navigation_node
#  worm_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(worm_test ncurses)

ament_package()
