cmake_minimum_required(VERSION 3.8)
project(ws281x)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

option(ROS_WS2811_MOCK_LIB "Use mock ws281x library instead of the real one" OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(led_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGamma.srv"
 )

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
# )

add_subdirectory(vendor)

# ## Declare a C++ library
# # add_library(${PROJECT_NAME}
# #   src/${PROJECT_NAME}/ros_ws281x.cpp
# # )

# ## Add cmake target dependencies of the library
# ## as an example, code may need to be generated before libraries
# ## either from message generation or dynamic reconfigure
# # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Declare a C++ executable
# ## With catkin_make all packages are built within a single CMake context
# ## The recommended prefix ensures that target names across packages don't collide

add_executable(${PROJECT_NAME}_node src/ws281x_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp led_msgs)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# The following line is necessary to link the SetGamma service file from the current package
rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

#install(TARGETS
#    ${PROJECT_NAME}_node
#    DESTINATION lib/${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_node rpi_ws281x)

ament_package()
