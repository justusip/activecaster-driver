cmake_minimum_required(VERSION 3.0.2)
project(cheetah_driver)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs)

add_message_files(FILES VescStatus.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})
message("${catkin_INCLUDE_DIRS}")
add_executable(cheetah_driver src/main.cpp src/traffic/hid.cpp src/models/vesc_status.cpp src/models/vesc_output.cpp)
add_dependencies(cheetah_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cheetah_driver ${catkin_LIBRARIES})