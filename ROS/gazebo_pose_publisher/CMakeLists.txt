 
cmake_minimum_required(VERSION 2.8.3)
project(gazebo_pose_publisher)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include (FindPkgConfig)

if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    geometry_msgs
)

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
find_package(Protobuf REQUIRED)

 add_message_files(
   FILES
   custom_pose.msg
 )
generate_messages(DEPENDENCIES std_msgs geometry_msgs )

catkin_package(
    CATKIN_DEPENDS gazebo_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)
