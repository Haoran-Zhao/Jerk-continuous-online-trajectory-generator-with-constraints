cmake_minimum_required(VERSION 2.8.3)
project(robot_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  moveit_ros_planning_interface
  tf2
  tf
  kdl_conversions
  tf_conversions
  actionlib_msgs
  sensor_msgs
)

add_message_files(
		FILES
		RotationAngle.msg
		ScopeTip.msg
)

generate_messages(
		DEPENDENCIES
		std_msgs
		actionlib_msgs
		geometry_msgs
		sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    geometry_msgs
    tf2
	gazebo_ros
	std_msgs
	sensor_msgs
    moveit_ros_planning_interface
)

find_package(ignition-msgs1 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ruckig REQUIRED)

find_package(ignition-transport4 REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

include_directories(${Eigen3_INCLUDE_DIRS})


include_directories(
# include
  ${catkin_INCLUDE_DIRS} src/OTG
)

add_executable(robot_controller
				src/Main.cpp
				src/GazeboMarker.cpp
				src/GazeboMarker.h
				src/GazeboModelState.cpp
        src/GazeboModelState.h
				src/UR3PathComputation.cpp
				src/UR3PathComputation.h
				src/UR3RobotController.h
				src/UR3RobotController.cpp
				src/Utilities.h
				src/Utilities.cpp
        src/Semaphore.cpp
        src/Semaphore.h
        src/RobotController.cpp
        src/RobotController.h
        src/PathComputation.cpp
        src/PathComputation.h
        src/OTG/profileSegments.cpp
        src/OTG/profileSegments.h
        src/OTG/TrigonometricOTG.cpp
        src/OTG/TrigonometricOTG.h)

target_compile_features(robot_controller PUBLIC cxx_std_17)
target_link_libraries(robot_controller ${catkin_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(robot_controller ruckig)
add_dependencies(robot_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
