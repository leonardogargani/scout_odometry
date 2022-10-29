cmake_minimum_required(VERSION 3.0.2)
project(project1)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	nav_msgs
	message_generation
	message_filters
	tf2
	tf2_ros
	dynamic_reconfigure
)


add_message_files(
	FILES
	CustomMessage.msg
)

add_service_files(
	FILES
	ResetPoseService.srv
	SetPoseService.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	nav_msgs
)

generate_dynamic_reconfigure_options(
	cfg/integration.cfg
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime geometry_msgs message_filters nav_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(velocity_publisher src/velocity_publisher.cpp)
add_executable(setup_calibration src/setup_calibration.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(odom_to_baselink_tf src/odom_to_baselink_tf.cpp)
add_executable(reset_pose_client src/reset_pose_client.cpp)
add_executable(set_pose_client src/set_pose_client.cpp)

add_dependencies(velocity_publisher project1_generate_messages_cpp)
add_dependencies(setup_calibration project1_generate_messages_cpp)
add_dependencies(odometry project1_gencfg)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS})
add_dependencies(reset_pose_client ${catkin_EXPORTED_TARGETS})
add_dependencies(set_pose_client ${catkin_EXPORTED_TARGETS})

target_link_libraries(velocity_publisher
	${catkin_LIBRARIES}
)

target_link_libraries(setup_calibration
	${catkin_LIBRARIES}
)

target_link_libraries(odometry
	${catkin_LIBRARIES}
)

target_link_libraries(odom_to_baselink_tf
	${catkin_LIBRARIES}
)

target_link_libraries(reset_pose_client
	${catkin_LIBRARIES}
)

target_link_libraries(set_pose_client
	${catkin_LIBRARIES}
)

