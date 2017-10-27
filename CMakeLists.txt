cmake_minimum_required(VERSION 2.8.3)
project(ps3controller)

find_package(catkin REQUIRED COMPONENTS
	rospy
	joy
	std_msgs
	sensor_msgs
	geometry_msgs
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
 	joy
	std_msgs
	sensor_msgs
	geometry_msgs
)
