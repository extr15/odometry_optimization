cmake_minimum_required(VERSION 2.8.3)
project(odometry_optimization)
find_package(catkin REQUIRED COMPONENTS 
	rospy  
	tf)

catkin_package()

catkin_python_setup()