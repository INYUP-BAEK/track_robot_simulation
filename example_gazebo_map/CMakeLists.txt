cmake_minimum_required(VERSION 3.0.2)
project(example_gazebo_map)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)

catkin_package()


install(DIRECTORY launch worlds models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)