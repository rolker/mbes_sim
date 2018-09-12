cmake_minimum_required(VERSION 2.8.3)
project(mbes_sim)


find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  rospy
)

catkin_python_setup()

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/mbes_sim_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
