cmake_minimum_required(VERSION 3.0.2)
project(admittance_controller_smach)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
install(PROGRAMS
   scripts/joint_publisher
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
