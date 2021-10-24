cmake_minimum_required(VERSION 3.0.2)
project(ur5_kdl_solver)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
install(PROGRAMS
   scripts/controller
   scripts/force_generator
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
