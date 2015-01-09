cmake_minimum_required(VERSION 2.8.3)
project(robotiq_c2_model_visualization)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
