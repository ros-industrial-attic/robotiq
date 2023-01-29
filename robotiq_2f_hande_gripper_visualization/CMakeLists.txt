cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_hande_gripper_visualization)

find_package(catkin REQUIRED COMPONENTS
  # openrave_catkin
)

catkin_package()

install(DIRECTORY launch meshes urdf DESTINATION
                                            ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Add the meshes to the OpenRAVE path
install(DIRECTORY meshes
                  DESTINATION ${OpenRAVE_DEVEL_DIR}/${OpenRAVE_DATA_DIR}/robots)
install(DIRECTORY meshes
                DESTINATION ${OpenRAVE_INSTALL_DIR}/${OpenRAVE_DATA_DIR}/robots)
