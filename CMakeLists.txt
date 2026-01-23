cmake_minimum_required(VERSION 3.16.3)
project(iss_description)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rviz_rendering REQUIRED)

# Adds shallow folders containing ogre_media such as meshes and scripts to Rviz
# so that they can be found at runtime. Requires rviz_rendering.
register_rviz_ogre_media_exports(
    DIRECTORIES
    "meshes"
  )


set(DEPS
  rclcpp
  rclpy
  geometry_msgs
  std_msgs
  std_srvs
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()


#############
## Install ##
#############

install(PROGRAMS
#  scripts/ifc_convert_geometry_test.py
#  scripts/ifc_1.py
#  scripts/ifc_create_model.py
  DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY include/
#  DESTINATION include/
#)

install(
  DIRECTORY 
  launch 
  rviz 
  meshes 
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
