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

# Harmonic (default)
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

gz_find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  
find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})  

find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

#############
# Msgs      #
#############
# Define a variable 'MSGS_PROTOS' listing the .proto files
set(MSGS_PROTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/custom_msgs/dynamic_detachable_joint.proto
)
gz_msgs_generate_messages(
  TARGET gz_custom_msgs
  PROTO_PACKAGE "gz.custom_msgs"
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
  MSGS_PROTOS ${MSGS_PROTOS}
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

############
# Plugins  #
############
add_library(DynamicDetachableJoint SHARED
  plugins/DynamicDetachableJoint.cc
)

target_link_libraries(DynamicDetachableJoint PRIVATE 
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}  
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  ${PROJECT_NAME}-gz_custom_msgs
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/move_mss
  scripts/move_sun.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
#  gz_custom_msgs
  DynamicDetachableJoint
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)  

#install(DIRECTORY include/
#  DESTINATION include/
#)

install(
  DIRECTORY
  launch rviz meshes robots urdf config worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
