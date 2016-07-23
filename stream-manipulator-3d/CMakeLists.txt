# Software License Agreement (BSD License)
#
#   Stream Manipulator 3d - https://github.com/3DVision-Stack/stream-manipulator-3D
#   Copyright (c) 2016, Federico Spinelli (fspinelli@gmail.com)
#   All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder(s) nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Project top level cmake
cmake_minimum_required(VERSION 2.8.11)
project(stream_manipulator_3d)

################################################################################
########## Lets find everything we need  #######################################
################################################################################
##Find Catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pluginlib
  pcl_conversions
  roscpp
  roslib
  sensor_msgs
  visualization_msgs
  cmake_modules
  )
#Find Boost
find_package(Boost REQUIRED)
#Find PCL
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
##Eigen
find_package(Eigen REQUIRED)
#include a bunch of dirs to search path
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    )
include_directories(SYSTEM
                    ${Eigen_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${PCL_INCLUDE_DIRS}
                    )


################################################################################
####### Set the project sources ################################################
################################################################################
set(SOURCES
    src/common_ros.cpp
    src/main.cpp
    src/stream_manipulator.cpp
    )
################################################################################
########### Start doing things. Add all filters ################################
################################################################################
add_library(filter_plugins
    src/filters/cropbox.cpp
    include/stream_manipulator_3d/filters/cropbox.hpp
    src/filters/voxel_grid.cpp
    include/stream_manipulator_3d/filters/voxel_grid.hpp
    )
target_link_libraries(filter_plugins ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} rt)

################################################################################
########### Add Outputs ########################################################
################################################################################
add_library(output_plugins
    src/output/publisher.cpp
    include/stream_manipulator_3d/output/publisher.hpp
    )
target_link_libraries(output_plugins ${catkin_LIBRARIES} ${Boost_LIBRARIES} rt)
################################################################################
#### Export project and libraries through catkin ###############################
################################################################################
##Catkin specific configuration:
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    filter_plugins
    output_plugins
  CATKIN_DEPENDS
    geometry_msgs
    pcl_conversions
    pluginlib
    roscpp
    roslib
    sensor_msgs
    visualization_msgs
    cmake_modules
    message_runtime
  DEPENDS
    Boost
    Eigen
    PCL
)
################################################################################
####################### Now Build the project ##################################
################################################################################

add_executable(stream_manipulator_3d ${SOURCES})

#add those ros geneated headers
#add_dependencies(stream_manipulator_3d
#  stream_manipulator_3d_generate_messages_cpp
#  )

target_link_libraries(stream_manipulator_3d
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen_LIBRARIES}
    rt
)
################################################################################
######################### Install ##############################################
################################################################################

install(TARGETS stream_manipulator_3d filter_plugins output_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/stream_manipulator_3d/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES plugins.xml sm3d.rviz default_config.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
########################################################################## DONE!
