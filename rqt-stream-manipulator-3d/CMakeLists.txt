cmake_minimum_required(VERSION 2.8.3)
project(rqt_stream_manipulator_3d)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rqt_gui
  rqt_gui_cpp
  roslib
  cmake_modules
  stream_manipulator_3d
)

## System dependencies are found with CMake's conventions
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)

find_package(Boost REQUIRED)

find_package(TinyXML REQUIRED)

include (${QT_USE_FILE})

set(rqt_sm3d_INCLUDE_DIRECTORIES
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)
set(rqt_sm3d_SRC
    src/rqt_stream_manipulator.cpp
    src/wait_for_dialog.cpp
    src/plugin.cpp
)
set(rqt_sm3d_HDR
    include/rqt_stream_manipulator_3d/rqt_stream_manipulator.h
    include/rqt_stream_manipulator_3d/wait_for_dialog.h
    include/rqt_stream_manipulator_3d/plugin.h
)
set(rqt_sm3d_UI
    src/rqt_stream_manipulator.ui
)
set(rqt_sm3d_QRC
    resources/resources.qrc)

qt4_wrap_cpp(rqt_sm3d_MOC ${rqt_sm3d_HDR})
qt4_wrap_ui(rqt_sm3d_UI_H ${rqt_sm3d_UI})
qt4_add_resources(rqt_sm3d_QRC_H ${rqt_sm3d_QRC})

# Define the plugin library
set(PLUGIN_LIBRARY rqt_sm3d_plugins)

set(PLUGIN_LIBRARY_HDR
    include/rqt_stream_manipulator_3d/filters/cropbox.h
    include/rqt_stream_manipulator_3d/output/publisher.h
    )

set(PLUGIN_LIBRARY_SRC
    src/filters/cropbox.cpp
    src/output/publisher.cpp
    )

set(PLUGIN_LIBRARY_UI
    src/filters/cropbox.ui
    src/output/publisher.ui
    )

#qt wraps for PLUGIN_LIBRARY
qt4_wrap_cpp(PLUGIN_LIBRARY_MOC ${PLUGIN_LIBRARY_HDR})
qt4_wrap_ui(PLUGIN_LIBRARY_UI_H ${PLUGIN_LIBRARY_UI})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS ${rqt_sm3d_INCLUDE_DIRECTORIES}
    LIBRARIES ${PROJECT_NAME} ${PLUGIN_LIBRARY}
    CATKIN_DEPENDS pluginlib roscpp rqt_gui rqt_gui_cpp roslib stream_manipulator_3d
#  DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${rqt_sm3d_INCLUDE_DIRECTORIES}
  ${TinyXML_INCLUDE_DIRECTORIES}
  ${Boost_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
    ${rqt_sm3d_SRC} ${rqt_sm3d_MOC} ${rqt_sm3d_UI_H} ${rqt_sm3d_QRC_H}
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY}
    ${TinyXML_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_library(${PLUGIN_LIBRARY}
    ${PLUGIN_LIBRARY_SRC} ${PLUGIN_LIBRARY_MOC} ${PLUGIN_LIBRARY_UI_H}
    ${rqt_sm3d_QRC_H}
    )
target_link_libraries(${PLUGIN_LIBRARY}
    ${catkin_LIBRARIES}
    ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY}
    ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(FILES rqt_plugin.xml plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} ${PLUGIN_LIBRARY}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
