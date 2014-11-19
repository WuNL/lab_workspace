##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(pcdviewer)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp pcl_ros pcl_conversions)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS})
AUX_SOURCE_DIRECTORY(src/voxel SRC_LIST)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/pcdviewer/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(pcdviewer ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${SRC_LIST})
target_link_libraries(pcdviewer ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} QVTK libvtkCommon.so libvtkFiltering.so /usr/lib/libQVTK.so.5.8.0)
install(TARGETS pcdviewer RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

