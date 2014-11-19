cmake_minimum_required(VERSION 2.8.3)
project(transformation)
#set(CMAKE_BUILD_TYPE DEBUG)

#ROS
#=================================================================
find_package(catkin REQUIRED
COMPONENTS roscpp rospy std_msgs tf)
#=================================================================

#cmake settings
#=================================================================
#set output library directory;
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#message(STATUS "this is LIBRARY_OUTPUT_PATH " ${LIBRARY_OUTPUT_PATH})

#set output binary directory;
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#message(STATUS "this is EXECUTABLE_OUTPUT_PATH " ${EXECUTABLE_OUTPUT_PATH})

#set runtime output directory;
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#message(STATUS "this is CMAKE_RUNTIME_OUTPUT_DIRECTORY " ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

#set include files;
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
#message(STATUS "this is CMAKE_CURRENT_SOURCE_DIR " ${CMAKE_CURRENT_SOURCE_DIR})

#add messages;
#===============================================================
add_message_files(FILES VehiclePose.msg)
generate_messages(DEPENDENCIES std_msgs)
#===============================================================

#set source files;
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES)
#=================================================================

#message(STATUS "this is catkin_LIBRARIES " ${catkin_LIBRARIES})
#message(STATUS "this is QT_LIBRARIES " ${QT_LIBRARIES})
add_executable(app_transformation ${PROJECT_SOURCES})
target_link_libraries(app_transformation ${catkin_LIBRARIES})
install(TARGETS app_transformation
	ARCHIVE DESTINATION /program/catkin_ws/bin#dynamic lib, *.a;
	LIBRARY DESTINATION /program/catkin_ws/bin#static lib, *.so;
	RUNTIME DESTINATION /program/catkin_ws/bin#executable;
)
