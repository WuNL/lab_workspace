CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
project(lms1)
#set(CMAKE_BUILD_TYPE DEBUG)

#ROS
#=================================================================
find_package(catkin REQUIRED
COMPONENTS roscpp rospy std_msgs message_generation)
#=================================================================


#Qt4
#=================================================================
find_package(Qt4 REQUIRED)
set(QT_DONT_USE_QTGUI TRUE)
set(QT_USE_QTNETWORK TRUE)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
#message(STATUS "this is QT_USE_FILE " ${QT_USE_FILE})
#message(STATUS "this is QT_DEFINITIONS " ${QT_DEFINITIONS})
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

#set source files;
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES)
#=================================================================


#set qt headers wrap cpp;
#=================================================================
#set full path to enable QT4_WRAP_CPP to find the headers
SET(PROJECT_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/Lms511Thread.h)
#MESSAGE(STATUS "this is Lms511_1_Headers" ${Lms511_1_HEADERS})

#set qt moc
QT4_WRAP_CPP(PROJECT_HEADERS_MOC ${PROJECT_HEADERS})
#=================================================================


#add messages;
#===============================================================
add_message_files(FILES Lms1Status.msg)
generate_messages(DEPENDENCIES std_msgs)
#===============================================================


#message(STATUS "this is catkin_LIBRARIES " ${catkin_LIBRARIES})
#message(STATUS "this is QT_LIBRARIES " ${QT_LIBRARIES})
add_executable(app_lms1 ${PROJECT_SOURCES} ${PROJECT_HEADERS_MOC})
target_link_libraries(app_lms1 ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS app_lms1
	ARCHIVE DESTINATION /program/catkin_ws/bin#dynamic lib, *.a;
	LIBRARY DESTINATION /program/catkin_ws/bin#static lib, *.so;
	RUNTIME DESTINATION /program/catkin_ws/bin#executable;
)
