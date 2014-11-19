CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(lms1_to_pc)
#ROS
#=================================================================
find_package(catkin REQUIRED
COMPONENTS roscpp rospy std_msgs message_generation tf)
#=================================================================

#===============================================================
#Qt4
#===============================================================
#FIND_PACKAGE(Qt4 REQUIRED)
#SET(QT_DONT_USE_QTGUI TRUE)
#SET(QT_USE_QTNETWORK TRUE)
#INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})
#MESSAGE(STATUS "this is QT_USE_FILE" ${QT_USE_FILE})
#MESSAGE(STATUS "this is QT_DEFINITIONS" ${QT_DEFINITIONS})
#===============================================================
#===============================================================

#set output library directory
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set output binary directory
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set include files
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include/)
#MESSAGE(STATUS "this is CMAKE_CURRENT_SOURCE_DIR" ${CMAKE_CURRENT_SOURCE_DIR})

#set full path to enable QT4_WRAP_CPP to find the headers
#SET(Adu5630_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/Adu5630.h)
#MESSAGE(STATUS "this is lms1_Headers" ${lms1_HEADERS})

#set source files
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/src PROJECT_SOURCES)
#MESSAGE(STATUS "this is lms_SOURCES" ${lms1_SOURCES})

#set qt moc
#QT4_WRAP_CPP(Adu5630_HEADERS_MOC ${Adu5630_HEADERS})


#set source files
#SET(lms1_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/lms.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/lms_node.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/Lms511Thread.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)



#rosbuild_genmsg()
ADD_EXECUTABLE(app_lms1_to_pc  ${PROJECT_SOURCES})
TARGET_LINK_LIBRARIES(app_lms1_to_pc ${catkin_LIBRARIES})
install(TARGETS app_lms1_to_pc
	ARCHIVE DESTINATION /program/catkin_ws/bin#dynamic lib, *.a;
	LIBRARY DESTINATION /program/catkin_ws/bin#static lib, *.so;
	RUNTIME DESTINATION /program/catkin_ws/bin#executable;
)
