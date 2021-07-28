cmake_minimum_required(VERSION 2.8.12)


project(pattern-matching LANGUAGES CXX)
set(APP_NAME "pattern-matching")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



IF (WIN32)
SET(ROS_ROOT "C:/opt/ros/melodic/x64" CACHE PATH "ROS Root" FORCE)
SET(ROSDEPS_ROOT "C:/opt/rosdeps/melodic/x64" CACHE PATH "ROSDEPS Root" FORCE)
SET(ROS_CUSTOM_ROOT "C:/work/ws/devel/" CACHE PATH "ROS Custom Root")
ELSEIF (UNIX)
SET(ROS_ROOT "/opt/ros/melodic/devel" CACHE PATH "ROS Root")
SET(ROS_CUSTOM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../catkin_ws/devel" CACHE PATH "ROS Custom Root")



endif (WIN32)

OPTION(USEROSSERIAL "use ros serial for serial communication" ON)

if (USEROSSERIAL)
  add_definitions("-DUSEROSSERIAL ")
endif (USEROSSERIAL)

IF (WIN32)
  include_directories("${ROS_ROOT}/include")
  include_directories("${ROSDEPS_ROOT}/include")
  link_directories("${ROS_ROOT}/lib")
  link_directories("${ROSDEPS_ROOT}/lib")
  link_directories("${ROSDEPS_ROOT}/bin")
ELSEIF (UNIX)
    include_directories("${ROS_ROOT}/include")
    include_directories("${ROS_CUSTOM_ROOT}/include")
    link_directories("${ROS_ROOT}/lib")
ENDIF (WIN32)

find_package(vehicle_msgs)





find_package(Qt5Core)
set(${APP_NAME}_HEADERS

  include/patternmatcher.h
  include/eventvector.h
  include/pattern.h
  include/newpattern.h
)

set(${APP_NAME}_SRC_FILES
      src/eventvector.cpp
  src/patternmatcher.cpp
  src/pattern.cpp
  src/newpattern.cpp

)

add_executable(pattern-matching
    ${${APP_NAME}_SRC_FILES}
    ${${APP_NAME}_HEADERS}
    basic.h
    basic.cpp
    oldmain.cpp

)
target_link_libraries(pattern-matching Qt5::Core)


target_link_libraries(pattern-matching roscpp)
target_link_libraries(pattern-matching roscpp_serialization)
target_link_libraries(pattern-matching rosconsole)
target_link_libraries(pattern-matching rostime)

