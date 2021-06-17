set(APP_NAME "Basic")

include(SmplMacroUtils)

set(${APP_NAME}_PUBLIC_HEADERS
    basic.h
)

set(${APP_NAME}_PRIVATE_HEADERS
)

set(${APP_NAME}_HEADERS
  ${${APP_NAME}_PUBLIC_HEADERS}
  ${${APP_NAME}_PRIVATE_HEADERS}
)

set(${APP_NAME}_SRC_FILES
  basic.cpp
  main.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories("${ROS_CUSTOM_ROOT}/include")


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

add_executable(${APP_NAME}
${${APP_NAME}_SRC_FILES}
${${APP_NAME}_HEADERS}
)

SET(LINK_LIBRARY
roscpp
roscpp_serialization
rosconsole
rostime
SMPLToolBoxLib
Calculator
)

target_link_libraries(${APP_NAME} ${LINK_LIBRARY})


add_dependencies(${APP_NAME} ${LINK_LIBRARY})

if (CMAKE_DEBUG_POSTFIX)
  set_target_properties(${APP_NAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
endif (CMAKE_DEBUG_POSTFIX)
################################################################################
#
# Installation commands
#

SET(INSTALL_BINDIR bin)

if(WIN32)
    INSTALL(
        TARGETS ${APP_NAME}
        RUNTIME DESTINATION ${INSTALL_BINDIR}/Release COMPONENT bin
        CONFIGURATIONS Release
    )

    INSTALL(
        TARGETS ${APP_NAME}
        RUNTIME DESTINATION ${INSTALL_BINDIR}/RelWithDebInfo COMPONENT bin
        CONFIGURATIONS RelWithDebInfo
    )

    INSTALL(
        TARGETS ${APP_NAME}
        RUNTIME DESTINATION ${INSTALL_BINDIR}/Debug COMPONENT bin
        CONFIGURATIONS Debug
    )
elseif(UNIX)
    INSTALL(
        TARGETS ${APP_NAME}
        RUNTIME DESTINATION ${INSTALL_BINDIR} COMPONENT bin
    )
endif()
