cmake_minimum_required(VERSION 3.5.1)
project(ximea_ros_cam)

## gnu++11 feature
set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Set ximea environment variables
set(XIMEA_INCLUDE_PATH /usr/include)
set(XIMEA_LIBRARY m3api)

## Find catkin macros and libraries
set(catkin_deps
    roscpp
    roslib
    nodelet
    sensor_msgs
    image_transport
    camera_info_manager
    std_msgs
    message_generation
    diagnostic_updater
)

find_package(catkin REQUIRED ${catkin_deps})
## Change to OpenCV 3 if necessary
# find_package(OpenCV 3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

# Declare ROS messages and services
add_message_files(DIRECTORY msg FILES
  XiImageInfo.msg)

# Generate messages
generate_messages(DEPENDENCIES std_msgs)

catkin_package(INCLUDE_DIRS   include
               CATKIN_DEPENDS ${catkin_deps}
               DEPENDS        Boost
               LIBRARIES      ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    ${XIMEA_INCLUDE_PATH}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
            src/ximea_ros_cam.cpp
)

## Link the catkin libraries
target_link_libraries(${PROJECT_NAME}
    ${XIMEA_LIBRARY}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    yaml-cpp
)

## Package depends on its generated messages
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/ximea_ros_cam_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
                      ${XIMEA_LIBRARY}
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      yaml-cpp
                      ${PROJECT_NAME}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Need to explicitly list nodelets.xml for installation
install(FILES nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# #############
# ## Testing ##
# #############
#
# ## Add gtest based cpp test target and link libraries
# if (CATKIN_ENABLE_TESTING)
#   file(GLOB TEST_FILES "test/*.cpp")
#   catkin_add_gtest(${PROJECT_NAME}-test ${TEST_FILES})
#   target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
# endif(CATKIN_ENABLE_TESTING)
