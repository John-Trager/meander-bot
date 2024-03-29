cmake_minimum_required(VERSION 3.0.2)
project(meanderbot LANGUAGES CXX)

## Compile as C++17, supported in ROS Kinetic and newer, enable some static analysis
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(MEANDER_CPP_COMPILE_OPTIONS -Wall -Wextra -Werror -pedantic)

# ROS packages list
set(MEANDER_PACKAGES
        dynamic_reconfigure
        message_generation
        rospy
        roscpp
        rostest
        std_msgs
        sensor_msgs
        )

# Message files
set(MEANDER_MESSAGE_FILES
        CameraCmd.msg
        MotorCmd.msg
)

# Service files
set(MEANDER_SERVICE_FILES

)

# Generate messages list
set(MEANDER_ROS_MESSAGES
        sensor_msgs
        std_msgs
)

# Dynamic reconfigure parameter file list
set(MEANDER_PARAMETERS
config/params.cfg
)

# catkin packages list
set(MEANDER_CATKIN_PACKAGES
        roscpp rospy std_msgs message_runtime
        )

## CREATE CMAKE MACROS 

macro(add_tests_macro)
    # Add C++ unit tests
    #catkin_add_gtest(example-cpp-test test/example/cpp_test.cpp)

    # Python unit tests
    #catkin_add_nosetests(test/navigation/drive_test.py)
    #catkin_add_nosetests(test/teleop/teleop_test.py)
    #catkin_add_nosetests(test/util/SE3_test.py)
    #catkin_add_nosetests(test/util/SO3_test.py)

    # Integration tests (python and c++)
    find_package(rostest REQUIRED)
    #add_rostest(test/example/basic_integration_test.test)
    #add_rostest(test/integration/integration.test)
    #add_rostest(test/util/SE3_tf_test.test)
endmacro()

# launch install macro
macro(install_launch_macro)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
endmacro()


## Find catkin macros and libraries
# 3. Find Packages
find_package(
        catkin REQUIRED COMPONENTS
        ${MEANDER_PACKAGES}
)

# 4. Python module support
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)


# 5. Message Generators (add_xxx)

add_message_files(
        FILES
        ${MEANDER_MESSAGE_FILES}
)

# TODO: enable once I've created SRVs
#add_service_files(
#        FILES
#        ${MEANDER_SERVICE_FILES}
#)

# 6. Invoke messages (generate_messages)

generate_messages(
        DEPENDENCIES
        ${MEANDER_ROS_MESSAGES}
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

# TODO: uncomment if we plan to use dynamic reconfig
#generate_dynamic_reconfigure_options(
#        ${MEANDER_PARAMETERS}
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

# 7. Specify package build info export (catkin_package)

catkin_package(
        CATKIN_DEPENDS
        ${MEANDER_CATKIN_PACKAGES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/meanderbot.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/meanderbot_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# 10. Install rules
install_launch_macro()
if (COMMAND additional_install_macro)
    additional_install_macro()
endif ()

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#9. testing
if (COMMAND add_tests_macro)
    add_tests_macro()
endif ()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_meanderbot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
