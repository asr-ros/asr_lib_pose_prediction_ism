cmake_minimum_required(VERSION 2.8.3)
project(asr_lib_pose_prediction_ism)

# C++11 is needed
add_definitions(-Wall -std=gnu++0x -ggdb)
set(CMAKE_BUILD_TYPE Debug)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
find_package(cmake_modules)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    asr_msgs
    asr_lib_ism
)

################################################
## Declare ROS messages, services and actions ##
################################################


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp asr_msgs asr_lib_ism
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


## Declare a cpp library
# add_library(pose_prediction_ism
#   src/${PROJECT_NAME}/pose_prediction_ism.cpp
# )
FILE(GLOB_RECURSE HEADERS "include/pose_prediction_ism/*.h")
add_library(${PROJECT_NAME}
    ${HEADERS}
    src/best_path.cpp
    src/old_prediction_non_normalized.cpp
    src/old_prediction_normalized.cpp
    src/pose_predictor.cpp
    src/predictor_with_score.cpp
    src/random_path.cpp
    src/shortest_path.cpp
    src/utility.cpp

)
add_dependencies(${PROJECT_NAME}
    asr_msgs_gencpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ism
)

#add_executable(pose_prediction_test testcase/pose_prediction_tester.cpp)
#target_link_libraries(pose_prediction_test 
#  ism
#  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}
#)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pose_prediction_ism pose_prediction_ism_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )Hi[       OK ] TestSuite.testCase1 (0 ms)


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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(test_recognizer_prediction_ism
#    test/src/test_recognizer_prediction_ism.cpp
#    src/pose_prediction/RecognizerPredictionISM.cpp
#    src/pose_prediction/PaperPredictionISM.cpp
#    src/pose_prediction/PaperPredictionNormalized.cpp
#    #src/pose_prediction/PaperPredictionNonNormalized.cpp
#    src/pose_prediction/PosePredictionNode.cpp
#    src/SharedMemoryManager.cpp
#    src/Utility.cpp
#    src/SceneRecognizerService.cpp
##)
#add_dependencies(test_recognizer_prediction_ism
#    ${catkin_EXPORTED_TARGETS}
#    ${PROJECT_NAME}_generate_messages_cpp
#    ${PROJECT_NAME}_gencpp
#    ism
#)
#target_link_libraries(test_recognizer_prediction_ism
#    ${catkin_LIBRARIES}
#    ism
#)
#catkin_add_gtest(test_paper_prediction_normalized
#    test/src/test_paper_prediction_normalized.cpp
#    src/pose_prediction/RecognizerPredictionISM.cpp
#    src/pose_prediction/PaperPredictionISM.cpp
#    src/pose_prediction/PaperPredictionNormalized.cpp
#    src/Utility.cpp
#)
#target_link_libraries(test_paper_prediction_normalized
#    ${catkin_LIBRARIES}
#    ism
#)
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
