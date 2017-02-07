cmake_minimum_required(VERSION 2.8.3)
project(next_best_view)

add_definitions(-std=gnu++0x -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  object_database
  message_generation
  sensor_msgs
  nav_msgs
  asr_msgs
  world_model
  actionlib
  tf
  eigen_conversions
  tf_conversions
  std_srvs
  visualization_msgs
  urdf
  kdl_parser
  costmap_2d
  rapidxml
  dynamic_reconfigure
  voxel_grid
  robot_model_services
  # temporary workaround as assimp does not provide a .cmake file but just a .pc file
  #  assimp
)

message(WARNING "Object_database needs the packages 'assimp' which does not provide .cmake files. Install them manually.")

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(rsc/DynamicParameters.cfg)

add_message_files(
        DIRECTORY
        msg
        FILES
        RobotStateMessage.msg
	RatedViewport.msg
	NormalsInfo.msg
)


## Generate services in the 'srv' folder
add_service_files(
   DIRECTORY
   srv
   FILES
   GetNextBestView.srv
   GetAttributedPointCloud.srv
   SetAttributedPointCloud.srv
   SetInitRobotState.srv
   UpdatePointCloud.srv
   TriggerFrustumVisualization.srv
   TriggerFrustumsAndPointCloudVisualization.srv
   ResetCalculator.srv
   RateViewports.srv
   GetActiveNormals.srv
   RemoveObjects.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
        geometry_msgs
        sensor_msgs
        asr_msgs
        visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_helper 
  CATKIN_DEPENDS roscpp roslib message_runtime voxel_grid robot_model_services
  DEPENDS system_lib
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  # temporary workaround as assimp does not provide a .cmake file but just a .pc file
  /opt/ros/$ENV{ROS_DISTRO}/include/assimp_devel
)


## Declare a cpp library

add_library(
        ${PROJECT_NAME}_helper
        src/helper/MathHelper.cpp
        src/helper/MarkerHelper.cpp
        src/helper/TypeHelper.cpp
        src/helper/DebugHelper.cpp
        src/helper/WorldHelper.cpp
        src/helper/VoxelGridHelper.cpp
)

add_library(
        ${PROJECT_NAME}_packages
        src/common/CommonClass.cpp
        src/hypothesis_updater/HypothesisUpdater.cpp
        src/camera_model_filter/CameraModelFilter.cpp
        src/rating/BaseScoreContainer.cpp
        src/rating/RatingModule.cpp
        src/space_sampler/SpaceSampler.cpp
        src/unit_sphere_sampler/UnitSphereSampler.cpp
)

add_library(
        ${PROJECT_NAME}_packages_impl
        src/hypothesis_updater/impl/DefaultHypothesisUpdater.cpp
        src/hypothesis_updater/impl/PerspectiveHypothesisUpdater.cpp
        src/camera_model_filter/impl/SingleCameraModelFilter.cpp
        src/camera_model_filter/impl/StereoCameraModelFilter.cpp
        src/camera_model_filter/impl/Raytracing3DBasedSingleCameraModelFilter.cpp
        src/camera_model_filter/impl/Raytracing3DBasedStereoCameraModelFilter.cpp
        src/rating/impl/DefaultRatingModule.cpp
        src/rating/impl/DefaultScoreContainer.cpp
        src/space_sampler/impl/PlaneSubSpaceSampler.cpp
        src/space_sampler/impl/Raytracing2DBasedSpaceSampler.cpp
        src/space_sampler/impl/MapBasedHexagonSpaceSampler.cpp
        src/space_sampler/impl/MapBasedRandomSpaceSampler.cpp
        src/unit_sphere_sampler/impl/SpiralApproxUnitSphereSampler.cpp
        src/crop_box/CropBoxFilter.cpp
        src/crop_box/CropBoxWrapper.cpp
        src/pcl/DefaultViewportPoint.cpp
        src/pcl/RealObjectPoint.cpp
        src/pcl/SpaceSamplePoint.cpp
)
target_link_libraries(
        ${PROJECT_NAME}_packages_impl
        ${PROJECT_NAME}_helper
        ${PROJECT_NAME}_packages
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

target_link_libraries(
        ${PROJECT_NAME}_helper
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        # assimp
        /opt/ros/$ENV{ROS_DISTRO}/lib/libassimp_devel.so
)

FILE(GLOB_RECURSE HEADERS "include/*.hpp"  "include/*.h")
## Declare a cpp executable
add_executable(
        ${PROJECT_NAME}
        ${HEADERS}
        src/next_best_view.cpp
)

add_dependencies(
        ${PROJECT_NAME}
        ${PROJECT_NAME}_generate_messages_cpp
        object_database
        world_model
	    robot_model_services
)

target_link_libraries(
        ${PROJECT_NAME}
        ${PROJECT_NAME}_helper
        ${PROJECT_NAME}_packages
        ${PROJECT_NAME}_packages_impl
        ${PCL_LIBRARIES}
        glpk
		robot_model_services_packages
		robot_model_services_packages_impl
        ${catkin_LIBRARIES}
)

#TEST CASES
#SET VARIABLES TO ENABLE/DISABLE TEST BUILDING

set(BUILD_ALL_TESTS FALSE)

set (BUILD_TEST ${BUILD_ALL_TESTS}) #ToDo: Change Name
set (BUILD_PARAMETERS_TEST ${BUILD_ALL_TESTS})
set (BUILD_SINGLE_SCENE_TEST ${BUILD_ALL_TESTS})
set (BUILD_MULTI_ID_SCENE_TEST ${BUILD_ALL_TESTS})
set (BUILD_PERFORMANCE_TEST ${BUILD_ALL_TESTS})
set (BUILD_UPDATE_POINTCLOUD_TEST ${BUILD_ALL_TESTS})
set (BUILD_MATH_TEST ${BUILD_ALL_TESTS})
set (BUILD_STREAM_TEST ${BUILD_ALL_TESTS})
set (BUILD_RAYTRACING_TEST ${BUILD_ALL_TESTS})
set (BUILD_RATE_VIEWPORTS_TEST ${BUILD_ALL_TESTS})

set (TEST_LINK_LIBRARIES 
        ${PROJECT_NAME}_test_core
        ${PROJECT_NAME}_helper
        ${PROJECT_NAME}_packages
        ${PROJECT_NAME}_packages_impl
        ${PCL_LIBRARIES}
        glpk
		robot_model_services_packages
		robot_model_services_packages_impl
        ${catkin_LIBRARIES}
)

if (BUILD_TEST OR BUILD_PARAMETERS_TEST OR  BUILD_SINGLE_SCENE_TEST OR  BUILD_MULTI_ID_SCENE_TEST OR BUILD_PERFORMANCE_TEST OR BUILD_UPDATE_POINTCLOUD_TEST OR BUILD_MATH_TEST OR BUILD_STREAM_TEST OR BUILD_RATE_VIEWPORTS_TEST OR BUILD_RAYTRACING_TEST)
MESSAGE ("Building Tests:")
add_library(
        ${PROJECT_NAME}_test_core
        src/test_cases/BaseTest.cpp
)

target_link_libraries(
        ${TEST_LINK_LIBRARIES}
)
endif ()

function(addTest TEST_NAME)
  # TEST_NAME_LIST contains each single word in uppercase
  string(REPLACE "_" ";" TEST_NAME_LIST ${TEST_NAME})
  
  # convert TEST_NAME to lower case and camel case
  set(CAMEL_CASE "")
  set(LOWER_UNDERSCORE "")
  set(UPPER_UNDERSCORE "")
  
  FOREACH(WORD ${TEST_NAME_LIST})
    # lower case
    string(TOLOWER ${WORD} BFR)
    set(LOWER_UNDERSCORE "${LOWER_UNDERSCORE}_${BFR}")
    
    # upper case
    string(TOUPPER ${WORD} BFR)
    set(UPPER_UNDERSCORE "${UPPER_UNDERSCORE}_${BFR}")
    
    # camel case
    string(SUBSTRING ${WORD} 1 -1 REST)
    string(SUBSTRING ${WORD} 0  1 FIRST_CHAR)
    string(TOLOWER ${REST} REST)
    string(TOUPPER ${FIRST_CHAR} FIRST_CHAR)
    set(CAMEL_CASE "${CAMEL_CASE}${FIRST_CHAR}${REST}")
  ENDFOREACH()
  # remove first char (_)
  string(SUBSTRING ${LOWER_UNDERSCORE} 1 -1 LOWER_UNDERSCORE)
  string(SUBSTRING ${UPPER_UNDERSCORE} 1 -1 UPPER_UNDERSCORE)
  
  set(TEST_FILE "src/test_cases/${CAMEL_CASE}Test.cpp")
  set(TEST_NAME "_${LOWER_UNDERSCORE}_test")
  set(TEST_MESSAGE " - ${CAMEL_CASE} Test")

  if (${BUILD_${UPPER_UNDERSCORE}_TEST})
  
    MESSAGE(${TEST_MESSAGE})
    
    add_executable(
	    ${PROJECT_NAME}${TEST_NAME}
	    ${TEST_FILE}
    )

    target_link_libraries(
	    ${PROJECT_NAME}${TEST_NAME}
	    ${TEST_LINK_LIBRARIES}
    )
  endif()
endfunction()

#ToDo: Change Name
if (BUILD_TEST)
MESSAGE (" - Test")
add_executable(
        ${PROJECT_NAME}_test
        src/test_cases/Test.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_test
        ${TEST_LINK_LIBRARIES}
)
endif ()

addTest(SINGLE_SCENE)

addTest(MULTI_ID_SCENE)

addTest(PARAMETERS)

addTest(PERFORMANCE)

addTest(UPDATE_POINTCLOUD)

addTest(MATH)

addTest(STREAM)

addTest(RAYTRACING)

addTest(RATE_VIEWPORTS)


