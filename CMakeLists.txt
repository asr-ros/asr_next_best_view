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
  pbd_msgs
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
)

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
   GetMovementCosts.srv
   GetDistance.srv
   CalculateRobotState.srv
   CalculateCameraPose.srv
   CalculateCameraPoseCorrection.srv
   IsPositionAllowed.srv
   IsPositionReachable.srv
   TriggerFrustumVisualization.srv
   ResetCalculator.srv
   GetPose.srv
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
        pbd_msgs
        visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_helper
  CATKIN_DEPENDS roscpp roslib message_runtime
  DEPENDS system_lib
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp library

add_library(
        ${PROJECT_NAME}_helper
        src/helper/MathHelper.cpp
        src/helper/MarkerHelper.cpp
        src/helper/TypeHelper.cpp
        src/helper/DebugHelper.cpp
)

add_library(
        ${PROJECT_NAME}_packages
        src/common/CommonClass.cpp
        src/hypothesis_updater/HypothesisUpdater.cpp
        src/camera_model_filter/CameraModelFilter.cpp
        src/rating/BaseScoreContainer.cpp
        src/rating/RatingModule.cpp
        src/rating/IKRatingModule.cpp
        src/robot_model/RobotModel.cpp
        src/robot_model/RobotState.cpp
        src/space_sampler/SpaceSampler.cpp
        src/unit_sphere_sampler/UnitSphereSampler.cpp
)

add_library(
        ${PROJECT_NAME}_packages_impl
        src/hypothesis_updater/impl/DefaultHypothesisUpdater.cpp
        src/hypothesis_updater/impl/PerspectiveHypothesisUpdater.cpp
        src/camera_model_filter/impl/SingleCameraModelFilter.cpp
        src/camera_model_filter/impl/StereoCameraModelFilter.cpp
        src/camera_model_filter/impl/Raytracing2DBasedSingleCameraModelFilter.cpp
        src/camera_model_filter/impl/Raytracing2DBasedStereoCameraModelFilter.cpp
        src/rating/impl/DefaultRatingModule.cpp
        src/rating/impl/NavigationPathIKRatingModule.cpp
        src/rating/impl/SimpleIKRatingModule.cpp
        src/rating/impl/AngleApproximationIKRatingModule.cpp
        src/rating/impl/DefaultScoreContainer.cpp
        src/robot_model/impl/MILDRobotModel.cpp
        src/robot_model/impl/MILDRobotModelWithExactIK.cpp
        src/robot_model/impl/MILDRobotModelWithApproximatedIK.cpp
        src/robot_model/impl/MILDRobotState.cpp
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
)

target_link_libraries(
        ${PROJECT_NAME}
        ${PROJECT_NAME}_helper
        ${PROJECT_NAME}_packages
        ${PROJECT_NAME}_packages_impl
        ${PCL_LIBRARIES}
        glpk
        ${catkin_LIBRARIES}
)

#TEST CASES
#SET VARIABLES TO ENABLE/DISABLE TEST BUILDING
set (BUILD_TEST FALSE) #ToDo: Change Name
set (BUILD_PARAMETERS_TEST FALSE)
set (BUILD_IK_TEST FALSE)
set (BUILD_SINGLE_SCENE_TEST FALSE)
set (BUILD_MULTI_ID_SCENE_TEST FALSE)
set (BUILD_PERFORMANCE_TEST FALSE)
set (BUILD_UPDATE_POINTCLOUD_TEST FALSE)
set (BUILD_MATH_TEST FALSE)
set (BUILD_STREAM_TEST FALSE)

set (TEST_LINK_LIBRARIES 
        ${PROJECT_NAME}_test_core
        ${PROJECT_NAME}_helper
        ${PROJECT_NAME}_packages
        ${PROJECT_NAME}_packages_impl
        ${PCL_LIBRARIES}
        glpk
        ${catkin_LIBRARIES}
)

if (BUILD_TEST OR BUILD_PARAMETERS_TEST OR BUILD_IK_TEST OR BUILD_SINGLE_SCENE_TEST OR  BUILD_MULTI_ID_SCENE_TEST OR BUILD_PERFORMANCE_TEST OR BUILD_UPDATE_POINTCLOUD_TEST OR BUILD_MATH_TEST OR BUILD_STREAM_TEST)
MESSAGE ("Building Tests:")
add_library(
        ${PROJECT_NAME}_test_core
        src/test_cases/BaseTest.cpp
)

target_link_libraries(
        ${TEST_LINK_LIBRARIES}
)
endif ()

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


if (BUILD_SINGLE_SCENE_TEST)
MESSAGE (" - Single Scene Test")
add_executable(
        ${PROJECT_NAME}_single_scene_test
        src/test_cases/SingleSceneTest.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_single_scene_test
        ${TEST_LINK_LIBRARIES}
)
endif ()


if (BUILD_MULTI_ID_SCENE_TEST)
MESSAGE (" - Multi ID Scene Test")
add_executable(
        ${PROJECT_NAME}_multi_id_scene_test
        src/test_cases/MultiIdSceneTest.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_multi_id_scene_test
        ${TEST_LINK_LIBRARIES}
)
endif ()


if (BUILD_PARAMETERS_TEST)
MESSAGE (" - Parameter Test")
add_executable(
        ${PROJECT_NAME}_parameters_test
        src/test_cases/ParametersTest.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_parameters_test
        ${TEST_LINK_LIBRARIES}
)
endif ()

if (BUILD_IK_TEST)
MESSAGE (" - IK Test")
add_executable(
        ${PROJECT_NAME}_ik_test
        src/test_cases/IKTest.cpp
)

target_link_libraries(
       ${PROJECT_NAME}_ik_test
        ${TEST_LINK_LIBRARIES}
)
endif ()


if (BUILD_PERFORMANCE_TEST)
MESSAGE (" - Performance Test")
add_executable(
         ${PROJECT_NAME}_performance_test
        src/test_cases/PerformanceTest.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_performance_test
        ${TEST_LINK_LIBRARIES}
)
endif ()


if (BUILD_UPDATE_POINTCLOUD_TEST)
MESSAGE (" - UpdatePointcloud Test")
add_executable(
        ${PROJECT_NAME}_update_pointcloud_test
        src/test_cases/UpdatePointcloudTest.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_update_pointcloud_test
        ${TEST_LINK_LIBRARIES}
)
endif ()


if (BUILD_MATH_TEST)
MESSAGE (" - Math Test")
add_executable(
        ${PROJECT_NAME}_math_test
        src/test_cases/MathTest.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_math_test
        ${TEST_LINK_LIBRARIES}
)
endif ()

if (BUILD_STREAM_TEST)
MESSAGE (" - Stream Test")
add_executable(
        ${PROJECT_NAME}_stream_test
        src/test_cases/StreamTest.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_stream_test
        ${TEST_LINK_LIBRARIES}
)
endif ()


add_executable(
        ${PROJECT_NAME}_robot_model
        src/robot_model/RobotModel_Service.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_robot_model
        ${PCL_LIBRARIES}
        ${PROJECT_NAME}_packages_impl
        glpk
        ${catkin_LIBRARIES}
)





