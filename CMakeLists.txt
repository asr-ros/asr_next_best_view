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
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

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
   SetupVisualization.srv
   GetNextBestView.srv
   GetPointCloud2.srv
   GetAttributedPointCloud.srv
   SetAttributedPointCloud.srv
   SetInitRobotState.srv
   GetSpaceSampling.srv
   UpdatePointCloud.srv
   GetMovementCosts.srv
   GetDistance.srv
   CalculateRobotState.srv
   CalculateCameraPose.srv
   CalculateCameraPoseCorrection.srv
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
        src/rating/impl/DefaultIKRatingModule.cpp
        src/rating/impl/SimpleIKRatingModule.cpp
	src/rating/impl/DefaultScoreContainer.cpp
	src/robot_model/impl/MILDRobotModel.cpp
        src/robot_model/impl/MILDRobotModel_with_IK.cpp
	src/robot_model/impl/MILDRobotState.cpp
	src/space_sampler/impl/PlaneSubSpaceSampler.cpp
        src/space_sampler/impl/Raytracing2DBasedSpaceSampler.cpp
	src/space_sampler/impl/MapBasedHexagonSpaceSampler.cpp
        src/space_sampler/impl/MapBasedRandomSpaceSampler.cpp
	src/unit_sphere_sampler/impl/SpiralApproxUnitSphereSampler.cpp
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

#add_library(
#        ${PROJECT_NAME}_test_core
#        src/test_cases/BaseTest.cpp
#)

#target_link_libraries(
#        ${PROJECT_NAME}_test_core
#        ${PROJECT_NAME}_helper
#        ${PROJECT_NAME}_packages
#        ${PROJECT_NAME}_packages_impl
#        ${PCL_LIBRARIES}
#        glpk
#        ${catkin_LIBRARIES}
#)

#add_executable(
#        ${PROJECT_NAME}_test
#        src/test_cases/Test.cpp
#)

#add_executable(
#        ${PROJECT_NAME}_performance_test
#        src/test_cases/PerformanceTest.cpp
#)

#add_executable(
#        ${PROJECT_NAME}_single_scene_test
#        src/test_cases/SingleSceneTest.cpp
#)


#add_executable(
#        ${PROJECT_NAME}_multi_id_scene_test
#        src/test_cases/MultiIdSceneTest.cpp
#)

#add_executable(
#        ${PROJECT_NAME}_parameters_test
#        src/test_cases/ParametersTest.cpp
#)

#add_executable(
#        ${PROJECT_NAME}_ik_test
#        src/test_cases/IKTest.cpp
#        src/robot_model/RobotModel.cpp
#        src/robot_model/impl/MILDRobotModel_with_IK.cpp
#        src/robot_model/impl/MILDRobotState.cpp
#)

#target_link_libraries(
#        ${PROJECT_NAME}_test
#        ${PROJECT_NAME}_helper
#        ${PROJECT_NAME}_packages
#        ${PROJECT_NAME}_packages_impl
#        ${PROJECT_NAME}_test_core
#        ${PCL_LIBRARIES}
#        glpk
#        ${catkin_LIBRARIES}
#)

#target_link_libraries(
#        ${PROJECT_NAME}_single_scene_test
#        ${PROJECT_NAME}_helper
#        ${PROJECT_NAME}_packages
#        ${PROJECT_NAME}_packages_impl
#        ${PROJECT_NAME}_test_core
#        ${PCL_LIBRARIES}
#        glpk
#        ${catkin_LIBRARIES}
#)

#target_link_libraries(
#        ${PROJECT_NAME}_performance_test
#        ${PROJECT_NAME}_helper
#        ${PROJECT_NAME}_packages
#        ${PROJECT_NAME}_packages_impl
#        ${PROJECT_NAME}_test_core
#        ${PCL_LIBRARIES}
#        glpk
#        ${catkin_LIBRARIES}
#)

#target_link_libraries(
#        ${PROJECT_NAME}_multi_id_scene_test
#        ${PROJECT_NAME}_helper
#        ${PROJECT_NAME}_packages
#        ${PROJECT_NAME}_packages_impl
#        ${PROJECT_NAME}_test_core
#        ${PCL_LIBRARIES}
#        glpk
#        ${catkin_LIBRARIES}
#)

#target_link_libraries(
#        ${PROJECT_NAME}_parameters_test
#        ${PROJECT_NAME}_helper
#        ${PROJECT_NAME}_packages
#        ${PROJECT_NAME}_packages_impl
#        ${PROJECT_NAME}_test_core
#        ${PCL_LIBRARIES}
#        glpk
#        ${catkin_LIBRARIES}
#)

#target_link_libraries(
#       ${PROJECT_NAME}_ik_test
#       ${PROJECT_NAME}_helper
#       ${PROJECT_NAME}_packages
#       ${PROJECT_NAME}_packages_impl
#       ${PROJECT_NAME}_test_core
#       ${PCL_LIBRARIES}
#        glpk
#        ${catkin_LIBRARIES}
#)

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





