cmake_minimum_required(VERSION 2.8.3)
project(next_best_view)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  object_database
  message_generation
  sensor_msgs
  nav_msgs
)

find_package(PCL REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
	DIRECTORY
	msg
	FILES
	AttributedPoint.msg
	AttributedPointCloud.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_helper
  CATKIN_DEPENDS roscpp roslib
  DEPENDS system_lib
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library

add_library(
	${PROJECT_NAME}_helper
	src/helper/MathHelper.cpp
	src/helper/MarkerHelper.cpp
	src/helper/TypeHelper.cpp
)

add_library(
	${PROJECT_NAME}_packages
	src/common/CommonClass.cpp
	src/hypothesis_updater/HypothesisUpdater.cpp
	src/camera_model_filter/CameraModelFilter.cpp
	src/rating/BaseScoreContainer.cpp
	src/rating/RatingModule.cpp
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
	src/camera_model_filter/impl/MapBasedSingleCameraModelFilter.cpp
	src/camera_model_filter/impl/MapBasedStereoCameraModelFilter.cpp
	src/rating/impl/DefaultRatingModule.cpp
	src/rating/impl/DefaultScoreContainer.cpp
	src/robot_model/impl/MILDRobotModel.cpp
	src/robot_model/impl/MILDRobotState.cpp
	src/space_sampler/impl/PlaneSubSpaceSampler.cpp
	src/space_sampler/impl/MapBasedSpaceSampler.cpp
	src/unit_sphere_sampler/impl/SpiralApproxUnitSphereSampler.cpp
)
target_link_libraries(
	${PROJECT_NAME}_packages_impl
	${PROJECT_NAME}_packages
	${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(
	${PROJECT_NAME}
	src/next_best_view.cpp
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



add_executable(
	${PROJECT_NAME}_test
	src/test_cases/Test.cpp
)

target_link_libraries(
	${PROJECT_NAME}_test
	${PROJECT_NAME}_helper
	${PROJECT_NAME}_packages
	${PROJECT_NAME}_packages_impl
	${PCL_LIBRARIES}
	glpk
	${catkin_LIBRARIES}
)

