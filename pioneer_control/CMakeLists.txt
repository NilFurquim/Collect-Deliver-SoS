cmake_minimum_required(VERSION 2.8.3)
project(pioneer_control)


find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	sensor_msgs
	std_msgs
	geometry_msgs
	image_transport
	message_generation
	genmsg
	actionlib
)

## Declare ROS messages, services and actions ##
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Vec2i32.msg
   Vec2i8.msg
   PoseGrid.msg
   TransportRequest.msg
   TransportAccept.msg
   TransportDone.msg
   ChooseClosestMsg.msg
   PAInformation.msg
   UpdateMapMsg.msg
   RangeArray.msg
)

 add_service_files(
   FILES
   ForkliftLift.srv
   ForkliftLower.srv
   ForkliftMoveTo.srv
   MapInformationGetMap.srv
   MapInformationGetProductAreas.srv
   MapInformationUpdateMap.srv
   PathPlanningDefinePath.srv
   PathPlanningCalculateDistance.srv
   GetLocalization.srv
   LocalizationInit.srv
   #SonarSensorGetInformation.srv
 )
## Generate actions in the 'action' folder
add_action_files(
   FILES
   NavigationDriveTo.action
   NavigationExecutePath.action
   ControlGoPickUpProduct.action
   ControlGoDeliverProduct.action
   ObjectManipulationPickUp.action
   ObjectManipulationRelease.action
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	actionlib_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

macro(add_exec_dep_and_link name)
	add_executable(${name} src/${name}.cpp)
	add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(${name} ${catkin_LIBRARIES})
endmacro()

add_exec_dep_and_link(navigation)
add_exec_dep_and_link(forklift_actuator)
add_exec_dep_and_link(image_processing)
add_exec_dep_and_link(localization)
add_exec_dep_and_link(map_information)
add_exec_dep_and_link(path_planning)
add_exec_dep_and_link(sonar_sensor)
add_exec_dep_and_link(object_manipulation)
add_exec_dep_and_link(control)
add_exec_dep_and_link(robotic_agent)
add_exec_dep_and_link(application)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
