cmake_minimum_required(VERSION 2.8.3)
project(squirrel_leg_detector)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  visualization_msgs
  people_msgs
  tf
  squirrel_speech_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tracker_bonn
  CATKIN_DEPENDS 
	roscpp 
	roslib
	sensor_msgs
	people_msgs
  visualization_msgs
  people_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(squirrel_legs_close_to_pose src/squirrel_legs_close_to_pose.cpp)
add_executable(leg_detector src/leg_detector.cpp src/lfeatures.cpp src/lgeometry.cpp src/people2D_engine.cpp)
target_link_libraries(leg_detector ${catkin_LIBRARIES} gsl gslcblas)
target_link_libraries(squirrel_legs_close_to_pose ${catkin_LIBRARIES} )
add_dependencies(squirrel_legs_close_to_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(squirrel_legs_close_to_pose squirrel_speech_msgs_generate_messages_cpp)
