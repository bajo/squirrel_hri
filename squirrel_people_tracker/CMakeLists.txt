cmake_minimum_required(VERSION 2.8.3)
project(squirrel_people_tracker)
add_compile_options( -std=c++11 )


find_package(catkin REQUIRED
std_srvs roscpp squirrel_speech_msgs squirrel_hri_msgs sensor_msgs move_base_msgs pcl_conversions pcl_ros geometry_msgs people_msgs std_msgs tf squirrel_view_controller_msgs grid_map_core grid_map_ros nav_msgs

    COMPONENTS
        geometry_msgs
        grid_map_core
        grid_map_ros
        move_base_msgs
        nav_msgs
        pcl_conversions
        pcl_ros
        people_msgs
        robotino_msgs
        roscpp
        sensor_msgs
        spencer_tracking_msgs
        squirrel_hri_msgs
        squirrel_view_controller_msgs
        std_msgs
        std_srvs
        tf
        visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        geometry_msgs
        grid_map_core
        grid_map_ros
        move_base_msgs
        nav_msgs
        pcl_conversions
        pcl_ros
        people_msgs
        robotino_msgs
        roscpp
        sensor_msgs
        spencer_tracking_msgs
        squirrel_hri_msgs
        squirrel_view_controller_msgs
        std_msgs
        std_srvs
        tf
        visualization_msgs
    LIBRARIES
        squirrel_focus_on_head
)

#add_library(squirrel_people_tracker
#   src/squirrel_people_tracker.cpp
#)

add_library(squirrel_focus_on_head
   src/focus_head.cpp
)
add_dependencies(squirrel_focus_on_head ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    squirrel_focus_on_head
        ${catkin_LIBRARIES}
)

#add_executable(squirrel_people_tracker_node src/squirrel_people_tracker_node.cpp)
add_executable(squirrel_focus_on_head_node src/squirrel_focus_on_head_node.cpp)

add_dependencies(squirrel_focus_on_head_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(squirrel_people_tracker_node
#   squirrel_people_tracker
#)

#target_link_libraries(squirrel_people_tracker
#  ${PCL_INCLUDE_DIRS}
#  ${catkin_LIBRARIES}
#)

target_link_libraries(squirrel_focus_on_head_node
   ${catkin_LIBRARIES}
   squirrel_focus_on_head
)
add_executable(squirrel_follow_child_node src/follow_child.cpp)
add_dependencies(squirrel_follow_child_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(squirrel_follow_child_node
   ${catkin_LIBRARIES}
)
add_executable(child_test src/test.cpp)
add_dependencies(child_test ${catkin_EXPORTED_TARGETS})

target_link_libraries(child_test
   ${catkin_LIBRARIES}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
install(
    TARGETS
        squirrel_follow_child_node
        squirrel_focus_on_head_node
        child_test
        RUNTIME DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    TARGETS
        squirrel_focus_on_head
        ARCHIVE DESTINATION
        ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION
        ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION
        ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
    DIRECTORY
        include/${PROJECT_NAME}/
        DESTINATION
        ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
    FILES
        startup.launch spencer.launch
        DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
