cmake_minimum_required(VERSION 2.8.3)
project(bariago)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy roslib rosowl message_generation)




add_message_files(FILES
  Cocktail.msg
  CocktailMixing.msg
)

add_service_files(FILES
  CocktailOrder.srv
  DrinkEvaluation.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy roslib rosowl
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  control_msgs
  trajectory_msgs
)

catkin_package()

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(bariago_pick src/bariago_pick.cpp)
target_link_libraries(bariago_pick ${catkin_LIBRARIES})


### INSTALL ###

install(TARGETS bariago_pick
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(PROGRAMS spin.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})