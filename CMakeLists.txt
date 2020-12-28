cmake_minimum_required(VERSION 2.8.3)
project(bariago)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy message_generation)


add_message_files(FILES
  Cocktail.msg
)

add_service_files(FILES
  CocktailOrder.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy
)