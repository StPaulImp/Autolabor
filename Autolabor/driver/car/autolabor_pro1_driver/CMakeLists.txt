cmake_minimum_required(VERSION 2.8.3)
project(autolabor_pro1_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  message_generation
  tf
)

add_message_files(
  FILES
  Encode.msg
)

generate_messages()
generate_dynamic_reconfigure_options(cfg/SimAutolaborDriver.cfg)

catkin_package(CATKIN_DEPENDS roscpp tf message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(autolabor_pro1_driver src/autolabor_driver.cpp)
add_dependencies(autolabor_pro1_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(autolabor_pro1_driver ${catkin_LIBRARIES})

add_executable(sim_autolabor_pro1_driver src/sim_autolabor_driver.cpp)
add_dependencies(sim_autolabor_pro1_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(sim_autolabor_pro1_driver ${catkin_LIBRARIES})

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
install(TARGETS autolabor_pro1_driver sim_autolabor_pro1_driver
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
