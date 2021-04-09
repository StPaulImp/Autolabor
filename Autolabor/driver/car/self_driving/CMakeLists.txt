cmake_minimum_required(VERSION 3.10.2)

project(self_driving)
set(CMAKE_CXX_STANDARD 17)
#add_compile_options(-std=c++11)

if(UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
	set(CMAKE_BUILD_RPATH "$ORIGIN")
endif(UNIX)
set(LINK_STATIC_BOOST ON)
set(Boost_USE_STATIC_LIBS        ON)
set(Boost_USE_MULTITHREADED      ON)
set(CGAL_Boost_USE_STATIC_LIBS   ON)

include_directories(${CMAKE_SOURCE_DIR})


find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    std_msgs
    geometry_msgs
    nav_msgs
    message_generation
    )

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    CanBusMessage.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    CanBusService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp tf std_msgs geometry_msgs nav_msgs message_runtime
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
# find_package(CGAL REQUIRED)
# include( ${CGAL_USE_FILE} )

find_package (Eigen3 REQUIRED NO_MODULE)
include_directories( ${EIGEN3_INCLUDE_DIRS} )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time iostreams serialization chrono atomic regex program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# #---------------------------------------------------------------------------------------
# # generate excutable and add libraries
# #---------------------------------------------------------------------------------------
# add_executable(test_vehicle
#     ""
#     )
# #---------------------------------------------------------------------------------------
# # precompile macro and compile option
# #---------------------------------------------------------------------------------------
# # target_compile_options(test_vehicle
# #     PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
# #     )
# #---------------------------------------------------------------------------------------
# # source file
# #---------------------------------------------------------------------------------------
# target_sources(test_vehicle
#     # PRIVATE
#     PUBLIC
# 	src/main_test_vehicle.cpp
# 	src/net.cpp
# 	src/can.cpp
#     )
# #---------------------------------------------------------------------------------------
# # include file
# #---------------------------------------------------------------------------------------
# target_include_directories(test_vehicle
#     PUBLIC
#     ${catkin_INCLUDE_DIRS}/can.h
#     ${catkin_INCLUDE_DIRS}/candata_yhs.h
#     ${catkin_INCLUDE_DIRS}/net.h
#     ${catkin_INCLUDE_DIRS}/picojson.h
#     ${catkin_INCLUDE_DIRS}/rtk.h
#     ${catkin_INCLUDE_DIRS}/vehicle.h
#     # ${PCL_INCLUDE_DIRS}
#     ) 
    
# #---------------------------------------------------------------------------------------
# # link libraries
# #---------------------------------------------------------------------------------------
# target_link_libraries(test_vehicle
# 	${Boost_LIBRARIES}
# 	${catkin_LIBRARIES}
# 	# ${PCL_LIBRARIES}
# 	)

add_executable(test_vehicle src/main_test_vehicle.cpp include/vehicle.h src/vehicle.cpp include/net.h src/net.cpp include/picojson.h include/can.h src/can.cpp include/candata_yhs.h)
target_link_libraries(test_vehicle ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(path_recorder src/main_path_recorder.cpp include/rtk.h src/rtk.cpp)
target_link_libraries(path_recorder ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_patrol 
src/main_test_patrol.cpp 
include/rtk.h     src/rtk.cpp 
include/vehicle.h src/vehicle.cpp
include/net.h     src/net.cpp 
include/picojson.h 
include/can.h     src/can.cpp 
include/lidar.h  src/lidar.cpp 
include/candata_yhs.h
)
target_link_libraries(test_patrol ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_lidar 
src/main_test_lidar.cpp 
src/lidar.cpp include/lidar.h 
src/vehicle.cpp include/vehicle.h
src/can.cpp include/can.h
include/candata_yhs.h)
target_link_libraries(test_lidar ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(WIN32)
	add_subdirectory(win)
endif(WIN32)
