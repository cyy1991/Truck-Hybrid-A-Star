cmake_minimum_required(VERSION 3.1)
project(truck_trailer_hybrid_a_star)

# @Copyright Yongyu Chen 
# yongyu.chen@tum.de  

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#############################
#Dependencies
#############################s
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3.5)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(./include)
include_directories(${PYTHON_INCLUDE_DIRS})

##############################
#Grid A Star testing 
##############################

add_executable(grid_a_star src/grid_a_star.cpp)
target_link_libraries(grid_a_star ${PYTHON_LIBRARIES})

add_library(grid_a_star_shared SHARED src/grid_a_star.cpp)

##############################
# Hybrid A Star testing 
##############################

# add_executable(hybrid_a_star src/hybrid_a_star.cpp)
# target_link_libraries(hybrid_a_star ${PYTHON_LIBRARIES} grid_a_star nanoflann car trailer)

add_executable(hybrid_a_star src/hybrid_a_star.cpp)
target_link_libraries(hybrid_a_star ${PYTHON_LIBRARIES} grid_a_star_shared nanoflann_shared trailer_shared rs_path_shared)

##############################
#KD Tree Main Function
##############################

add_executable(kdtree src/kdtree.cpp)
target_link_libraries(kdtree ${PYTHON_LIBRARIES})

##############################
#PathTracking
##############################

# add_executable(reeds_shepp_planning src/reeds_shepp.cpp src/reeds_shepp_wrapper.cpp src/reeds_shepp_test.cpp)
# target_link_libraries(reeds_shepp_planning ${PYTHON_LIBRARIES})

##############################
# Testing for nanoflann 
##############################

add_executable(nanoflann src/test_kdtree.cpp)
target_link_libraries(nanoflann ${PYTHON_LIBRARIES})
add_library(nanoflann_shared SHARED src/test_kdtree.cpp)

##############################
# Testing for visualizing the car
##############################

add_executable(car src/vehicle.cpp)
target_link_libraries(car ${PYTHON_LIBRARIES})

##################################
# Testing for visualizing the trailer 
##################################

add_executable(trailer src/trailerlib.cpp)
target_link_libraries(trailer ${PYTHON_LIBRARIES})

add_library(trailer_shared SHARED src/trailerlib.cpp)

##################################
# Testing for reeds_shepp path  
##################################

add_executable(rs_path src/reeds_shepp_path.cpp src/reeds_shepp_path_test.cpp)
target_link_libraries(rs_path ${PYTHON_LIBRARIES})
add_library(rs_path_shared SHARED src/reeds_shepp_path.cpp src/reeds_shepp_path_test.cpp)
