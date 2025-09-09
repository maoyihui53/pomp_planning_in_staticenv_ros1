cmake_minimum_required(VERSION 3.0.2)
project(pomp_planning_in_static_env)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
geometric_shapes
geometry_msgs
nav_msgs
octomap_msgs
octomap_msgs
octomap_ros
octomap_ros
pcl_conversions
pcl_ros
roscpp
sensor_msgs
std_msgs
tf2
tf2_ros
visualization_msgs
image_transport
cv_bridge

)

set(BUILD_SHARED_LIBS ON)



find_package(OpenMP REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(BUILD_SHARED_LIBS ON)

set(GUROBI_HOME "/opt/gurobi1101/linux64") # Change based on your Gurobi installation

FIND_PACKAGE(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(CGAL REQUIRED)
find_package(osqp REQUIRED)
find_package(TBB REQUIRED)
find_package(octomap REQUIRED)
find_package(superray-octomap REQUIRED)
find_package(superray-quadmap REQUIRED)
find_package(superray-gridmap3d REQUIRED)
find_package(superray-gridmap2d REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io keypoints)
include(${CGAL_USE_FILE})

if(OPENMP_FOUND)
    message(STATUS "OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
    message(STATUS "ERROR: OpenMP could not be found.")
endif()



catkin_package(
   INCLUDE_DIRS include
  #  LIBRARIES pathfinding
   CATKIN_DEPENDS message_runtime geometric_shapes geometry_msgs nav_msgs octomap_msgs octomap_msgs octomap_ros octomap_ros pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf2 tf2_ros visualization_msgs image_transport cv_bridge 
   #  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  
)



include_directories(
  include
  lib
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${GUROBI_HOME}/include

  # ${PCL_INCLUDE_DIRS}
)

# add_compile_options(-fopenmp)
# add_compile_options(-std=c++17)
# add_compile_options(-flto)
# add_compile_options(-O3 -march=native -flto -funroll-loops -ftree-vectorize)
# add_compile_options(-fconcepts)
add_compile_options(
  -fopenmp
  -std=c++17
  -flto
  -O3
  -march=native
  # -funroll-loops
  # -ftree-vectorize
)

link_directories(/usr/lib/x86_64-linux-gnu/)  


# set(MAIN_FUNCTION_3D
#     src/main_function/main_function_3d.cpp
# )



# set(POMP_SCANGRAPH_3D
#   src/pomp_utils/scan_graph/ScanGraph.cpp
# )

# set(POMP_BAGSUBSCRIBER_3D
#   src/pomp_utils/bag_subscriber/BagSub.cpp
# )

# set(POPMP_PCDREADER_3D
#   src/pomp_utils/pcd_reader/PcdReader.cpp
# )

add_executable(static_cylinders_planning_example src/static_cylinders_planning_example.cpp)
add_executable(static_realmap_planning_example src/static_realmap_planning_example.cpp )

# add_executable(static_planning_example src/static_planning_example.cpp)

# add_executable(graph_mapping_example src/graph_mapping_example.cpp)
# add_executable(bag_mapping_example src/bag_mapping_example.cpp)
# add_executable(pcd_mapping_example src/pcd_mapping_example.cpp)
# add_library(main_function_3d STATIC ${MAIN_FUNCTION_3D})
# add_library(scan_graph_3d STATIC ${POMP_SCANGRAPH_3D})
# add_library(bag_subscriber_3d STATIC ${POMP_BAGSUBSCRIBER_3D})
# add_library(pcd_reader_3d STATIC ${POPMP_PCDREADER_3D})
# add_library(ikdTree_pomp src/ikd_tree/ikdTree.cpp )



add_library(pomp_jps_tbb_lib src/jps_tbb/jps_planner/graph_search.cpp
  src/jps_tbb/jps_planner/jps_planner.cpp)

add_library(pomp_dmp_tbb_lib src/jps_tbb/distance_map_planner/graph_search.cpp
  src/jps_tbb/distance_map_planner/distance_map_planner.cpp)


add_library(pomp_jps_lib src/jps/jps_planner/graph_search.cpp
  src/jps/jps_planner/jps_planner.cpp)

add_library(pomp_dmp_lib src/jps/distance_map_planner/graph_search.cpp
  src/jps/distance_map_planner/distance_map_planner.cpp)


target_link_libraries( static_cylinders_planning_example  PRIVATE osqp::osqp  ${GUROBI_HOME}/lib/libgurobi_c++.a 
              ${GUROBI_HOME}/lib/libgurobi110.so 
              pomp_jps_lib pomp_dmp_lib  ${OpenMP_CXX_LIBRARIES} ${CGAL_LIBRARIES}  ${catkin_LIBRARIES}  ${OCTOMAP_LIBRARIES}  TBB::tbb pomp_jps_tbb_lib pomp_dmp_tbb_lib
pthread rt)


target_link_libraries( static_realmap_planning_example  PRIVATE osqp::osqp  ${GUROBI_HOME}/lib/libgurobi_c++.a 
              ${GUROBI_HOME}/lib/libgurobi110.so 
              pomp_jps_lib pomp_dmp_lib  ${OpenMP_CXX_LIBRARIES} ${CGAL_LIBRARIES}  ${catkin_LIBRARIES}  ${OCTOMAP_LIBRARIES}  TBB::tbb pomp_jps_tbb_lib pomp_dmp_tbb_lib
pthread rt)

# target_link_libraries( static_planning_example  PRIVATE osqp::osqp ikdTree_pomp ${GUROBI_HOME}/lib/libgurobi_c++.a 
#               ${GUROBI_HOME}/lib/libgurobi110.so 
#               ${OpenMP_CXX_LIBRARIES} ${CGAL_LIBRARIES}  ${catkin_LIBRARIES}  ${OCTOMAP_LIBRARIES}  TBB::tbb
# pthread rt)

# target_link_libraries( graph_mapping_example  PRIVATE osqp::osqp ${GUROBI_HOME}/lib/libgurobi_c++.a 
#               ${GUROBI_HOME}/lib/libgurobi110.so main_function_3d scan_graph_3d 
#               ${OpenMP_CXX_LIBRARIES} ${CGAL_LIBRARIES}  ${catkin_LIBRARIES}  ${OCTOMAP_LIBRARIES}  TBB::tbb
# pthread rt)


# target_link_libraries( bag_mapping_example  PRIVATE osqp::osqp ${GUROBI_HOME}/lib/libgurobi_c++.a 
#               ${GUROBI_HOME}/lib/libgurobi110.so main_function_3d bag_subscriber_3d
#               ${OpenMP_CXX_LIBRARIES} ${CGAL_LIBRARIES}  ${catkin_LIBRARIES}  ${OCTOMAP_LIBRARIES}  TBB::tbb
# pthread rt)


# target_link_libraries( pcd_mapping_example  PRIVATE osqp::osqp ${GUROBI_HOME}/lib/libgurobi_c++.a 
#               ${GUROBI_HOME}/lib/libgurobi110.so main_function_3d pcd_reader_3d
#               ${OpenMP_CXX_LIBRARIES} ${CGAL_LIBRARIES}  ${catkin_LIBRARIES}  ${OCTOMAP_LIBRARIES}  TBB::tbb
# pthread rt)



foreach(dir launch )
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)