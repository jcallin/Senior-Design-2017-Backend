cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(greedy_projection)

find_package(PCL 1.7 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (greedy_projection greedy_projection.cpp)
#add_executable (pcd_gen pcd_gen.cpp)
#add_executable (concatenate_clouds concatenate_clouds.cpp)

#target_link_libraries (greedy_projection ${PCL_LIBRARIES})
#add_library (greedy_projection ${PCL_LIBRARIES})
#target_link_libraries (pcd_gen ${PCL_LIBRARIES})
#target_link_libraries (concatenate_clouds ${PCL_LIBRARIES})

# Make greedy projection into a library for the server to use
 add_library(greedy_projection STATIC greedy_projection_lib.cpp)
 install(TARGETS greedy_projection DESTINATION lib)
 install(FILES greedy_projection_lib.h DESTINATION include)
