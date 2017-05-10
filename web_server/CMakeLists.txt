cmake_minimum_required (VERSION 2.8.8)
project (Simple-Web-Server)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")
# Include command for linking projects
#include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
include_directories(.)

find_package(Threads REQUIRED)

set(BOOST_COMPONENTS system thread filesystem date_time)
# Late 2017 TODO: remove the following checks and always use std::regex
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
                set(BOOST_COMPONENTS ${BOOST_COMPONENTS} regex)
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_BOOST_REGEX")
        endif()
endif()
find_package(Boost 1.53.0 COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

if(APPLE)
        set(OPENSSL_ROOT_DIR "/usr/local/opt/openssl")
endif()


## Add greedy projection project build
#ExternalProject_Add(project_greedy_projection
#        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/../pcl/rotate_and_concat
#        )
#
#ExternalProject_Get_Property(project_greedy_projection install_dir)
#
#add_library(greedy_projection STATIC IMPORTED)
#set_property(TARGET greedy_projection
#        PROPERTY IMPORTED_LOCATION ${install_dir}/lib/greedy_projection)
#add_dependencies(greedy_projection project_greedy_projection)
#
#include_directories(${install_dir}/include/luajit-2.0)
#
find_package(PCL 1.7 REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(http_examples http_examples.cpp)
add_subdirectory(greedy_projection)

target_link_libraries(http_examples ${PCL_LIBRARIES})
target_link_libraries(http_examples greedy_projection)
target_link_libraries(http_examples ${Boost_LIBRARIES})
target_link_libraries(http_examples ${CMAKE_THREAD_LIBS_INIT})


#TODO: add requirement for version 1.0.1g (can it be done in one line?)
find_package(OpenSSL)

if(OPENSSL_FOUND)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_OPENSSL")
        target_link_libraries(http_examples ${OPENSSL_LIBRARIES})
        include_directories(SYSTEM ${OPENSSL_INCLUDE_DIR})

        add_executable(https_examples https_examples.cpp)
        target_link_libraries(https_examples ${Boost_LIBRARIES})
        target_link_libraries(https_examples ${OPENSSL_LIBRARIES})
        target_link_libraries(https_examples ${CMAKE_THREAD_LIBS_INIT})
endif()

if(MSYS) #TODO: Is MSYS true when MSVC is true?
        target_link_libraries(http_examples ws2_32 wsock32)
        target_link_libraries(http_examples greedy_projection)
        if(OPENSSL_FOUND)
                target_link_libraries(https_examples ws2_32 wsock32)
        endif()
endif()


enable_testing()
add_subdirectory(tests)

install(FILES server_http.hpp client_http.hpp server_https.hpp client_https.hpp DESTINATION include/simple-web-server)
