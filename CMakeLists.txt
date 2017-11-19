project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

#add_subdirectory(lib/tinyspline)

add_definitions(-std=c++11)

#include_directories(lib/splinter/include)
#include_directories(src/Eigen-3.3)

#include_directories(lib/tinyspline/library)

set(sources 
  src/main.cpp 
  #lib/tinyspline/library/tinysplinecpp.h
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path_planning ${sources})

#link_directories(build/lib)
target_link_libraries(path_planning 
  z ssl uv uWS 
  #${CMAKE_CURRENT_SOURCE_DIR}/build/lib/libtinysplinecpp.a  
)
