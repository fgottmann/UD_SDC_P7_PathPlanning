project(PATH_PLANNING)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources main.cpp sensorfusion.cpp trajectoryplanning.cpp behaviorplanning.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "Build examples." FORCE)
add_subdirectory(external/qpOASES)

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS qpOASES)

ADD_CUSTOM_COMMAND(
    TARGET path_planning
    POST_BUILD
    COMMAND cp -R ${CMAKE_CURRENT_BINARY_DIR}/path_planning ${CMAKE_CURRENT_BINARY_DIR}/../
)

