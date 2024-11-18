cmake_minimum_required(VERSION 3.29.0)
project(ExtendedKalmanFilter)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LISTS)

add_library(common
SHARED
${SRC_LISTS}
)

target_include_directories(common
PUBLIC
${PROJECT_SOURCE_DIR}/include
)

add_executable(ExtendedKalmanFilter main.cpp)

target_link_libraries(ExtendedKalmanFilter
PUBLIC
common
)