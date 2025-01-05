cmake_minimum_required(VERSION 3.10)
project(Wheel-GINS)

set(CMAKE_CXX_STANDARD 14)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_definitions( -D_USE_MATH_DEFINES=True)
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
else()
    set(CMAKE_CXX_FLAGS "-Wno-unused-result")
endif()

file(GLOB_RECURSE SOURCE
        src/main.cpp
        src/fileio/fileloader.cpp
        src/fileio/filesaver.cpp
        src/wheelgins/wheelgins.cpp
        src/wheelgins/insmech.cpp)


include_directories(${PROJECT_SOURCE_DIR}/src)
add_executable(${PROJECT_NAME} ${SOURCE})



include_directories(3rdParty/eigen-3.3.9)

add_subdirectory(3rdParty/yaml-cpp-0.7.0)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(3rdParty/abseil-cpp)
target_link_libraries(${PROJECT_NAME} absl)
