cmake_minimum_required(VERSION 3.1)

project(mousetrack)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# options
option(ENABLE_GUI "Build with GUI" ON)
option(ENABLE_OPENMP "Build with OpenMP" ON)
option(ENABLE_ROSBAG "Build with Ros-Bag support" ON)

message("GUI: " ${ENABLE_GUI})
message("OpenMP: " ${ENABLE_OPENMP})
message("RosBag: " ${ENABLE_ROSBAG})

if(ENABLE_OPENMP)
    find_package(OpenMP)
    if (OPENMP_FOUND)
        message("OpenMP found.")
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    else(OPENMP_FOUND)
        message("OpenMP not found.")
    endif(OPENMP_FOUND)
endif(ENABLE_OPENMP)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_subdirectory(lib)
add_subdirectory(app)
