# add this path to include path of library files
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# list here your source files (*.cpp) of your module
set(source_files
        generic/cluster.cpp
        generic/cluster_chain.cpp
        generic/disparity_map.cpp
        generic/point_cloud.cpp
        generic/trajectory.cpp
        generic/vec_addition.cpp
        generic/center.cpp
        descripting/cog.cpp
	      matching/nearest_neighbour.cpp
	  )

# list here your testing files (*.test.cc) of your module
set(test_files
        test_root.cc
        generic/point_cloud.test.cc
        generic/vec_addition.test.cc
        )

add_library(libmousetrack ${source_files})

# add this path to include path of target
target_include_directories(libmousetrack PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

find_package( Boost COMPONENTS unit_test_framework REQUIRED )
find_package( OpenCV REQUIRED )

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

target_link_libraries(libmousetrack ${OpenCV_LIBS})

add_executable(tests ${test_files})
target_link_libraries(tests libmousetrack ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

# create a target for the headers, we don't actually need the result
# but this way, IDE's also show header files in their project overviews
file(GLOB_RECURSE header_files "*.h")
add_custom_target(headers SOURCES ${header_files})
