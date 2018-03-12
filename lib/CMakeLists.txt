include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# list here your source files (*.cpp) of your module
set(source_files
	generic/vec_addition.cpp
	)

# list here your testing files (*.test.cc) of your module
set(test_files
	test_root.cc
	generic/vec_addition.test.cc
	)

add_library(libmousetrack ${source_files})

target_include_directories(libmousetrack PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

find_package( Boost COMPONENTS unit_test_framework REQUIRED )
find_package( OpenCV REQUIRED )
target_link_libraries(libmousetrack ${OpenCV_LIBS})

add_executable(tests ${test_files})
target_link_libraries(tests libmousetrack ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

