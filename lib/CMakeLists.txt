include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(source_files
	generic/vec_addition.cpp
	)

add_library(libmousetrack ${source_files})

target_include_directories(libmousetrack PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

find_package( OpenCV REQUIRED )
target_link_libraries(libmousetrack ${OpenCV_LIBS})


