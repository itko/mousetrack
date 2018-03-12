include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(source_files
	main.cpp
	)

add_executable(mousetrack ${source_files})

target_link_libraries(mousetrack libmousetrack)


