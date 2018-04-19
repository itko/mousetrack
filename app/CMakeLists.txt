# make app headers available
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# list here all source files appart from main
set(source_files
        cli_controller.cpp
        cli_options.cpp
        controller.cpp
        matlab_reader.cpp
        matlab_reader_concurrent.cpp
        pipeline.cpp
        pipeline_observer.cpp
        pipeline_factory.cpp
        pipeline_timer.cpp
        pipeline_writer.cpp
	)

# list here all source files related with the gui
set(gui_source_files
        qt_controller.cpp
        )

# list here your testing files (*.test.cc) of your module
set(test_files
        test_root.cc
        matlab_reader.test.cc
        )

# find and import dependencies

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( Boost COMPONENTS
    system
    filesystem
    program_options
    log
    unit_test_framework
    REQUIRED )

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

if(ENABLE_GUI)
  find_package(Qt5Core)
  
  # Find the QtWidgets library
  find_package(Qt5Widgets)
  set(GUI_LIBRARIES
    Qt5::Widgets
    )
  add_definitions(-DENABLE_GUI)
else(ENABLE_GUI)
    # reset gui sources
    set(gui_source_files "")
endif(ENABLE_GUI)


# compile everything into a library and then link into the target executable (tests, main, etc.)
# this should increase compilation speed
add_library(libmousetrackAll ${source_files})

# link all dependencies
target_link_libraries(libmousetrackAll
    libmousetrack
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_LOG_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
)

# create the mousetrack executable
add_executable(mousetrack main.cpp ${gui_source_files})
target_link_libraries(mousetrack
    libmousetrackAll
    ${GUI_LIBRARIES}
    )

# create unit test executable
add_executable(appTests ${test_files})
target_link_libraries(appTests
    libmousetrackAll
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    )


# create a target for the headers, we don't actually need the result
# but this way, IDE's also show header files in their project overviews
file(GLOB_RECURSE appHeader_files "*.h")
add_custom_target(appHeaders SOURCES ${appHeader_files})
