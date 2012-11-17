cmake_minimum_required( VERSION 2.6 )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if (NOT CMAKE_BUILD_TYPE)
	message(STATUS "No build type selected, default to Release")
	set(CMAKE_BUILD_TYPE "Release")
endif()


project( vrlt )

find_package( CVD REQUIRED )

include_directories( ${CVD_INCLUDE_DIR} )

include_directories( $(vrlt)/MultiView )
add_subdirectory( MultiView )
