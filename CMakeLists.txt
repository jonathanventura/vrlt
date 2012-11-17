cmake_minimum_required( VERSION 2.6 )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if (NOT CMAKE_BUILD_TYPE)
	message(STATUS "No build type selected, default to Release")
	set(CMAKE_BUILD_TYPE "Release")
endif()


project( vrlt )

set( BUILD_MULTIVIEW TRUE CACHE BOOL "Build MultiView module" )
set( BUILD_FEATURE_EXTRACTION TRUE CACHE BOOL "Build FeatureExtraction module" )
set( BUILD_FEATURE_MATCHER TRUE CACHE BOOL "Build FeatureMatcher module" )

find_package( CVD REQUIRED )

include_directories( ${CVD_INCLUDE_DIR} )

if( BUILD_MULTIVIEW )
include_directories( $(vrlt)/MultiView )
add_subdirectory( MultiView )
endif()

if( BUILD_FEATURE_EXTRACTION )
include_directories( $(vrlt)/FeatureExtraction )
add_subdirectory( FeatureExtraction )
endif()

if( BUILD_FEATURE_MATCHER )
include_directories( $(vrlt)/FeatureMatcher )
add_subdirectory( FeatureMatcher )
endif()
