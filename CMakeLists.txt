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
set( BUILD_ESTIMATOR TRUE CACHE BOOL "Build Estimator module" )
set( BUILD_GLUTILS TRUE CACHE BOOL "Build GLUtils module" )
set( BUILD_PATCHTRACKER TRUE CACHE BOOL "Build PatchTracker module" )
set( BUILD_LOCALIZER TRUE CACHE BOOL "Build Localizer module" )

set( USE_ACCELERATE TRUE CACHE BOOL "Use Accelerate framework" )
set( USE_DISPATCH TRUE CACHE BOOL "Use Grand Central Dispatch for multithreading" )
set( USE_VL TRUE CACHE BOOL "Use VLFeat library" )

if( USE_ACCELERATE )
find_library( ACCELERATE Accelerate REQUIRED )
add_definitions( -DUSE_ACCELERATE )
endif()

if( USE_DISPATCH )
add_definitions( -DUSE_DISPATCH )
endif()

if ( USE_VL )
find_library( VL vl REQUIRED )
add_definitions( -DUSE_VL )
endif()

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

if( BUILD_ESTIMATOR )
include_directories( $(vrlt)/Estimator )
add_subdirectory( Estimator )
endif()

if( BUILD_GLUTILS )
include_directories( $(vrlt)/GLUtils )
add_subdirectory( GLUtils )
endif()

if( BUILD_PATCHTRACKER )
include_directories( $(vrlt)/PatchTracker )
add_subdirectory( PatchTracker )
endif()

if( BUILD_LOCALIZER )
include_directories( $(vrlt)/Localizer )
add_subdirectory( Localizer )
endif()

