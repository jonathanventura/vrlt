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
set( BUILD_IMAGECACHE TRUE CACHE BOOL "Build ImageCache module" )
set( BUILD_BUNDLEADJUSTMENT TRUE CACHE BOOL "Build BundleAdjusment module" )
set( BUILD_SFMPIPELINE TRUE CACHE BOOL "Build reconstruction pipeline programs" )
set( BUILD_TEST TRUE CACHE BOOL "Build test programs" )

set( USE_ACCELERATE TRUE CACHE BOOL "Use Accelerate framework" )
set( USE_DISPATCH TRUE CACHE BOOL "Use Grand Central Dispatch for multithreading" )

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

find_package( Sophus REQUIRED )
include_directories( ${SOPHUS_INCLUDE_DIR} )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

if( USE_ACCELERATE )
find_library( ACCELERATE Accelerate REQUIRED )
add_definitions( -DUSE_ACCELERATE )
endif()

if( USE_DISPATCH )
add_definitions( -DUSE_DISPATCH )
endif()

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
include_directories( $(vrlt)/LocalizerClient )
include_directories( $(vrlt)/LocalizerServer )
add_subdirectory( Localizer )
add_subdirectory( LocalizerClient )
add_subdirectory( LocalizerServer )
endif()

if ( BUILD_IMAGECACHE )
include_directories( $(vrlt)/ImageCache )
add_subdirectory( ImageCache )
endif()

if ( BUILD_BUNDLEADJUSTMENT )
include_directories( $(vrlt)/BundleAdjustment )
add_subdirectory( BundleAdjustment )
endif()

if ( BUILD_SFMPIPELINE )
include_directories( $(vrlt)/SfMPipeline )
add_subdirectory( SfMPipeline )
endif()

if ( BUILD_TEST )
include_directories( $(vrlt)/Test )
add_subdirectory( Test )
endif()

