
add_executable( InitializeFromFile InitializeFromFile.cpp )
target_link_libraries( InitializeFromFile vrlt_multiview )

add_executable( ExtractImagesCatadioptric ExtractImagesCatadioptric.cpp ocam/ocam_functions.cpp ocam/rpoly.cpp ocam/world2cam.cpp extract_images.h extract_images.cpp )
target_link_libraries( ExtractImagesCatadioptric vrlt_multiview )

add_executable( ExtractImagesSpherical ExtractImagesSpherical.cpp extract_images.h extract_images.cpp )
target_link_libraries( ExtractImagesSpherical vrlt_multiview )

add_executable( EstimateVertical EstimateVertical.cpp lines.cpp lines.h )
target_link_libraries( EstimateVertical vrlt_multiview )
target_link_libraries( EstimateVertical vrlt_estimator  )

add_executable( ExtractSIFT ExtractSIFT.cpp )
target_link_libraries( ExtractSIFT vrlt_multiview )
target_link_libraries( ExtractSIFT vrlt_features )

add_executable( ExtractORB ExtractORB.cpp )
target_link_libraries( ExtractORB vrlt_multiview )
target_link_libraries( ExtractORB vrlt_features )

add_executable( PairwiseMatch PairwiseMatch.cpp match.cpp match.h )
target_link_libraries( PairwiseMatch vrlt_multiview )
target_link_libraries( PairwiseMatch vrlt_features )
target_link_libraries( PairwiseMatch vrlt_featurematcher )
target_link_libraries( PairwiseMatch vrlt_estimator )

add_executable( LinearMatch LinearMatch.cpp match.cpp match.h )
target_link_libraries( LinearMatch vrlt_multiview )
target_link_libraries( LinearMatch vrlt_features )
target_link_libraries( LinearMatch vrlt_featurematcher )
target_link_libraries( LinearMatch vrlt_estimator )

add_executable( ShowMatches ShowMatches.cpp )
target_link_libraries( ShowMatches vrlt_multiview )

add_executable( MakeTracks MakeTracks.cpp )
target_link_libraries( MakeTracks vrlt_multiview )

add_executable( Bootstrap Bootstrap.cpp )
target_link_libraries( Bootstrap vrlt_multiview )
target_link_libraries( Bootstrap vrlt_estimator )
target_link_libraries( Bootstrap vrlt_bundleadjustment )

add_executable( AddImages AddImages.cpp )
target_link_libraries( AddImages vrlt_multiview )
target_link_libraries( AddImages vrlt_estimator )
target_link_libraries( AddImages vrlt_bundleadjustment )

add_executable( EstimateGround EstimateGround.cpp )
target_link_libraries( EstimateGround vrlt_multiview )
target_link_libraries( EstimateGround vrlt_estimator  )

add_executable( EstimateNormals EstimateNormals.cpp )
target_link_libraries( EstimateNormals vrlt_multiview )

find_library( Geographic Geographic REQUIRED )
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )
add_executable( AlignOSM AlignOSM.cpp osm_xml_reader.h osm_xml_reader.cpp )
target_link_libraries( AlignOSM vrlt_multiview )
target_link_libraries( AlignOSM ${Geographic} )
target_link_libraries( AlignOSM ${CERES_LIBRARIES} )

find_library( Geographic Geographic REQUIRED )
add_executable( RefineAlignOSM RefineAlignOSM.cpp osm_xml_reader.h osm_xml_reader.cpp )
target_link_libraries( RefineAlignOSM vrlt_multiview )
target_link_libraries( RefineAlignOSM vrlt_bundleadjustment )
target_link_libraries( RefineAlignOSM ${Geographic} )

add_executable( MakePLY MakePLY.cpp )
target_link_libraries( MakePLY vrlt_multiview )

add_executable( Organize Organize.cpp )
target_link_libraries( Organize vrlt_multiview )

add_executable( ExportPoses ExportPoses.cpp )
target_link_libraries( ExportPoses vrlt_multiview )

