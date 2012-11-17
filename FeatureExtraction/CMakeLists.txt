
find_library( VL vl REQUIRED )
add_library( vrlt_features FeatureExtraction/features.h src/features.cpp )
target_link_libraries( vrlt_features ${VL_LIBRARIES} )

