
add_library( vrlt_features FeatureExtraction/features.h src/features.cpp )
if ( USE_VL )
target_link_libraries( vrlt_features ${VL} )
endif()

