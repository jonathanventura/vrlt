
add_library( vrlt_features FeatureExtraction/features.h src/features.cpp )
target_compile_features( vrlt_features PRIVATE cxx_auto_type )
target_link_libraries( vrlt_features vrlt_multiview )


