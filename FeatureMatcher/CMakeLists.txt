
find_library( OPENCL OpenCL REQUIRED )
add_library( vrlt_featurematcher FeatureMatcher/featurematcher.h FeatureMatcher/nn.h FeatureMatcher/bruteforce.h FeatureMatcher/approxnn.h src/featurematcher.cpp src/bruteforce.cpp src/approxnn.cpp )
target_compile_features( vrlt_featurematcher PRIVATE cxx_auto_type )
target_link_libraries( vrlt_featurematcher ${OPENCL} )
target_link_libraries( vrlt_featurematcher vrlt_multiview )


