
find_library( OPENCL OpenCL REQUIRED )
add_library( vrlt_featurematcher FeatureMatcher/featurematcher.h FeatureMatcher/nn.h FeatureMatcher/bruteforce.h src/featurematcher.cpp src/bruteforce.cpp )
target_link_libraries( vrlt_featurematcher ${OPENCL} )

