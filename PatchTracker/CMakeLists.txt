
add_library( vrlt_patchtracker PatchTracker/patch.h src/patch.cpp PatchTracker/tracker.h src/tracker.cpp PatchTracker/ncc.h src/ncc.cpp PatchTracker/search.h PatchTracker/nccsearch.h src/nccsearch.cpp PatchTracker/ssd.h src/ssd.cpp PatchTracker/sampler.h src/sampler.cpp PatchTracker/robustlsq.h src/robustlsq.cpp )
target_compile_features( vrlt_patchtracker PRIVATE cxx_auto_type )
if( USE_ACCELERATE )
target_link_libraries( vrlt_patchtracker ${ACCELERATE} )
endif()
target_link_libraries( vrlt_patchtracker ${CVD_LIBRARIES} )
