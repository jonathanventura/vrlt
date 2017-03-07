
add_library( vrlt_localizer Localizer/localizer.h src/localizer.cpp Localizer/nnlocalizer.h src/nnlocalizer.cpp )
target_compile_features( vrlt_localizer PRIVATE cxx_auto_type )
target_link_libraries( vrlt_localizer vrlt_multiview )
target_link_libraries( vrlt_localizer vrlt_estimator )
target_link_libraries( vrlt_localizer vrlt_featurematcher )
target_link_libraries( vrlt_localizer vrlt_patchtracker )
target_link_libraries( vrlt_localizer vrlt_bundleadjustment )


