
add_library( vrlt_imagecache ImageCache/imagecache.h src/imagecache.cpp )
target_compile_features( vrlt_imagecache PRIVATE cxx_auto_type )
target_link_libraries( vrlt_imagecache vrlt_patchtracker )

