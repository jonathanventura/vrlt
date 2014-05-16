
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )
add_library( vrlt_bundleadjustment BundleAdjustment/bundle.h src/bundle.cpp BundleAdjustment/outliers.h src/outliers.cpp BundleAdjustment/updatepose.h src/updatepose.cpp )
target_link_libraries( vrlt_bundleadjustment ${CERES_LIBRARIES} )

