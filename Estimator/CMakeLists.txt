
add_library( vrlt_estimator Estimator/estimator.h src/estimator.cpp )
if( USE_ACCELERATE )
target_link_libraries( vrlt_estimator ${ACCELERATE} )
endif()

