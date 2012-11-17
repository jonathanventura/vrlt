
find_library( TOONTAG toontag REQUIRED )
add_library( vrlt_estimator src/estimator.cpp )
target_link_libraries( vrlt_estimator ${TOONTAG_LIBRARIES} )
if( USE_ACCELERATE )
target_link_libraries( vrlt_estimator ${ACCELERATE_LIBRARIES} )
endif()

