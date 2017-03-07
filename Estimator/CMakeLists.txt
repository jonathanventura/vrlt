
add_library( vrlt_estimator Estimator/estimator.h src/estimator.cpp src/closed_form_polynomial_solver.h src/five_point_relative_pose.h src/gauss_jordan.h src/perspective_three_point.h src/polynomial.h src/util.h src/closed_form_polynomial_solver.cc src/five_point_relative_pose.cc src/perspective_three_point.cc src/polynomial.cc src/util.cc)
target_link_libraries( vrlt_estimator vrlt_multiview )
target_compile_features( vrlt_estimator PRIVATE cxx_auto_type )
if( USE_ACCELERATE )
target_link_libraries( vrlt_estimator ${ACCELERATE} )
endif()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_link_libraries( vrlt_estimator dispatch)
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fblocks")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
