find_library( Geographic Geographic REQUIRED )
add_executable( vrlt_server Server.cpp )
target_link_libraries( vrlt_server vrlt_multiview )
target_link_libraries( vrlt_server vrlt_features )
target_link_libraries( vrlt_server vrlt_featurematcher )
target_link_libraries( vrlt_server vrlt_estimator )
target_link_libraries( vrlt_server vrlt_localizer )
target_link_libraries( vrlt_server ${Geographic} )



