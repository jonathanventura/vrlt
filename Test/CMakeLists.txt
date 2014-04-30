
add_executable( TestLocalizer TestLocalizer.cpp )
target_link_libraries( TestLocalizer vrlt_multiview )
target_link_libraries( TestLocalizer vrlt_localizer  )
target_link_libraries( TestLocalizer vrlt_features  )
target_link_libraries( TestLocalizer vrlt_featurematcher  )

add_executable( TestTracker TestTracker.cpp )
target_link_libraries( TestTracker vrlt_multiview  )
target_link_libraries( TestTracker vrlt_patchtracker  )

add_executable( RenderTrack RenderTrack.cpp )
target_link_libraries( RenderTrack vrlt_multiview  )
target_link_libraries( RenderTrack vrlt_glutils  )
target_link_libraries( RenderTrack ${GLUT} )

