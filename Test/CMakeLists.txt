
find_library( Geographic Geographic REQUIRED )
add_executable( TestLocalizer TestLocalizer.cpp )
target_link_libraries( TestLocalizer vrlt_multiview )
target_link_libraries( TestLocalizer vrlt_localizer  )
target_link_libraries( TestLocalizer vrlt_features  )
target_link_libraries( TestLocalizer vrlt_featurematcher  )
target_link_libraries( TestLocalizer ${Geographic} )

add_executable( TestTracker TestTracker.cpp )
target_link_libraries( TestTracker vrlt_multiview  )
target_link_libraries( TestTracker vrlt_patchtracker  )
target_link_libraries( TestTracker vrlt_bundleadjustment  )
target_link_libraries( TestTracker ${Geographic} )

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
add_executable( RenderTrack RenderTrack.cpp )
target_link_libraries( RenderTrack vrlt_multiview  )
target_link_libraries( RenderTrack vrlt_glutils  )
target_link_libraries( RenderTrack ${GLUT_LIBRARY} )

