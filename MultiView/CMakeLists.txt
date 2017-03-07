
add_library( vrlt_multiview MultiView/multiview.h MultiView/pyramid.h src/pyramid.cpp src/multiview.cpp MultiView/multiview_io_xml.h src/multiview_io_xml.cpp TinyXML/tinystr.cpp TinyXML/tinyxml.cpp TinyXML/tinyxmlerror.cpp TinyXML/tinyxmlparser.cpp )
target_compile_features( vrlt_multiview PRIVATE cxx_auto_type )
target_link_libraries( vrlt_multiview ${OpenCV_LIBS} )

add_executable( ReadReconstruction src/ReadReconstruction.cpp )
target_compile_features( ReadReconstruction PRIVATE cxx_auto_type )
target_link_libraries( ReadReconstruction vrlt_multiview )
