
find_library( OPENGL OpenGL REQUIRED )
add_library( vrlt_glutils GLUtils/CheckGL.h GLUtils/GLGenericDrawable.h GLUtils/GLModel.h GLUtils/GLShadow.h GLUtils/GLSL.h GLUtils/GLSL.h GLUtils/GLSLHelpers.h GLUtils/GLSLPrograms.h GLUtils/LoadOBJ.h GLUtils/TexturePacker.h src/CheckGL.cpp src/GLGenericDrawable.cpp src/GLModel.cpp src/GLShadow.cpp src/GLSL.cpp src/GLSLHelpers.cpp src/GLSLPrograms.cpp src/LoadOBJ.cpp src/TexturePacker.cpp )
target_link_libraries( vrlt_glutils ${OPENGL} )

