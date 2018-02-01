# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  



# You shouldn't have to modify anything below this line 
########################################################


INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
../OpenGL 
${GLUT_INCLUDE_DIR} 
${BULLET_PHYSICS_SOURCE_DIR}/btgui
${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
)
LINK_LIBRARIES(
Bullet3AppSupport lua-5.2.3 Bullet3Common  BulletSoftBody BulletDynamics BulletCollision LinearMath OpenGLWindow gwen OpenGLSupport BulletDynamics BulletCollision LinearMath ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} X11 pthread Xext m dl
)

ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
ADD_DEFINITIONS("-DGLEW_STATIC")
ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

ADD_EXECUTABLE(Hexapod2
        CustomImportURDFSetup.cpp
        main.cpp
        ${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
        ../../btgui/urdf/urdfdom/urdf_parser/src/pose.cpp
                        ../../btgui/urdf/urdfdom/urdf_parser/src/model.cpp
                        ../../btgui/urdf/urdfdom/urdf_parser/src/link.cpp
                        ../../btgui/urdf/urdfdom/urdf_parser/src/joint.cpp
                        ../../btgui/urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h
                        ../../btgui/urdf/urdfdom_headers/urdf_exception/include/urdf_exception/exception.h
                        ../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/pose.h
                        ../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h
                        ../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/link.h
                        ../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/joint.h
                        ../../btgui/tinyxml/tinystr.cpp
                        ../../btgui/tinyxml/tinyxml.cpp
                        ../../btgui/tinyxml/tinyxmlerror.cpp
                        ../../btgui/tinyxml/tinyxmlparser.cpp
                        ../../btgui/urdf/boost_replacement/lexical_cast.h
                        ../../btgui/urdf/boost_replacement/shared_ptr.h
                        ../../btgui/urdf/boost_replacement/printf_console.cpp
                        ../../btgui/urdf/boost_replacement/printf_console.h
                        ../../btgui/urdf/boost_replacement/string_split.cpp
                        ../../btgui/urdf/boost_replacement/string_split.h
	../../Demos/BasicDemo/BasicDemoPhysicsSetup.cpp
	../../Demos/BasicDemo/BasicDemoPhysicsSetup.h
  ../../Demos3/bullet2/BasicDemo/BasicDemo.h
  ../../Demos3/bullet2/BasicDemo/BasicDemo.cpp
)

set (CMAKE_CXX_FLAGS "-std=c++11 -g -Bstatic -pthread")

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppMotorDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppMotorDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppMotorDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
