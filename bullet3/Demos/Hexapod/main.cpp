#include "Hexapod.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include <cstring>

int main(int argc,char* argv[])
{
  int simpleMode = 0;

  if(argc>1)
    if(strcmp(argv[1],"-s") == 0)
      simpleMode=1;
  Hexapod demoApp(simpleMode);
  demoApp.initPhysics();
  demoApp.setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
  return glutmain(argc, argv,640,480,"Hexapod Simulator",&demoApp);
}
