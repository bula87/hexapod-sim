#include "Hexapod.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include <cstring>

int main(int argc,char* argv[])
{
  int simpleMode = 0;
  std::string serialPort = "";

  if(argc>1)
    if(strcmp(argv[1],"-s") == 0)
    {
      simpleMode=1;
    }
    else
    {
      serialPort = std::string(argv[1]);
    }
  if(argc>2)
    if(strcmp(argv[2],"-s") == 0)
    {
      simpleMode=1;
    }
    else
    {
      serialPort = std::string(argv[2]);
    }

  std::cout << "Debug.." << std::endl
            << "simpleMode: " << simpleMode << std::endl
            << "serialPort: " << serialPort << std::endl;

  Hexapod demoApp(simpleMode,serialPort);
  demoApp.initPhysics();
  demoApp.setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
  return glutmain(argc, argv,640,480,"Hexapod Simulator",&demoApp);
}
