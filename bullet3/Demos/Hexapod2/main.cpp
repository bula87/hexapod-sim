#include "CustomImportURDFSetup.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include <cstring>
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "../../Demos3/bullet2/BasicDemo/BasicDemo.h"

int main(int argc,char* argv[])
{
  static BulletDemoInterface* sCurrentDemo;
  SimpleOpenGL3App* simpleApp=0;
  simpleApp = new SimpleOpenGL3App("TEST",300,300);

  CustomImportURDFSetup* physicsSetup = new CustomImportURDFSetup();
  physicsSetup->setFileName("hex.urdf");

  sCurrentDemo = new BasicDemo(simpleApp, physicsSetup);
  simpleApp->setUpAxis(2);
  simpleApp->m_renderer->setCameraDistance(13);
	simpleApp->m_renderer->setCameraPitch(0);
	simpleApp->m_renderer->setCameraTargetPosition(0,0,0);


	sCurrentDemo->initPhysics();
	

  do
	{
	    static int frameCount = 0;
		frameCount++;
		
		simpleApp->m_instancingRenderer->init();
		simpleApp->m_instancingRenderer->updateCamera();

		simpleApp->drawGrid();
		char bla[1024];
		sprintf(bla,"Simple test frame %d", frameCount);

		simpleApp->drawText(bla,10,10);
		simpleApp->swapBuffer();
	  sCurrentDemo->stepSimulation(1.f/60.f);
    sCurrentDemo->renderScene();
	} while (!simpleApp->m_window->requestedExit());

  sCurrentDemo->exitPhysics();
  delete sCurrentDemo;
  delete physicsSetup;
  
  return 0;
}

