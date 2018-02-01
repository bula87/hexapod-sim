#ifndef IMPORT_CU_URDF_SETUP_H
#define IMPORT_CU_URDF_SETUP_H


#include "Bullet3AppSupport/CommonMultiBodySetup.h"
#include "Bullet3AppSupport/BulletDemoInterface.h"

class CustomImportURDFSetup : public CommonMultiBodySetup
{
  char m_fileName[1024];
  struct ImportUrdfInternalData* m_data;
    
public:
  CustomImportURDFSetup();
  virtual ~CustomImportURDFSetup();

	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
	virtual void stepSimulation(float deltaTime);
    
  void setFileName(const char* urdfFileName);
};

#endif //IMPORT_CU_URDF_SETUP_H
