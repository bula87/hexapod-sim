#ifndef HEXAPOD_H
#define HEXAPOD_H

#include <thread>
#include <memory>
#include <functional>
#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <fstream>
#include "GlutDemoApplication.h"
#include "serial.h"
#include "LinearMath/btAlignedObjectArray.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class Hexapod;

#define NUM_LEGS 6
#define BODYPART_COUNT 3 * NUM_LEGS + 1
#define JOINT_COUNT BODYPART_COUNT - 1


class HexapodServer
{
public:
    HexapodServer(Hexapod* _hexapod,int mode, std::string serialDevice):hexapod(_hexapod), simpleMode_(mode), serialDevice_(serialDevice){
        servosMap.clear();
        std::ifstream configFile;
        configFile.open("config.txt");
        int count;
        configFile>>count;
        std::cout<<"Mode: " << simpleMode_ << std::endl;
        std::cout<<"Servo mapping: "<<std::endl;
        for(int i=0; i<count; ++i)
        {
            int a,b;
            configFile>>a>>b;
            std::cout<<a<<" -> "<<b<<std::endl;
            servosMap[a] = b;
        }
    }
    void run();
private:
    Hexapod* hexapod;
    int simpleMode_;
    std::string serialDevice_;
    std::map<int, int> servosMap; // first: input servo ID
                                 // second: actual servo ID in this system
};

class Hexapod : public GlutDemoApplication
{
    float m_Time;
    float m_fCyclePeriod; // in milliseconds
    float m_fMuscleStrength;

    btAlignedObjectArray<class HexapodRig*> m_rigs;

    //keep the collision shapes, for deletion/cleanup
    btAlignedObjectArray<btCollisionShape*>    m_collisionShapes;

    btBroadphaseInterface*    m_broadphase;

    btCollisionDispatcher*    m_dispatcher;

    btConstraintSolver*    m_solver;

    btDefaultCollisionConfiguration* m_collisionConfiguration;

    HexapodServer *hexapodServer;

public:
    Hexapod(int mode, std::string serialDevice):simpleMode_(mode)
    {
        hexapodServer = new HexapodServer(this,simpleMode_,serialDevice);
        std::thread t1(std::bind(&HexapodServer::run, hexapodServer));
        t1.detach();
        for(int i=0; i<JOINT_COUNT; ++i)
        {
            servoPercentage[i] = 0.5f;
        }
    }
    void initPhysics();

    void exitPhysics();

    virtual ~Hexapod()
    {
        exitPhysics();
    }

    void spawnHexapodRig(const btVector3& startOffset, bool bFixed);

    virtual void clientMoveAndDisplay();

    virtual void displayCallback();

    virtual void keyboardCallback(unsigned char key, int x, int y);

//    static DemoApplication* Create()
//    {
//        Hexapod* demo = new Hexapod(simpleMode_);
//        demo->myinit();
//        demo->initPhysics();
//        return demo;
//    }

    int simpleMode_;
    float servoPercentage[JOINT_COUNT];
    void setServoPercentValue(int rigId, int jointId, btScalar targetPercent);
    void setServoPercent(int rigId, int jointId, btScalar targetPercent, float deltaMs);
    void setMotorTargets(btScalar deltaTime);

};


#endif
