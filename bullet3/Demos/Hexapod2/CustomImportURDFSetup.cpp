
#include "CustomImportURDFSetup.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../../Demos3/ImportSTLDemo/LoadMeshFromSTL.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "Bullet3Common/b3FileUtils.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"//to create a tesselation of a generic btConvexShape

#include "urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h"

#include <iostream>
#include <fstream>

using namespace urdf;

static int bodyCollisionFilterGroup=btBroadphaseProxy::CharacterFilter;
static int bodyCollisionFilterMask=btBroadphaseProxy::AllFilter&(~btBroadphaseProxy::CharacterFilter);
static bool enableConstraints = true;//false;

#define MAX_NUM_MOTORS 1024

struct ImportUrdfInternalData
{
        ImportUrdfInternalData(): m_numMotors(0)
        {
        }

        btScalar m_motorTargetVelocities[MAX_NUM_MOTORS];
	      btMultiBodyJointMotor* m_jointMotors[MAX_NUM_MOTORS];
        int m_numMotors;
};

CustomImportURDFSetup::CustomImportURDFSetup()
{
        m_data = new ImportUrdfInternalData;
}

CustomImportURDFSetup::~CustomImportURDFSetup()
{
        delete m_data;
}

static btVector4 colors[4] =
{
        btVector4(1,0,0,1),
        btVector4(0,1,0,1),
        btVector4(0,1,1,1),
        btVector4(1,1,0,1),
};


btVector3 selectColor()
{

        static int curColor = 0;
        btVector4 color = colors[curColor];
        curColor++;
        curColor&=3;
        return color;
}

void CustomImportURDFSetup::setFileName(const char* urdfFileName)
{
    memcpy(m_fileName,urdfFileName,strlen(urdfFileName)+1);
}

void printTree(my_shared_ptr<const Link> link,int level = 0)
{
    level+=2;
    int count = 0;
    for (std::vector<my_shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    {
        if (*child)
        {
            for(int j=0;j<level;j++) std::cout << "  "; //indent
            std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << std::endl;
            // first grandchild
            printTree(*child,level);
        }
        else
        {
            for(int j=0;j<level;j++) std::cout << " "; //indent
            std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
        }
    }

}


struct URDF_LinkInformation
{
    const Link* m_thisLink;
    int m_linkIndex;
    //int m_parentIndex;

    btTransform m_localInertialFrame;
    //btTransform m_localVisualFrame;
    btTransform m_bodyWorldTransform;
    btVector3 m_localInertiaDiagonal;
    btScalar m_mass;

    btCollisionShape* m_collisionShape;
    btRigidBody* m_bulletRigidBody;

    URDF_LinkInformation(): m_thisLink(0),
                            m_linkIndex(-2),
                            //m_parentIndex(-2),
                            m_collisionShape(0),
                            m_bulletRigidBody(0)
    {

    }

    virtual ~URDF_LinkInformation()
    {
        printf("~\n");
    }
};

struct URDF_JointInformation
{

};


struct URDF2BulletMappings
{
    btHashMap<btHashPtr, URDF_LinkInformation*> m_link2rigidbody;

    btAlignedObjectArray<btScalar>			m_linkMasses;

    bool m_createMultiBody;
    int m_totalNumJoints;
    btMultiBody* m_bulletMultiBody;

    URDF2BulletMappings(): m_createMultiBody(false),
                           m_totalNumJoints(0),
                           m_bulletMultiBody(0)
    {

    }
};

enum MyFileType
{
    FILE_STL=1,
};

void convertURDFToVisualShape(const Visual* visual, const char* pathPrefix, const btTransform& visualTransform, btAlignedObjectArray<GLInstanceVertex>& verticesOut, btAlignedObjectArray<int>& indicesOut)
{
    GLInstanceGraphicsShape* glmesh = 0;

    btConvexShape* convexColShape = 0;

    switch (visual->geometry->type)
    {
        case Geometry::CYLINDER:
        {
            printf("processing a cylinder\n");
            urdf::Cylinder* cyl = (urdf::Cylinder*)visual->geometry.get();
            btAlignedObjectArray<btVector3> vertices;

            int numSteps = 32;
            for (int i = 0; i<numSteps; i++)
            {
                btVector3 vert(cyl->radius*btSin(SIMD_2_PI*(float(i) / numSteps)), cyl->radius*btCos(SIMD_2_PI*(float(i) / numSteps)), cyl->length / 2.);
                vertices.push_back(vert);
                vert[2] = -cyl->length / 2.;
                vertices.push_back(vert);
            }

            btConvexHullShape* cylZShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
            cylZShape->setMargin(0.001);
            convexColShape = cylZShape;
            break;
        }
        case Geometry::BOX:
        {
            printf("processing a box\n");
            urdf::Box* box = (urdf::Box*)visual->geometry.get();
            btVector3 extents(box->dim.x, box->dim.y, box->dim.z);
            btBoxShape* boxShape = new btBoxShape(extents*0.5f);
            convexColShape = boxShape;
            convexColShape->setMargin(0.001);
            break;
        }
        case Geometry::SPHERE:
        {
            printf("processing a sphere\n");
            urdf::Sphere* sphere = (urdf::Sphere*)visual->geometry.get();
            btScalar radius = sphere->radius;
            btSphereShape* sphereShape = new btSphereShape(radius);
            convexColShape = sphereShape;
            convexColShape->setMargin(0.001);
            break;
        }
        case Geometry::MESH:
        {
            if (visual->name.length())
            {
                printf("visual->name=%s\n", visual->name.c_str());
            }

            if (visual->geometry)
            {
                const urdf::Mesh* mesh = (const urdf::Mesh*) visual->geometry.get();
                if (mesh->filename.length())
                {
                    const char* filename = mesh->filename.c_str();
                    printf("mesh->filename=%s\n", filename);
                    char fullPath[1024];
                    int fileType = 0;
                    sprintf(fullPath, "%s%s", pathPrefix, filename);
                    b3FileUtils::toLower(fullPath);

                    if (strstr(fullPath, ".stl"))
                    {
                        fileType = FILE_STL;
                    }


                    sprintf(fullPath, "%s%s", pathPrefix, filename);
                    FILE* f = fopen(fullPath, "rb");
                    if (f)
                    {
                        fclose(f);
                        switch (fileType)
                        {
                            case FILE_STL:
                            {
                                glmesh = LoadMeshFromSTL(fullPath);
                                break;
                            }
                            default:
                            {
                            }
                        }

                        if (glmesh && (glmesh->m_numvertices>0))
                        {

                        }
                        {
                            printf("issue extracting mesh from COLLADA/STL file %s\n", fullPath);
                        }

                    }
                    else
                    {
                        printf("mesh geometry not found %s\n", fullPath);
                    }

            }
        }
            break;
        }
        default:
        {
            printf("Error: unknown visual geometry type\n");
        }
    }

    //if we have a convex, tesselate into localVertices/localIndices
    if (convexColShape)
    {
        btShapeHull* hull = new btShapeHull(convexColShape);
        hull->buildHull(0.0);

        //	int strideInBytes = 9*sizeof(float);
        int numVertices = hull->numVertices();
        int numIndices = hull->numIndices();


        glmesh = new GLInstanceGraphicsShape;
        glmesh->m_indices = new b3AlignedObjectArray<int>();
        glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();


        for (int i = 0; i < numVertices; i++)
        {
            GLInstanceVertex vtx;
            btVector3 pos = hull->getVertexPointer()[i];
            vtx.xyzw[0] = pos.x();
            vtx.xyzw[1] = pos.y();
            vtx.xyzw[2] = pos.z();
            vtx.xyzw[3] = 1.f;
            pos.normalize();
            vtx.normal[0] = pos.x();
            vtx.normal[1] = pos.y();
            vtx.normal[2] = pos.z();
            vtx.uv[0] = 0.5f;
            vtx.uv[1] = 0.5f;
            glmesh->m_vertices->push_back(vtx);
        }

        for (int i = 0; i < numIndices; i++)
        {
            glmesh->m_indices->push_back(hull->getIndexPointer()[i]);
        }

        glmesh->m_numvertices = glmesh->m_vertices->size();
        glmesh->m_numIndices = glmesh->m_indices->size();

        delete convexColShape;
        convexColShape = 0;
    }

    if (glmesh && glmesh->m_numIndices>0 && glmesh->m_numvertices >0)
    {
        int baseIndex = verticesOut.size();

        for (int i = 0; i < glmesh->m_indices->size(); i++)
        {
            indicesOut.push_back(glmesh->m_indices->at(i) + baseIndex);
        }

        for (int i = 0; i < glmesh->m_vertices->size(); i++)
        {
            GLInstanceVertex& v = glmesh->m_vertices->at(i);
            btVector3 vert(v.xyzw[0],v.xyzw[1],v.xyzw[2]);
            btVector3 vt = visualTransform*vert;
            v.xyzw[0] = vt[0];
            v.xyzw[1] = vt[1];
            v.xyzw[2] = vt[2];
            btVector3 triNormal(v.normal[0],v.normal[1],v.normal[2]);
            triNormal = visualTransform.getBasis()*triNormal;
            v.normal[0] = triNormal[0];
            v.normal[1] = triNormal[1];
            v.normal[2] = triNormal[2];
            verticesOut.push_back(v);
        }
    }
}

btCollisionShape* convertURDFToCollisionShape(const Collision* visual, const char* pathPrefix)
{
    btCollisionShape* shape = 0;

    switch (visual->geometry->type)
    {
        case Geometry::CYLINDER:
        {
            printf("processing a cylinder\n");
            urdf::Cylinder* cyl = (urdf::Cylinder*)visual->geometry.get();
            btAlignedObjectArray<btVector3> vertices;

            int numSteps = 32;
            for (int i = 0; i<numSteps; i++)
            {
                btVector3 vert(cyl->radius*btSin(SIMD_2_PI*(float(i) / numSteps)), cyl->radius*btCos(SIMD_2_PI*(float(i) / numSteps)), cyl->length / 2.);
                vertices.push_back(vert);
                vert[2] = -cyl->length / 2.;
                vertices.push_back(vert);
            }

            btConvexHullShape* cylZShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
            cylZShape->setMargin(0.001);
            cylZShape->initializePolyhedralFeatures();

            shape = cylZShape;
            break;
        }
        case Geometry::BOX:
        {
            printf("processing a box\n");
            urdf::Box* box = (urdf::Box*)visual->geometry.get();
            btVector3 extents(box->dim.x, box->dim.y, box->dim.z);
            btBoxShape* boxShape = new btBoxShape(extents*0.5f);
            shape = boxShape;
            shape ->setMargin(0.001);
            break;
        }
        case Geometry::SPHERE:
        {
            printf("processing a sphere\n");
            urdf::Sphere* sphere = (urdf::Sphere*)visual->geometry.get();
            btScalar radius = sphere->radius;
            btSphereShape* sphereShape = new btSphereShape(radius);
            shape = sphereShape;
            shape ->setMargin(0.001);
            break;
        }
        case Geometry::MESH:
        {
            if (visual->name.length())
            {
                printf("visual->name=%s\n", visual->name.c_str());
            }

            if (visual->geometry)
            {
                const urdf::Mesh* mesh = (const urdf::Mesh*) visual->geometry.get();
                if (mesh->filename.length())
                {
                    const char* filename = mesh->filename.c_str();
                    printf("mesh->filename=%s\n", filename);
                    char fullPath[1024];
                    int fileType = 0;
                    sprintf(fullPath, "%s%s", pathPrefix, filename);
                    b3FileUtils::toLower(fullPath);

                    if (strstr(fullPath, ".stl"))
                    {
                        fileType = FILE_STL;
                    }


                    sprintf(fullPath, "%s%s", pathPrefix, filename);
                    FILE* f = fopen(fullPath, "rb");
                    if (f)
                    {
                        fclose(f);
                        GLInstanceGraphicsShape* glmesh = 0;

                        switch (fileType)
                        {
                            case FILE_STL:
                            {
                                glmesh = LoadMeshFromSTL(fullPath);
                                break;
                            }
                            default:
                            {
                            }
                        }


                        if (glmesh && (glmesh->m_numvertices>0))
                        {
                            printf("extracted %d verticed from STL file %s\n", glmesh->m_numvertices,fullPath);
                            btAlignedObjectArray<btVector3> convertedVerts;
                            convertedVerts.reserve(glmesh->m_numvertices);
                            for (int i=0;i<glmesh->m_numvertices;i++)
                            {
                                convertedVerts.push_back(btVector3(glmesh->m_vertices->at(i).xyzw[0],glmesh->m_vertices->at(i).xyzw[1],glmesh->m_vertices->at(i).xyzw[2]));
                            }
                            btConvexHullShape* cylZShape = new btConvexHullShape(&convertedVerts[0].getX(), convertedVerts.size(), sizeof(btVector3));
                            cylZShape->setMargin(0.001);
                            shape = cylZShape;
                        }
                        else
                        {
                            printf("issue extracting mesh from STL file %s\n", fullPath);
                        }

                    }
                    else
                    {
                        printf("mesh geometry not found %s\n",fullPath);
                    }


                }
            }
            break;
        }
        default:
        {
        printf("Error: unknown visual geometry type\n");
        }
    }
    return shape;
}

void URDFvisual2BulletCollisionShape(my_shared_ptr<const Link> link, GraphicsPhysicsBridge& gfxBridge, const btTransform& parentTransformInWorldSpace, btMultiBodyDynamicsWorld* world1, URDF2BulletMappings& mappings, const char* pathPrefix, ImportUrdfInternalData* data)
{
    btTransform linkTransformInWorldSpace;
    linkTransformInWorldSpace.setIdentity();

    btScalar mass = 0;
    btTransform inertialFrame;
    inertialFrame.setIdentity();
    const Link* parentLink = (*link).getParent();
    URDF_LinkInformation* pp = 0;
    int linkIndex =  mappings.m_linkMasses.size();
    btVector3 localInertiaDiagonal(0,0,0);

    int parentIndex = -1;

    if (parentLink)
    {
        parentIndex = parentLink->m_link_index;
        btAssert(parentIndex>=0);
    }

    URDF_LinkInformation** ppRigidBody = mappings.m_link2rigidbody.find(parentLink);
    if (ppRigidBody)
    {
        pp = (*ppRigidBody);
        btTransform tr = pp->m_bodyWorldTransform;
        printf("rigidbody origin (COM) of link(%s) parent(%s): %f,%f,%f\n",(*link).name.c_str(), parentLink->name.c_str(), tr.getOrigin().x(), tr.getOrigin().y(), tr.getOrigin().z());
    }

    (*link).m_link_index = linkIndex;

    if ((*link).inertial)
    {
        mass = (*link).inertial->mass;
        localInertiaDiagonal.setValue((*link).inertial->ixx,(*link).inertial->iyy,(*link).inertial->izz);
        inertialFrame.setOrigin(btVector3((*link).inertial->origin.position.x,(*link).inertial->origin.position.y,(*link).inertial->origin.position.z));
        inertialFrame.setRotation(btQuaternion((*link).inertial->origin.rotation.x,(*link).inertial->origin.rotation.y,(*link).inertial->origin.rotation.z,(*link).inertial->origin.rotation.w));
    }

    btTransform parent2joint;
    parent2joint.setIdentity();

    if ((*link).parent_joint)
    {
        const urdf::Vector3 pos = (*link).parent_joint->parent_to_joint_origin_transform.position;
        const urdf::Rotation orn = (*link).parent_joint->parent_to_joint_origin_transform.rotation;

        parent2joint.setOrigin(btVector3(pos.x,pos.y,pos.z));
        parent2joint.setRotation(btQuaternion(orn.x,orn.y,orn.z,orn.w));
        linkTransformInWorldSpace =parentTransformInWorldSpace*parent2joint;
    }
    else
    {
        linkTransformInWorldSpace = parentTransformInWorldSpace;
    }

    printf("converting visuals of link %s", link->name.c_str());

    btAlignedObjectArray<GLInstanceVertex> vertices;
    btAlignedObjectArray<int> indices;
    btTransform startTrans; startTrans.setIdentity();
    int graphicsIndex = -1;

    for (int v = 0; v < (int)link->visual_array.size(); v++)
    {
        const Visual* vis = link->visual_array[v].get();
        btVector3 childPos(vis->origin.position.x, vis->origin.position.y, vis->origin.position.z);
        btQuaternion childOrn(vis->origin.rotation.x, vis->origin.rotation.y, vis->origin.rotation.z, vis->origin.rotation.w);
        btTransform childTrans;
        childTrans.setOrigin(childPos);
        childTrans.setRotation(childOrn);

        convertURDFToVisualShape(vis, pathPrefix, inertialFrame.inverse()*childTrans, vertices, indices);
    }

    if (vertices.size() && indices.size())
    {
        graphicsIndex  = gfxBridge.registerGraphicsShape(&vertices[0].xyzw[0], vertices.size(), &indices[0], indices.size());
    }

    btCompoundShape* compoundShape = new btCompoundShape();
    compoundShape->setMargin(0.001);
    for (int v=0;v<(int)link->collision_array.size();v++)
    {
        const Collision* col = link->collision_array[v].get();
        btCollisionShape* childShape = convertURDFToCollisionShape(col ,pathPrefix);
        if (childShape)
        {
            btVector3 childPos(col->origin.position.x, col->origin.position.y, col->origin.position.z);
            btQuaternion childOrn(col->origin.rotation.x, col->origin.rotation.y, col->origin.rotation.z, col->origin.rotation.w);
            btTransform childTrans;
            childTrans.setOrigin(childPos);
            childTrans.setRotation(childOrn);
            compoundShape->addChildShape(inertialFrame.inverse()*childTrans,childShape);
        }
    }

    if (compoundShape)
    {
        btVector3 color = selectColor();

        btTransform inertialFrameInWorldSpace = linkTransformInWorldSpace*inertialFrame;
        URDF_LinkInformation* linkInfo = new URDF_LinkInformation;

        btRigidBody::btRigidBodyConstructionInfo rbci(mass, 0, compoundShape, localInertiaDiagonal);
        rbci.m_startWorldTransform = inertialFrameInWorldSpace;
        linkInfo->m_bodyWorldTransform = inertialFrameInWorldSpace;//visualFrameInWorldSpace
        //rbci.m_startWorldTransform = inertialFrameInWorldSpace;//linkCenterOfMass;
        btRigidBody* body = new btRigidBody(rbci);
        world1->addRigidBody(body, bodyCollisionFilterGroup, bodyCollisionFilterMask);

        compoundShape->setUserIndex(graphicsIndex);

        gfxBridge.createRigidBodyGraphicsObject(body, color);
        linkInfo->m_bulletRigidBody = body;

        linkInfo->m_collisionShape = compoundShape;
        linkInfo->m_localInertiaDiagonal = localInertiaDiagonal;
        linkInfo->m_mass = mass;
        //linkInfo->m_localVisualFrame =visual_frame;
        linkInfo->m_localInertialFrame =inertialFrame;
        linkInfo->m_thisLink = link.get();
        const Link* p = link.get();
        mappings.m_link2rigidbody.insert(p, linkInfo);

        //create a joint if necessary
        if ((*link).parent_joint && pp)
        {
            btAssert(pp);

            const Joint* pj = (*link).parent_joint.get();
            btTransform offsetInA,offsetInB;
            static bool once = true;

            offsetInA.setIdentity();
            static bool toggle=false;

            //offsetInA = pp->m_localVisualFrame.inverse()*parent2joint;
            offsetInA = pp->m_localInertialFrame.inverse()*parent2joint;

            offsetInB.setIdentity();
            //offsetInB = visual_frame.inverse();
            offsetInB = inertialFrame.inverse();


            bool disableParentCollision = true;
            btVector3 jointAxis(pj->axis.x,pj->axis.y,pj->axis.z);
            switch (pj->type)
            {
                case Joint::FIXED:
                {
                    printf("Fixed joint\n");

                    btMatrix3x3 rm(offsetInA.getBasis());
                    btScalar y,p,r;
                    rm.getEulerZYX(y,p,r);
                    printf("y=%f,p=%f,r=%f\n", y,p,r);

                    btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*pp->m_bulletRigidBody, *linkInfo->m_bulletRigidBody, offsetInA, offsetInB);
                    dof6->setLinearLowerLimit(btVector3(0,0,0));
                    dof6->setLinearUpperLimit(btVector3(0,0,0));

                    dof6->setAngularLowerLimit(btVector3(0,0,0));
                    dof6->setAngularUpperLimit(btVector3(0,0,0));

                    if (enableConstraints)
                        world1->addConstraint(dof6,true);

                    break;
                }
                case Joint::CONTINUOUS:
                case Joint::REVOLUTE:
                {
                    //only handle principle axis at the moment,
                    //@todo(erwincoumans) orient the constraint for non-principal axis
                    printf("Hinge joint (btGeneric6DofSpring2Constraint)\n");
                    btVector3 axis(pj->axis.x,pj->axis.y,pj->axis.z);
                    int principleAxis = axis.closestAxis();
                    switch (principleAxis)
                    {
                        case 0:
                        {
                            btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*pp->m_bulletRigidBody, *linkInfo->m_bulletRigidBody, offsetInA, offsetInB,RO_ZYX);
                            dof6->setLinearLowerLimit(btVector3(0,0,0));
                            dof6->setLinearUpperLimit(btVector3(0,0,0));

                            dof6->setAngularUpperLimit(btVector3(-1,0,0));
                            dof6->setAngularLowerLimit(btVector3(1,0,0));

                            if (enableConstraints)
                                world1->addConstraint(dof6,true);
                            break;
                        }
                        case 1:
                        {
                            btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*pp->m_bulletRigidBody, *linkInfo->m_bulletRigidBody, offsetInA, offsetInB,RO_XZY);
                            dof6->setLinearLowerLimit(btVector3(0,0,0));
                            dof6->setLinearUpperLimit(btVector3(0,0,0));

                            dof6->setAngularUpperLimit(btVector3(0,-1,0));
                            dof6->setAngularLowerLimit(btVector3(0,1,0));

                            if (enableConstraints)
                                world1->addConstraint(dof6,true);
                            break;
                        }
                        case 2:
                        default:
                        {
                            btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*pp->m_bulletRigidBody, *linkInfo->m_bulletRigidBody, offsetInA, offsetInB,RO_XYZ);
                            dof6->setLinearLowerLimit(btVector3(0,0,0));
                            dof6->setLinearUpperLimit(btVector3(0,0,0));

                            dof6->setAngularUpperLimit(btVector3(0,0,-1));
                            dof6->setAngularLowerLimit(btVector3(0,0,0));

                            if (enableConstraints)
                                world1->addConstraint(dof6,true);
                        }
                    };
                    printf("Revolute/Continuous joint\n");

                    break;
                }
                case Joint::PRISMATIC:
                {
                    printf("Slider joint (btGeneric6DofSpring2Constraint)\n");
                    btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*pp->m_bulletRigidBody, *linkInfo->m_bulletRigidBody, offsetInA, offsetInB);
                    //todo(erwincoumans) for now, we only support principle axis along X, Y or Z
                    btVector3 axis(pj->axis.x,pj->axis.y,pj->axis.z);
                    int principleAxis = axis.closestAxis();
                    switch (principleAxis)
                    {
                        case 0:
                        {
                            dof6->setLinearLowerLimit(btVector3(pj->limits->lower,0,0));
                            dof6->setLinearUpperLimit(btVector3(pj->limits->upper,0,0));
                            break;
                        }
                        case 1:
                        {
                            dof6->setLinearLowerLimit(btVector3(0,pj->limits->lower,0));
                            dof6->setLinearUpperLimit(btVector3(0,pj->limits->upper,0));
                            break;
                        }
                        case 2:
                        default:
                        {
                            dof6->setLinearLowerLimit(btVector3(0,0,pj->limits->lower));
                            dof6->setLinearUpperLimit(btVector3(0,0,pj->limits->upper));
                        }
                    };

                    dof6->setAngularLowerLimit(btVector3(0,0,0));
                    dof6->setAngularUpperLimit(btVector3(0,0,0));
                    if (enableConstraints)
                        world1->addConstraint(dof6,true);

                    printf("Prismatic\n");
                    break;
                }
                default:
                {
                    printf("Error: unsupported joint type in URDF (%d)\n", pj->type);
                }
            }
        }
    }

    mappings.m_linkMasses.push_back(mass);

    for (std::vector<my_shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    {
        if (*child)
        {
            URDFvisual2BulletCollisionShape(*child,gfxBridge, linkTransformInWorldSpace, world1,mappings,pathPrefix, data);
        }
        else
        {
            std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
        }
    }
}


void CustomImportURDFSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
    int upAxis = 2;
    gfxBridge.setUpAxis(2);

    this->createEmptyDynamicsWorld();
    //m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
    gfxBridge.createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode( btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawAabb );//+btIDebugDraw::DBG_DrawConstraintLimits);

    btVector3 gravity(0,0,0);
    gravity[upAxis]=-9.8;

    m_dynamicsWorld->setGravity(gravity);
    //int argc=0;
    char relativeFileName[1024];

    b3FileUtils fu;
    printf("m_fileName=%s\n", m_fileName);
    bool fileFound = fu.findFile(m_fileName, relativeFileName, 1024);

    std::string xml_string;
    char pathPrefix[1024];
    pathPrefix[0] = 0;

    if (!fileFound){
        std::cerr << "URDF file not found!" << std::endl;
        return ;
    }
    else
    {
        int maxPathLen = 1024;
        fu.extractPath(relativeFileName,pathPrefix,maxPathLen);

        std::fstream xml_file(relativeFileName, std::fstream::in);
        while ( xml_file.good() )
        {
            std::string line;
            std::getline( xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
    }

    my_shared_ptr<ModelInterface> robot = parseURDF(xml_string);

    if (!robot){
        std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
        return ;
    }

    std::cout << "robot name is: " << robot->getName() << std::endl;

    // get info from parser
    std::cout << "---------- Successfully Parsed XML ---------------" << std::endl;
    // get root link
    my_shared_ptr<const Link> root_link=robot->getRoot();
    if (!root_link)
        return ;

    std::cout << "root Link: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << std::endl;

    // print entire tree
    printTree(root_link);
    btTransform identityTrans;
    identityTrans.setIdentity();

    int numJoints = (*robot).m_numJoints;

    URDF2BulletMappings mappings;
    mappings.m_createMultiBody = false;
    mappings.m_totalNumJoints = numJoints;
    URDFvisual2BulletCollisionShape(root_link, gfxBridge, identityTrans,m_dynamicsWorld,mappings,pathPrefix,m_data);

    printf("numJoints/DOFS = %d\n", numJoints);

    bool createGround=false;
    if (createGround)
    {
        btVector3 groundHalfExtents(20,20,20);
        groundHalfExtents[upAxis]=1.f;
        btBoxShape* box = new btBoxShape(groundHalfExtents);
        box->initializePolyhedralFeatures();

        gfxBridge.createCollisionShapeGraphicsObject(box);
        btTransform start; start.setIdentity();
        btVector3 groundOrigin(0,0,0);
        groundOrigin[upAxis]=-1.5;
        start.setOrigin(groundOrigin);
        btRigidBody* body =  createRigidBody(0,start,box);
        //m_dynamicsWorld->removeRigidBody(body);
        // m_dynamicsWorld->addRigidBody(body,2,1);
        btVector3 color(0.5,0.5,0.5);
        gfxBridge.createRigidBodyGraphicsObject(body,color);
    }

    ///this extra stepSimulation call makes sure that all the btMultibody transforms are properly propagates.
    m_dynamicsWorld->stepSimulation(1. / 240., 0);// 1., 10, 1. / 240.);
}

void CustomImportURDFSetup::stepSimulation(float deltaTime)
{
    if (m_dynamicsWorld)
    {
        //set the new target velocities
        for (int i=0;i<m_data->m_numMotors;i++)
        {
            m_data->m_jointMotors[i]->setVelocityTarget(m_data->m_motorTargetVelocities[i]);
        }
        //the maximal coordinates/iterative MLCP solver requires a smallish timestep to converge
        int actualSteps = m_dynamicsWorld->stepSimulation(deltaTime,10,1./240.);
        
        //int actualSteps = m_dynamicsWorld->stepSimulation(deltaTime,1000,1./3000.f);//240.);
        //printf("deltaTime %f took actualSteps = %d\n",deltaTime,actualSteps);
    }
}

