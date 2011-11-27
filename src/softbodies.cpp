#include "softbodies.h"
#include "util.h"

#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseInterface.h>
#include <osgbCollision/Utils.h>

void BulletSoftObject::init() {
    btDefaultCollisionConfiguration* m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
    btCollisionDispatcher* m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    getEnvironment()->bullet->dispatcher = m_dispatcher;
	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	btBroadphaseInterface* m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,32766);

	getEnvironment()->bullet->broadphase = m_broadphase;

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	getEnvironment()->bullet->solver = solver;

	//btSoftRigidDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,solver,m_collisionConfiguration);
	//getEnvironment()->bullet->dynamicsWorld = world;
	//m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback,this,true);


	//getEnvironment()->bullet->dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	//getEnvironment()->bullet->dynamicsWorld->setGravity(btVector3(0,0,-10));
	//m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);

    getEnvironment()->bullet->dynamicsWorld->addSoftBody(softBody.get());

    vertices = new osg::Vec3Array;
    normals = new osg::Vec3Array;
    geom = new osg::Geometry;
    geom->setDataVariance(osg::Object::DYNAMIC);
    geom->setUseDisplayList(false);
    geom->setUseVertexBufferObjects(true);
    geom->setVertexArray(vertices.get());
    geom->setNormalArray(normals.get());
    geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom);
    transform = new osg::MatrixTransform;
    transform->addChild(geode);
    getEnvironment()->osg->root->addChild(transform);
}

void BulletSoftObject::preDraw() {
#if 0
btSoftBodyHelpers::DrawFrame(softBody.get(), getEnvironment()->osg->softBodyDrawer.get());
btSoftBodyHelpers::Draw(softBody.get(), getEnvironment()->osg->softBodyDrawer.get(),
getEnvironment()->bullet->dynamicsWorld->getDrawFlags());
#endif
    transform->setMatrix(osgbCollision::asOsgMatrix(softBody->getWorldTransform()));

    if (geom->getNumPrimitiveSets() > 0)
        geom->removePrimitiveSet(0); // there should only be one
    const btSoftBody::tFaceArray &faces = softBody->m_faces;
    vertices->clear();
    normals->clear();

    for (int i = 0; i < faces.size(); ++i) {
        vertices->push_back(util::toOSGVector(faces[i].m_n[0]->m_x));
        normals->push_back(util::toOSGVector(faces[i].m_n[0]->m_n));

        vertices->push_back(util::toOSGVector(faces[i].m_n[1]->m_x));
        normals->push_back(util::toOSGVector(faces[i].m_n[1]->m_n));

        vertices->push_back(util::toOSGVector(faces[i].m_n[2]->m_x));
        normals->push_back(util::toOSGVector(faces[i].m_n[2]->m_n));
    }
    vertices->dirty();
    normals->dirty();
    geom->dirtyBound();
    geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, vertices->size()));
}

void BulletSoftObject::destroy() {
    getEnvironment()->bullet->dynamicsWorld->removeSoftBody(softBody.get());
}
