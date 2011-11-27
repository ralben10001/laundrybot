
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///btSoftBody implementation by Nathanael Presson

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include "../GimpactTestDemo/BunnyMesh.h"
#include "../GimpactTestDemo/TorusMesh.h"
#include <stdio.h> //printf debugging
#include "LinearMath/btConvexHull.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "SoftDemo.h"
#include "GL_ShapeDrawer.h"
#include "GLDebugFont.h"
#include "GlutStuff.h"
#include <iostream>
#include <fstream>

int* nodestofaces;

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;
btSoftBody::Node* m_nodes[5];
int numnodes = 0;
static bool sDemoMode = false;

const int maxProxies = 32766;
const int maxOverlap = 65535;

static btVector3*	gGroundVertices=0;
static int*	gGroundIndices=0;
static btBvhTriangleMeshShape* trimeshShape =0;
static btRigidBody* staticBody = 0;
static float waveheight = 5.f;
static bool MakeVideo = false;
const float TRIANGLE_SIZE=8.f;
int		current_demo=29;
btScalar		xs[3];
btScalar		ys[3];
btScalar		zs[3];
#define DEMO_MODE_TIMEOUT 15.f //15 seconds for each demo
unsigned char image[3*64*64*sizeof(char)];
float clothlength;
float clothheight;
#ifdef _DEBUG
#define dim 15
const int gNumObjects = 1;
#else
#define dim 31
const int gNumObjects = 1;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cp
#endif
btRigidBody* box[2];
const int maxNumObjects = 32760;

#define CUBE_HALF_EXTENTS 1.5
#define EXTRA_HEIGHT -10.f
std::ofstream goalpositions[2];

btVector3 findCenterOfMass(btSoftBody* psb) {
	btVector3 mid(0,0,0);
	for (int i = 0; i < psb->m_nodes.size(); i++)
		mid += psb->m_nodes[i].m_x;
	return mid/psb->m_nodes.size();
}

//
void SoftDemo::createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos )
{
	btTransform trans;
	trans.setIdentity();

	for(int i=0; i<size; i++)
	{
		// This constructs a row, from left to right
		int rowSize = size - i;
		for(int j=0; j< rowSize; j++)
		{
			btVector3 pos;
			pos.setValue(
				-rowSize * halfCubeSize + halfCubeSize + j * 2.0f * halfCubeSize,
				halfCubeSize + i * halfCubeSize * 2.0f,
				zPos);

			trans.setOrigin(pos);
			btScalar mass = 1.f;

			btRigidBody* body = 0;
			body = localCreateRigidBody(mass,trans,boxShape);

		}
	}
}
void pickLowest(btSoftBody* psb) {
		
		btScalar min = -1;
		int index = -1;
		btSoftBody::Node* m_node = 0;
		btSoftBody::Node* min_node = 0;
		for (int i = 0; i < psb->m_nodes.size(); i++) {
			//btSoftBody::Face&	f=psb->m_faces[i];
			m_node=&psb->m_nodes[i];
			btVector3 coord = m_node->m_x;
			btScalar ycoord = coord.getY();
			if (min == -1 || ycoord < min) {
				min = ycoord;
				min_node = m_node;
			}
		}
		m_node				=	min_node;
		m_nodes[numnodes] = min_node;
		btVector3 vel(0,0,0);
		psb->setVelocity(vel);
		numnodes ^= 1;
		return;
}

void pickLastAcross(btSoftBody* psb, int direction) {
		btScalar min = -1;
		btSoftBody::Node* m_node = 0;
		btSoftBody::Node* min_node = 0;
		for (int i = 0; i < psb->m_nodes.size(); i++) {
			//btSoftBody::Face&	f=psb->m_faces[i];
			
			m_node=&psb->m_nodes[i];
			btVector3 coord = m_node->m_x;
			btScalar pos[3];
			pos[0] = coord.getX()*direction;
			pos[1] = coord.getY();
			pos[2] = coord.getZ();
			if (min == -1 || pos[abs(direction)-1] > min) {
				min = pos[abs(direction)-1];
				min_node=m_node;
			}
		}
		xs[numnodes] = min*direction;
		m_node				=	min_node;
		m_nodes[numnodes] = m_node;
		btVector3 vel(0,0,0);
		psb->setVelocity(vel);
		numnodes ^= 1;
		return;
}

void dragovertable(int direction) {
	if (direction == 0) {
		xs[0] -= 0.02;
		xs[1] -= 0.02;
	}
	else if (direction == 1) {
		xs[0] += 0.02;
		xs[1] += 0.02;
	}
	else if (direction == 2)
	{
		zs[0] -=0.02;
		zs[1] -=0.02;
	}
	else if (direction == 3)
	{
		zs[0] +=0.02;
		zs[1] +=0.02;
	}
}

void lift() {
	ys[0] += 10;
}

void drop() {
	ys[0] -= 10;
}

void bringforward() {
	zs[0] -= 20;
	zs[1] -= 20;
	//ys[0] -= 1;
	//ys[1] -= 1;
}

void fold(btSoftBody* psb) {
	btVector3 corner1 = m_nodes[0]->m_x;
	btVector3 corner2 = m_nodes[1]->m_x;
	btSoftBody::Node* m_node;
	btVector3 coord;
	btScalar maxing;
	btScalar max = 0;
	int index = -1;
	for (int i = 0; i < psb->m_faces.size(); i++) {
		m_node=psb->m_faces[i].m_n[0];
		coord = m_node->m_x;
		maxing = corner1.distance2(coord);
		if (maxing > max) {
			index = i;
			max = maxing;
		}
	}
	m_node = psb->m_faces[index].m_n[0];
	m_nodes[3] = m_node;
	max = 0;
	for (int i = 0; i < psb->m_faces.size(); i++) {
		m_node=psb->m_faces[i].m_n[0];
		coord = m_node->m_x;
		maxing = corner2.distance2(coord);
		if (maxing > max) {
			index = i;
			max = maxing;
		}
	}
	m_node = psb->m_faces[index].m_n[0];
	m_nodes[2] = m_node;
}

void liftandcalcheight(btSoftBody* psb) {
	btSoftBody::Node* m_node = 0;
	btSoftBody::Node* min_node = 0;
	btScalar min = -1;
	for (int i = 0; i < psb->m_nodes.size(); i++) {
		//btSoftBody::Face&	f=psb->m_faces[i];
			
		m_node=&psb->m_nodes[i];
		btVector3 coord = m_node->m_x;
		btScalar pos;
		pos = coord.getY();
		if (min == -1 || pos < min) {
			min = pos;
			min_node=m_node;
		}
	}
	if (min+5 < 0.1) {
		ys[0] += 0.02;
		ys[1] += 0.02;
	}
	else
		clothheight = ys[0] + 5;

}

void gfold(int dir) {
	if (ys[0] + 5 > clothheight/2 && dir == 0) {
		zs[0] -= 0.02;
		zs[1] -= 0.02;
		ys[0] -= 0.02;
		ys[1] -= 0.02;
	}
	if (ys[0] + 5 > 0.1 && dir == 1) {
		zs[0] += 0.02;
		zs[1] += 0.02;
		ys[0] -= 0.02;
		ys[1] -= 0.02;
	}
}

void center() {
	float dx = xs[1] - xs[0];
	float dz = zs[1] - zs[2];
	clothlength = sqrt(pow(dx,2) + pow(dz,2));
	xs[1] = clothlength/2;
	xs[0] = -clothlength/2;
	zs[1] = 0;
	zs[0] = 0;
}

void fold2(btSoftBody* psb) {
	xs[1] = xs[0];
	ys[1] = ys[0];
}
////////////////////////////////////

extern int gNumManifold;
extern int gOverlappingPairs;
static float currtime;
static int iterations;
static int currcount = 0;
static float theta;
static int currstep = 0;
///for mouse picking
void pickingPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	
	SoftDemo* softDemo = (SoftDemo*)world->getWorldUserInfo();
	btSoftBody* psb = softDemo->getSoftDynamicsWorld()->getSoftBodyArray()[0];
	/*if (iterations == 1) {
	btVector3* oldvels = new btVector3[psb->m_nodes.size()];
	psb->m_nodes[0].m_v = btVector3(0,0.166667,0);
	for (int i = 0; i < psb->m_nodes.size(); i++)
		oldvels[i] = psb->m_nodes[i].m_v;
	psb->addVelocity(btVector3(0,-0.166667,0));
	psb->applyForces();
	btVector3 changevel = psb->m_nodes[0].m_v - oldvels[0];
	delete oldvels;
	}*/

	
	int				x=softDemo->m_lastmousepos[0];
	int				y=softDemo->m_lastmousepos[1];
	btVector3			rayFrom=softDemo->getCameraPosition();
	btVector3			rayTo=softDemo->getRayTo(x,y);
	btVector3			rayDir=(rayTo-rayFrom).normalized();
	btVector3			N=(softDemo->getCameraTargetPosition()-softDemo->getCameraPosition()).normalized();
	btScalar			O=btDot(softDemo->m_impact,N);
	btScalar			den=btDot(N,rayDir);
	if((den*den)>0)
	{
		const btScalar			num=O-btDot(N,rayFrom);
		const btScalar			hit=num/den;
		if((hit>0)&&(hit<1500))
		{				
			softDemo->m_goal=rayFrom+rayDir*hit;
		}				
	}
	btVector3				delta;
	static btScalar	maxdrag=10;
	if(softDemo->m_drag && m_nodes[0] == 0)	{	
		iterations = 0;
		delta=softDemo->m_goal-softDemo->m_node->m_x;
			

		if(delta.length2()>(maxdrag*maxdrag))
		{
			delta=delta.normalized()*maxdrag;
		}
		softDemo->m_node->m_v=delta/timeStep;
	}
	btScalar xg, yg, zg;
	for (int i = 0; i < 4; i++) {
		if (m_nodes[i] == 0) {
			if (box[i]) {
				box[i]->setLinearVelocity(btVector3(0,0,0));
				box[i]->setAngularVelocity(btVector3(0,0,0));
			}
			continue;
		}
		xg=xs[i%2];
		yg=ys[i%2];
		zg=zs[i%2];
	if (box)
	{
		btVector3 mid = findCenterOfMass(psb);
		box[i]->setActivationState(ACTIVE_TAG);
		btVector3 goalpos = btVector3(4,5,0);
		if (iterations > 0)
			goalpos = btVector3(m_nodes[i]->m_x);
		btVector3 relPos = goalpos - mid;
		theta = -atan(relPos.x()/relPos.y());
		btTransform trans(btMatrix3x3(cos(theta),0-sin(theta),0,sin(theta),cos(theta),0,0,0,1),goalpos);
		trans.setOrigin(goalpos);
		box[i]->setWorldTransform(trans);
		btMatrix3x3 mat = trans.getBasis();
		btVector3 orig = trans.getOrigin();
	}
		softDemo->m_goal=btVector3(xg,yg,zg);
		btScalar realx = softDemo->m_goal.getX();
		btScalar realy = softDemo->m_goal.getY();
		btScalar realz = softDemo->m_goal.getZ();
		delta = softDemo->m_goal-m_nodes[i]->m_x;
		if(delta.length2()>(1))
		{
			delta=delta.normalized()*maxdrag;
		}/*
		if (delta.getY() > 3.0)
			delta.setY(3.0);
		if (delta.getY() < 0)
			delta.setY(0);
		if (delta.getX() > 2.0)
			delta.setX(2.0);
		if (delta.getX() < -2.0)
			delta.setX(-2.0);
		if (delta.getZ() >  2.0)
			delta.setZ( 2.0);
		if (delta.getZ() <  -2.0)
			delta.setZ(-2.0);*/
		m_nodes[i]->m_v+=delta/timeStep;
	}
	if (m_nodes[4] != 0) {
		xg=xs[2];
		yg=ys[2];
		rayFrom=softDemo->getCameraPosition();
		rayTo=softDemo->getRayTo(x,y);
		rayDir=(rayTo-rayFrom).normalized();
		N=(softDemo->getCameraTargetPosition()-softDemo->getCameraPosition()).normalized();
		O=btDot(softDemo->m_impact,N);
		den=btDot(N,rayDir);
		if((den*den)>0)
		{
			const btScalar			num=O-btDot(N,rayFrom);
			const btScalar			hit=num/den;
			if((hit>0)&&(hit<1500))
			{				
				softDemo->m_goal=btVector3(xg,yg,zg);//rayFrom+rayDir*hit;
			}				
		}	
		delta = softDemo->m_goal-m_nodes[4]->m_x;
		if(delta.length2()>(maxdrag*maxdrag))
		{
			delta=delta.normalized()*maxdrag;
		}
		m_nodes[4]->m_v+=delta/timeStep;
	}
	
	currtime += timeStep;
	/*for (int i = 0; i < psb->m_nodes.size(); i++) {
		btScalar yvel = psb->m_nodes[i].m_v.getY();
		psb->m_nodes[i].m_v /= 1.1;
		psb->m_nodes[i].m_v.setY(yvel);
	}*/
	/*if (currtime > 2 && (currtime - floor(currtime/2)*2) < timeStep) {
		for (int i = 0; i < psb->m_nodes.size(); i++)
			psb->m_nodes[i].m_v*= 0;
	}*/
	if (iterations == 2 || iterations == 3) {
		dragovertable(0);
		if (currtime > 5)
			theta = 3.1415/2.0;
	}
	if (iterations == 4) {
		dragovertable(1);
	}
	
	if (iterations == 5) {
		dragovertable(2);
	}
	if (iterations == 7) 
		liftandcalcheight(psb);
	if (iterations == 8 )
		gfold(0);
	if (iterations == 9)
		gfold(1);
	if (currtime > 10) {
		if (iterations == 0) {
			pickLowest(psb);
			//lift();
			theta = -3.1415/4.0;
		}
		if (iterations == 1) {
			//drop();
			theta = 0;
		}
		if (iterations == 3){
			pickLastAcross(psb,1);
			m_nodes[0] = 0;
		}
		if (iterations == 4)
			pickLastAcross(psb,-1);
		if (iterations == 5)
			pickLastAcross(psb,3);
		if (iterations == 6)
			center();
		currtime = 0;
		iterations++;
	};

}




void SoftDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	renderme();

	glFlush();
	swapBuffers();
}


//
// ImplicitShape
//

//
struct	ImplicitSphere : btSoftBody::ImplicitFn
{
	btVector3	center;
	btScalar	sqradius;
	ImplicitSphere() {}
	ImplicitSphere(const btVector3& c,btScalar r) : center(c),sqradius(r*r) {}
	btScalar	Eval(const btVector3& x)
	{
		return((x-center).length2()-sqradius);
	}
};



//
// Random
//

static inline btScalar	UnitRand()
{
	return(rand()/(btScalar)RAND_MAX);
}

static inline btScalar	SignedUnitRand()
{
	return(UnitRand()*2-1);
}

static inline btVector3	Vector3Rand()
{
	const btVector3	p=btVector3(SignedUnitRand(),SignedUnitRand(),SignedUnitRand());
	return(p.normalized());
}

//
// Rb rain
//
static void	Ctor_RbUpStack(SoftDemo* pdemo,int currcount)
{
	float				mass=10;

	btCompoundShape* cylinderCompound = new btCompoundShape;
	btCollisionShape* cylinderShape = new btCylinderShapeX(btVector3(4,1,1));
	btCollisionShape* boxShape = new btBoxShape(btVector3(4,1,1));
	btTransform localTransform;
	localTransform.setIdentity();
	cylinderCompound->addChildShape(localTransform,boxShape);
	btQuaternion orn(SIMD_HALF_PI,0,0);
	localTransform.setRotation(orn);
	//	localTransform.setOrigin(btVector3(1,1,1));
	cylinderCompound->addChildShape(localTransform,cylinderShape);


	btCollisionShape*	shape[]={cylinderCompound,
		new btBoxShape(btVector3(1,1,1)),
		new btSphereShape(1.5)
		
	};
	static const int	nshapes=sizeof(shape)/sizeof(shape[0]);
	for(int i=0;i<currcount;++i)
	{
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(0,2+6*i,0));
		pdemo->localCreateRigidBody(mass,startTransform,shape[i%nshapes]);
		//pdemo->localCreateRigidBody(mass,startTransform,shape[0]);
	}
}

//
// Big ball
//
static void	Ctor_BigBall(SoftDemo* pdemo,btScalar mass=10)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,13,0));
	pdemo->localCreateRigidBody(mass,startTransform,new btSphereShape(3));
}

//
// Big plate
//
static btRigidBody*	Ctor_BigPlate(SoftDemo* pdemo,btScalar mass=15,btScalar height=4)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,height,0.5));
	btRigidBody*		body=pdemo->localCreateRigidBody(mass,startTransform,new btBoxShape(btVector3(5,1,5)));
	body->setFriction(1);
	return(body);
}

//
// Linear stair
//
static void Ctor_LinearStair(SoftDemo* pdemo,const btVector3& org,const btVector3& sizes,btScalar angle,int currcount)
{
	btBoxShape*	shape=new btBoxShape(sizes);
	for(int i=0;i<currcount;++i)
	{
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(org+btVector3(sizes.x()*i*2,sizes.y()*i*2,0));
		btRigidBody* body=pdemo->localCreateRigidBody(0,startTransform,shape);
		body->setFriction(1);
	}
}

//
// Softbox
//
static btSoftBody* Ctor_SoftBox(SoftDemo* pdemo,const btVector3& p,const btVector3& s)
{
	const btVector3	h=s*0.5;
	const btVector3	c[]={	p+h*btVector3(-1,-1,-1),
		p+h*btVector3(+1,-1,-1),
		p+h*btVector3(-1,+1,-1),
		p+h*btVector3(+1,+1,-1),
		p+h*btVector3(-1,-1,+1),
		p+h*btVector3(+1,-1,+1),
		p+h*btVector3(-1,+1,+1),
		p+h*btVector3(+1,+1,+1)};
	btSoftBody*		psb=btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo,c,8);
	psb->generateBendingConstraints(2);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	return(psb);

}

//
// SoftBoulder
//
static btSoftBody* Ctor_SoftBoulder(SoftDemo* pdemo,const btVector3& p,const btVector3& s,int np,int id)
{
	btAlignedObjectArray<btVector3>	pts;
	if(id) srand(id);
	for(int i=0;i<np;++i)
	{
		pts.push_back(Vector3Rand()*s+p);
	}
	btSoftBody*		psb=btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo,&pts[0],pts.size());
	psb->generateBendingConstraints(2);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	return(psb);
}



//
// 100kg cloth locked at corners, 10 falling 10kg rb's.
//
static void	Init_Cloth(SoftDemo* pdemo)
{
	//TRACEDEMO
	goalpositions[0].open("larm.txt");
	goalpositions[1].open("rarm.txt");
	const btScalar	s=8;
	btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(	pdemo->m_softBodyWorldInfo,btVector3(-s,0,-s),
		btVector3(+s,0,-s),
		btVector3(-s,0,+s),
		btVector3(+s,0,+s),
		15,15,
		//		31,31,
		0,true);
	psb->getCollisionShape()->setMargin(0.5);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.4;
	psb->m_cfg.piterations = 2;
	pm->m_flags		-=	0;//btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2,pm);
	psb->setTotalMass(1000);
	psb->generateClusters(1024);
	psb->m_cfg.kSR_SPLT_CL = 1;
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;// +btSoftBody::fCollision::CL_SELF;
	pdemo->m_cutting=false;
	xs[0] = 10;
	xs[1] = -5;
	ys[0] = 0;
	ys[1] = 0;
	zs[0] = 0;
	zs[1] = 0;
	btBoxShape*	shape=new btBoxShape(btVector3(10,5,10));
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,-10,0));
	btRigidBody* body=pdemo->localCreateRigidBody(0,startTransform,shape);
	body->setFriction(1);
	btBoxShape*	shape2=new btBoxShape(btVector3(1,1,1));
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(5,10,0));
	box[0]=pdemo->localCreateRigidBody(1,startTransform,shape2);
	box[0]->setGravity(btVector3(0,0,0));
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(-5,10,0));
	box[1]=pdemo->localCreateRigidBody(1,startTransform,shape2);
	box[1]->setGravity(btVector3(0,0,0));
	psb->m_collisionDisabledObjects.push_back(box[0]);
	psb->m_collisionDisabledObjects.push_back(box[1]);
	/*RaveInstance::Ptr rave;
    rave.reset(new RaveInstance());
	btTransform trans(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));
	RaveRobotKinematicObject::Ptr pr2;
	
    //pr2.reset(new RaveRobotKinematicObject(rave, "robots/pr2-beta-sim.robot.xml", trans));*/

}


//
// Clusters
//

//
static void			Ctor_Gear(SoftDemo* pdemo,const btVector3& pos,btScalar speed)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(pos);
	btCompoundShape*	shape=new btCompoundShape();
#if 1
	shape->addChildShape(btTransform(btQuaternion(0,0,0)),new btBoxShape(btVector3(5,1,6)));
	shape->addChildShape(btTransform(btQuaternion(0,0,SIMD_HALF_PI)),new btBoxShape(btVector3(5,1,6)));
#else
	shape->addChildShape(btTransform(btQuaternion(0,0,0)),new btCylinderShapeZ(btVector3(5,1,7)));
	shape->addChildShape(btTransform(btQuaternion(0,0,SIMD_HALF_PI)),new btBoxShape(btVector3(4,1,8)));
#endif
	btRigidBody*		body=pdemo->localCreateRigidBody(10,startTransform,shape);
	body->setFriction(1);
	btDynamicsWorld*	world=pdemo->getDynamicsWorld();
	btHingeConstraint*	hinge=new btHingeConstraint(*body,btTransform::getIdentity());
	if(speed!=0) hinge->enableAngularMotor(true,speed,3);
	world->addConstraint(hinge);
}

//
static btSoftBody*	Ctor_ClusterBunny(SoftDemo* pdemo,const btVector3& x,const btVector3& a)
{
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,gVerticesBunny,&gIndicesBunny[0][0],BUNNY_NUM_TRIANGLES);
	btSoftBody::Material*	pm=psb->appendMaterial();
	pm->m_kLST				=	1;
	pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2,pm);
	psb->m_cfg.piterations	=	2;
	psb->m_cfg.kDF			=	1;
	psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+
		btSoftBody::fCollision::CL_RS;
	psb->randomizeConstraints();
	btMatrix3x3	m;
	m.setEulerZYX(a.x(),a.y(),a.z());
	psb->transform(btTransform(m,x));
	psb->scale(btVector3(8,8,8));
	psb->setTotalMass(150,true);
	psb->generateClusters(1);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	return(psb);
}

//
static btSoftBody*	Ctor_ClusterTorus(SoftDemo* pdemo,const btVector3& x,const btVector3& a,const btVector3& s=btVector3(2,2,2))
{
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,gVertices,&gIndices[0][0],NUM_TRIANGLES);
	btSoftBody::Material*	pm=psb->appendMaterial();
	pm->m_kLST				=	1;
	pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;			
	psb->generateBendingConstraints(2,pm);
	psb->m_cfg.piterations	=	2;
	psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+
		btSoftBody::fCollision::CL_RS;
	psb->randomizeConstraints();
	psb->scale(s);
	psb->rotate(btQuaternion(a[0],a[1],a[2]));
	psb->translate(x);
	psb->setTotalMass(50,true);
	psb->generateClusters(64);			
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	return(psb);
}

//
static struct MotorControl : btSoftBody::AJoint::IControl
{
	MotorControl()
	{
		goal=0;
		maxtorque=0;
	}
	btScalar	Speed(btSoftBody::AJoint*,btScalar current)
	{
		return(current+btMin(maxtorque,btMax(-maxtorque,goal-current)));
	}
	btScalar	goal;
	btScalar	maxtorque;
}	motorcontrol;

//
struct SteerControl : btSoftBody::AJoint::IControl
{
	SteerControl(btScalar s)
	{
		angle=0;
		sign=s;
	}
	void		Prepare(btSoftBody::AJoint* joint)
	{
		joint->m_refs[0][0]=btCos(angle*sign);
		joint->m_refs[0][2]=btSin(angle*sign);
	}
	btScalar	Speed(btSoftBody::AJoint* joint,btScalar current)
	{
		return(motorcontrol.Speed(joint,current));
	}
	btScalar	angle;
	btScalar	sign;
};

static SteerControl	steercontrol_f(+1);
static SteerControl	steercontrol_r(-1);





	/* Init		*/ 
	void (*demofncs[])(SoftDemo*)=
	{
		Init_Cloth,

	};

void	SoftDemo::clientResetScene()
{
	m_azi = 0;
	m_cameraDistance = 30.f;
	m_cameraTargetPosition.setValue(0,0,0);

	DemoApplication::clientResetScene();
	/* Clean up	*/ 
	for(int i=m_dynamicsWorld->getNumCollisionObjects()-1;i>=0;i--)
	{
		btCollisionObject*	obj=m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}
		while(m_dynamicsWorld->getNumConstraints())
		{
			btTypedConstraint*	pc=m_dynamicsWorld->getConstraint(0);
			m_dynamicsWorld->removeConstraint(pc);
			delete pc;
		}
		btSoftBody* softBody = btSoftBody::upcast(obj);
		if (softBody)
		{
			getSoftDynamicsWorld()->removeSoftBody(softBody);
		} else
		{
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body)
				m_dynamicsWorld->removeRigidBody(body);
			else
				m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}


	//create ground object
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-12,0));

	btCollisionObject* newOb = new btCollisionObject();
	newOb->setWorldTransform(tr);
	newOb->setInterpolationWorldTransform( tr);
	int lastDemo = (sizeof(demofncs)/sizeof(demofncs[0]))-1;

	if (current_demo<0)
		current_demo = lastDemo;
	if (current_demo > lastDemo)
		current_demo =0;
		

	if (current_demo>19)
	{
		newOb->setCollisionShape(m_collisionShapes[0]);
	} else
	{
		newOb->setCollisionShape(m_collisionShapes[1]);
	}

	m_dynamicsWorld->addCollisionObject(newOb);

	m_softBodyWorldInfo.m_sparsesdf.Reset();





	

	motorcontrol.goal = 0;
	motorcontrol.maxtorque = 0;



	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
	m_softBodyWorldInfo.water_density	=	0;
	m_softBodyWorldInfo.water_offset		=	0;
	m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);


	m_autocam						=	false;
	m_raycast						=	false;
	m_cutting						=	false;
	m_results.fraction				=	1.f;
	demofncs[current_demo](this);
}


void SoftDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT); 




	float ms = getDeltaTimeMicroseconds();
	float dt = ms / 1000000.f;//1.0/60.;	



	if (m_dynamicsWorld)
	{
		
		if (sDemoMode)
		{
			static float demoCounter = DEMO_MODE_TIMEOUT;
			demoCounter-= dt;
			if (demoCounter<0)
			{
				
				demoCounter=DEMO_MODE_TIMEOUT;
				current_demo++;
				current_demo=current_demo%(sizeof(demofncs)/sizeof(demofncs[0]));
				clientResetScene();
			}
		}
		

//#define FIXED_STEP
#ifdef FIXED_STEP
		m_dynamicsWorld->stepSimulation(dt=1.0f/60.f,0);

#else
		//during idle mode, just run 1 simulation step maximum, otherwise 4 at max
		int maxSimSubSteps = m_idle ? 1 : 4;
		//if (m_idle)
		//	dt = 1.0/420.f;

		int numSimSteps;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt);
		//numSimSteps = m_dynamicsWorld->stepSimulation(dt,10,1./240.f);

#ifdef VERBOSE_TIMESTEPPING_CONSOLEOUTPUT
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_TIMESTEPPING_CONSOLEOUTPUT

#endif		

		if(m_drag)
		{
			m_node->m_v*=0;
			for (int i = 0; i < numnodes; i++)
				m_nodes[i]->m_v*=0;
		}

		m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();

		//optional but useful: debug drawing

	}

#ifdef USE_QUICKPROF 
	btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 

	renderme(); 

	//render the graphics objects, with center of mass shift

	updateCamera();



#ifdef USE_QUICKPROF 
	btProfiler::endBlock("render"); 
#endif 
	glFlush();
	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	printf("num manifolds: %i\n",gNumManifold);
	printf("num gOverlappingPairs: %i\n",gOverlappingPairs);
	
#endif //PRINT_CONTACT_STATISTICS


	swapBuffers();

}



void	SoftDemo::renderme()
{
	btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	m_dynamicsWorld->debugDrawWorld();

	int debugMode = m_dynamicsWorld->getDebugDrawer()? m_dynamicsWorld->getDebugDrawer()->getDebugMode() : -1;

	btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	btIDebugDraw*	sdraw = softWorld ->getDebugDrawer();


	for (  int i=0;i<softWorld->getSoftBodyArray().size();i++)
	{
		btSoftBody*	psb=(btSoftBody*)softWorld->getSoftBodyArray()[i];
		if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
		{
			btSoftBodyHelpers::DrawFrame(psb,softWorld->getDebugDrawer());
			btSoftBodyHelpers::Draw(psb,softWorld->getDebugDrawer(),softWorld->getDrawFlags());
		}
	}

	/* Bodies		*/ 
	btVector3	ps(0,0,0);
	int			nps=0;

	btSoftBodyArray&	sbs=getSoftDynamicsWorld()->getSoftBodyArray();
	for(int ib=0;ib<sbs.size();++ib)
	{
		btSoftBody*	psb=sbs[ib];
		nps+=psb->m_nodes.size();
		for(int i=0;i<psb->m_nodes.size();++i)
		{
			ps+=psb->m_nodes[i].m_x;
		}		
	}
	ps/=nps;
	if(m_autocam)
		m_cameraTargetPosition+=(ps-m_cameraTargetPosition)*0.05;
	/* Anm			*/ 
	if(!isIdle())
		m_animtime=m_clock.getTimeMilliseconds()/1000.f;
	/* Ray cast		*/ 
	if(m_raycast)
	{		
		/* Prepare rays	*/ 
		const int		res=64;
		const btScalar	fres=res-1;
		const btScalar	size=8;
		const btScalar	dist=10;
		btTransform		trs;
		trs.setOrigin(ps);
		btScalar rayLength = 1000.f;

		const btScalar	angle=m_animtime*0.2;
		trs.setRotation(btQuaternion(angle,SIMD_PI/4,0));
		btVector3	dir=trs.getBasis()*btVector3(0,-1,0);
		trs.setOrigin(ps-dir*dist);
		btAlignedObjectArray<btVector3>	origins;
		btAlignedObjectArray<btScalar>	fractions;
		origins.resize(res*res);
		fractions.resize(res*res,1.f);
		for(int y=0;y<res;++y)
		{
			for(int x=0;x<res;++x)
			{
				const int	idx=y*res+x;
				origins[idx]=trs*btVector3(-size+size*2*x/fres,dist,-size+size*2*y/fres);
			}
		}
		/* Cast rays	*/ 		
		{
			m_clock.reset();
			btVector3*		org=&origins[0];
			btScalar*				fraction=&fractions[0];
			btSoftBody**			psbs=&sbs[0];
			btSoftBody::sRayCast	results;
			for(int i=0,ni=origins.size(),nb=sbs.size();i<ni;++i)
			{
				for(int ib=0;ib<nb;++ib)
				{
					btVector3 rayFrom = *org;
					btVector3 rayTo = rayFrom+dir*rayLength;
					if(psbs[ib]->rayTest(rayFrom,rayTo,results))
					{
						*fraction=results.fraction;
					}
				}
				++org;++fraction;
			}
			long	ms=btMax<long>(m_clock.getTimeMilliseconds(),1);
			long	rayperseconds=(1000*(origins.size()*sbs.size()))/ms;
			printf("%d ms (%d rays/s)\r\n",int(ms),int(rayperseconds));
		}
		/* Draw rays	*/ 
		const btVector3	c[]={	origins[0],
			origins[res-1],
			origins[res*(res-1)],
			origins[res*(res-1)+res-1]};
		idraw->drawLine(c[0],c[1],btVector3(0,0,0));
		idraw->drawLine(c[1],c[3],btVector3(0,0,0));
		idraw->drawLine(c[3],c[2],btVector3(0,0,0));
		idraw->drawLine(c[2],c[0],btVector3(0,0,0));
		for(int i=0,ni=origins.size();i<ni;++i)
		{
			const btScalar		fraction=fractions[i];
			const btVector3&	org=origins[i];
			if(fraction<1.f)
			{
				idraw->drawLine(org,org+dir*rayLength*fraction,btVector3(1,0,0));
			}
			else
			{
				idraw->drawLine(org,org-dir*rayLength*0.1,btVector3(0,0,0));
			}
		}
#undef RES
	}
	/* Water level	*/ 
	static const btVector3	axis[]={btVector3(1,0,0),
		btVector3(0,1,0),
		btVector3(0,0,1)};
	if(m_softBodyWorldInfo.water_density>0)
	{
		const btVector3	c=	btVector3((btScalar)0.25,(btScalar)0.25,1);
		const btScalar	a=	(btScalar)0.5;
		const btVector3	n=	m_softBodyWorldInfo.water_normal;
		const btVector3	o=	-n*m_softBodyWorldInfo.water_offset;
		const btVector3	x=	btCross(n,axis[n.minAxis()]).normalized();
		const btVector3	y=	btCross(x,n).normalized();
		const btScalar	s=	25;
		idraw->drawTriangle(o-x*s-y*s,o+x*s-y*s,o+x*s+y*s,c,a);
		idraw->drawTriangle(o-x*s-y*s,o+x*s+y*s,o-x*s+y*s,c,a);
	}
	//

	int lineWidth=280;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	if((getDebugMode() & btIDebugDraw::DBG_NoHelpText)==0)
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		
		glRasterPos3f(xStart, yStart, 0);
		if (sDemoMode)
		{		
			sprintf(buf,"d to toggle demo mode (on)");
		} else
		{
			sprintf(buf,"d to toggle demo mode (off)");
		}
		GLDebugDrawString(xStart,20,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"] for next demo (%d)",current_demo);
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"c to visualize clusters");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"; to toggle camera mode");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
        sprintf(buf,"n,m,l,k for power and steering");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);


		resetPerspectiveProjection();
	}
	glLoadIdentity();
	glEnable(GL_DEPTH_TEST);
	{
		GLfloat LightDiffuse[] = { 1, 1, 1.0, 1};
		GLfloat LightPosition[] = { 0, 0, -100};
		glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
		glLightfv(GL_LIGHT0, GL_POSITION,LightPosition);
		glEnable(GL_LIGHT0);
	}
	{
		GLfloat LightDiffuse[] = { 1.0, 1, 1, 1};
		GLfloat LightPosition[] = { 0, 0, 100};
		glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
		glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);
		glEnable(GL_LIGHT1);
	}
	glEnable(GL_LIGHTING);
	
	glShadeModel(GL_SMOOTH);

	DemoApplication::renderme();

}

void	SoftDemo::setDrawClusters(bool drawClusters)
{
	if (drawClusters)
	{
		getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags()|fDrawFlags::Clusters);
	} else
	{
		getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags()& (~fDrawFlags::Clusters));
	}
}



void	SoftDemo::keyboardCallback(unsigned char key, int x, int y)
{
	btSoftBodyArray&		sbs=getSoftDynamicsWorld()->getSoftBodyArray();
	btSoftBody*				psb=sbs[0];
	btScalar max = -1;
	int index = -1;
	
	btVector3 corner1; if (m_nodes[0]) corner1 =m_nodes[0]->m_x;
	btVector3 corner2; if (m_nodes[1]) corner2 =m_nodes[1]->m_x;
	btVector3 coord;
	btScalar maxing;
	int x2 = (xs[1] - xs[0])/2+xs[0];
	int y2 = ys[0]+10;
	const btVector3			rayFrom=m_cameraPosition;
	const btVector3			rayTo=getRayTo(x2,y2);
	const btVector3			rayDir=(rayTo-rayFrom).normalized();
	btSoftBody::sRayCast	res;
	switch(key)
	{
	case	'w':	ys[0] -= 0.1;ys[1]-=0.1;break;
	case	's':	ys[0] += 0.1;ys[1]+=0.1;break;
	case	'a':	xs[0] -= 0.1;break;
	case	'd':	xs[0] += 0.1;break;
	case	'c':	getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags()^fDrawFlags::Clusters);break;
	case	'1':
		for (int i = 0; i < 1800; i++) {
			btSoftBody::Face&	f=m_results.body->m_faces[i];
			m_node=f.m_n[0];
			coord = m_node->m_x;
			maxing = corner1.distance2(coord);
			if (maxing > max) {
				index = i;
				max = maxing;
			}
		}
		m_node = m_results.body->m_faces[index].m_n[0];
		m_nodes[3] = m_node;
		max = 0;
		for (int i = 0; i < 1800; i++) {
			btSoftBody::Face&	f=m_results.body->m_faces[i];
			m_node=f.m_n[0];
			coord = m_node->m_x;
			maxing = corner2.distance2(coord);
			if (maxing > max) {
				index = i;
				max = maxing;
			}
		}
		m_node = m_results.body->m_faces[index].m_n[0];
		m_nodes[2] = m_node;
		break;
	case '2':
		if(psb->rayTest(rayFrom,rayTo,res))
		{
			xs[1] = xs[0];
			ys[1] = ys[0];
			xs[2] = x2;
			ys[2] = y2;
			//m_results=res;
			//m_node = m_results.body->m_faces[m_results.index].m_n[0];
			//m_nodes[4] = m_node;
		}
		break;
	case	'`':
		{
			btSoftBodyArray&	sbs=getSoftDynamicsWorld()->getSoftBodyArray();
			for(int ib=0;ib<sbs.size();++ib)
			{
				btSoftBody*	psb=sbs[ib];
				psb->staticSolve(128);
			}
		}
		break;
	default:		DemoApplication::keyboardCallback(key,x,y);
	}
}

//
void	SoftDemo::mouseMotionFunc(int x,int y)
{
	if(m_node&&(m_results.fraction<1.f))
	{
		if(!m_drag)
		{
#define SQ(_x_) (_x_)*(_x_)
			if((SQ(x-m_lastmousepos[0])+SQ(y-m_lastmousepos[1]))>6)
			{
				m_drag=true;
			}
#undef SQ
		}
		if(m_drag)
		{
			m_lastmousepos[0]	=	500;
			m_lastmousepos[1]	=	200;		
		}
	}
	else
	{
		DemoApplication::mouseMotionFunc(x,y);
	}
}

//
void	SoftDemo::mouseFunc(int button, int state, int x, int y)
{
	if(button==0)
	{
		switch(state)
		{
			case	0:
			{
				m_results.fraction=1.f;
				DemoApplication::mouseFunc(button,state,x,y);
				if(!m_pickConstraint)
				{
					const btVector3			rayFrom=m_cameraPosition;
					const btVector3			rayTo=getRayTo(x,y);
					const btVector3			rayDir=(rayTo-rayFrom).normalized();
					btSoftBodyArray&		sbs=getSoftDynamicsWorld()->getSoftBodyArray();
					for(int ib=0;ib<sbs.size();++ib)
					{
						btSoftBody*				psb=sbs[ib];
						btSoftBody::sRayCast	res;
						m_results.feature = btSoftBody::eFeature::Face;
						m_results.index = rand()%1800;
						m_results.fraction = 0.003;
						m_results.body = psb;
						/*if(psb->rayTest(rayFrom,rayTo,res))
						{
							m_results=res;
						}*/
					}
					if(m_results.fraction<1.f)
					{				
						m_impact			=	rayFrom+(rayTo-rayFrom)*m_results.fraction;
						m_drag				=	m_cutting ? false : true;
						m_lastmousepos[0]	=	500;
						m_lastmousepos[1]	=	200;
						m_node				=	0;
						switch(m_results.feature)
						{
						case btSoftBody::eFeature::Tetra:
							{
								btSoftBody::Tetra&	tet=m_results.body->m_tetras[m_results.index];
								m_node=tet.m_n[0];
								for(int i=1;i<4;++i)
								{
									if(	(m_node->m_x-m_impact).length2()>
										(tet.m_n[i]->m_x-m_impact).length2())
									{
										m_node=tet.m_n[i];
									}
								}
								break;
							}
						case	btSoftBody::eFeature::Face:
							{
								btSoftBody::Face&	f=m_results.body->m_faces[m_results.index];
								m_node=f.m_n[0];
								for(int i=1;i<3;++i)
								{
									if(	(m_node->m_x-m_impact).length2()>
										(f.m_n[i]->m_x-m_impact).length2())
									{
										m_node=f.m_n[i];
									}
								}
							}
							break;
						}
						if(m_node) m_goal=m_node->m_x;

						return;
					}
				}
			}
			break;
		case	1:
			if((!m_drag)&&m_cutting&&(m_results.fraction<1.f))
			{
				ImplicitSphere	isphere(m_impact,1);
				printf("Mass before: %f\r\n",m_results.body->getTotalMass());
				m_results.body->refine(&isphere,0.0001,true);
				printf("Mass after: %f\r\n",m_results.body->getTotalMass());
			}
			m_results.fraction=1.f;
			m_drag=false;
			DemoApplication::mouseFunc(button,state,x,y);
			break;
		}
	}
	else if (button == 2 && state == 0)
	{
		m_results.fraction=1.f;
		//DemoApplication::mouseFunc(button,state,x,y);
		if(!m_pickConstraint)
		{
			const btVector3			rayFrom=m_cameraPosition;
			const btVector3			rayTo=getRayTo(x,y);
			const btVector3			rayDir=(rayTo-rayFrom).normalized();
			btSoftBodyArray&		sbs=getSoftDynamicsWorld()->getSoftBodyArray();
			for(int ib=0;ib<sbs.size();++ib)
			{
				btSoftBody*				psb=sbs[ib];
				btScalar min = -1;
				int index = -1;
				for (int i = 0; i < 1800; i++) {
					btSoftBody::Face&	f=m_results.body->m_faces[i];
					m_node=f.m_n[0];
					btVector3 coord = m_node->m_x;
					btScalar ycoord = coord.getY();
					if (min == -1 || ycoord < min) {
						index = i;
						min = ycoord;
					}
				}
				btSoftBody::sRayCast	res;
				m_results.feature = btSoftBody::eFeature::Face;
				m_results.index = index;
				m_results.fraction = 0.003;
				m_results.body = psb;
				/*if(psb->rayTest(rayFrom,rayTo,res))
				{
					m_results=res;
				}*/
			}
			if(m_results.fraction<1.f)
			{				
				m_impact			=	rayFrom+(rayTo-rayFrom)*m_results.fraction;
				m_drag				=	m_cutting ? false : true;
				m_lastmousepos[0]	=	500;
				m_lastmousepos[1]	=	200;
				m_node				=	0;
				switch(m_results.feature)
				{
				case btSoftBody::eFeature::Tetra:
					{
						btSoftBody::Tetra&	tet=m_results.body->m_tetras[m_results.index];
						m_node=tet.m_n[0];
						for(int i=1;i<4;++i)
						{
							if(	(m_node->m_x-m_impact).length2()>
								(tet.m_n[i]->m_x-m_impact).length2())
							{
								m_node=tet.m_n[i];
							}
						}
						break;
					}
				case	btSoftBody::eFeature::Face:
					{
						btSoftBody::Face&	f=m_results.body->m_faces[m_results.index];
						m_node=f.m_n[0];
						for(int i=1;i<3;++i)
						{
							if(	(m_node->m_x-m_impact).length2()>
								(f.m_n[i]->m_x-m_impact).length2())
							{
								m_node=f.m_n[i];
							}
						}
					}
					break;
				}
				if(m_node) m_goal=m_node->m_x;
				btVector3 vel(0,0,0);
				sbs[0]->setVelocity(vel);
				m_nodes[numnodes] = m_node;
				numnodes ^= 1;
				return;
			}
		}
	}
	else
		DemoApplication::mouseFunc(button,state,x,y);
}


void	SoftDemo::initPhysics()
{
	///create concave ground mesh

	
	m_azi = 0;

	//reset and disable motorcontrol at the start
	motorcontrol.goal = 0;
	motorcontrol.maxtorque = 0;

	btCollisionShape* groundShape = 0;
	{
		int i;
		int j;

		const int NUM_VERTS_X = 30;
		const int NUM_VERTS_Y = 30;
		const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
		const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

		gGroundVertices = new btVector3[totalVerts];
		gGroundIndices = new int[totalTriangles*3];

		btScalar offset(-50);

		for ( i=0;i<NUM_VERTS_X;i++)
		{
			for (j=0;j<NUM_VERTS_Y;j++)
			{
				gGroundVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
					//0.f,
					waveheight*sinf((float)i)*cosf((float)j+offset),
					(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
			}
		}

		int vertStride = sizeof(btVector3);
		int indexStride = 3*sizeof(int);

		int index=0;
		for ( i=0;i<NUM_VERTS_X-1;i++)
		{
			for (int j=0;j<NUM_VERTS_Y-1;j++)
			{
				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = j*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i;
			}
		}

		btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
			gGroundIndices,
			indexStride,
			totalVerts,(btScalar*) &gGroundVertices[0].x(),vertStride);

		bool useQuantizedAabbCompression = true;

		groundShape = new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression);
		groundShape->setMargin(0.5);
	}

	m_collisionShapes.push_back(groundShape);

	btCollisionShape* groundBox = new btBoxShape (btVector3(100,CUBE_HALF_EXTENTS,100));
	m_collisionShapes.push_back(groundBox);

	btCompoundShape* cylinderCompound = new btCompoundShape;
	btCollisionShape* cylinderShape = new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
	btTransform localTransform;
	localTransform.setIdentity();
	cylinderCompound->addChildShape(localTransform,cylinderShape);
	btQuaternion orn(btVector3(0,1,0),SIMD_PI);
	localTransform.setRotation(orn);
	cylinderCompound->addChildShape(localTransform,cylinderShape);

	m_collisionShapes.push_back(cylinderCompound);


	m_dispatcher=0;

	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();


	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	////////////////////////////
	///Register softbody versus softbody collision algorithm


	///Register softbody versus rigidbody collision algorithm


	////////////////////////////

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = world;
	m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback,this,true);


	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);





	
	//	clientResetScene();

	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	clientResetScene();
}






void	SoftDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;



	delete m_collisionConfiguration;


}



