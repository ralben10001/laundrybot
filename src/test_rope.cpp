#include "rope.h"
#include "simplescene.h"
#include "unistd.h"
#include "file_reading.h"
#include "grabbing.h"
#include "softbodies.h"

#include <BulletSoftBody/btSoftBodyHelpers.h>
static btSoftBody::Node* m_nodes[2];
btScalar		xs[3];
btScalar		ys[3];
btScalar		zs[3];
int numnodes = 0;
int clothheight;
int clothlength;

btVector3 findCenterOfMass(btSoftBody* psb) {
	btVector3 mid(0,0,0);
	for (int i = 0; i < psb->m_nodes.size(); i++)
		mid += psb->m_nodes[i].m_x;
	return mid/psb->m_nodes.size();
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
			btScalar zcoord = coord.getZ();
			if (min == -1 || zcoord < min) {
				min = zcoord;
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
			pos[0] = coord.getY()*direction;
			pos[1] = coord.getZ();
			pos[2] = coord.getX();
			if (min == -1 || pos[abs(direction)-1] > min) {
				min = pos[abs(direction)-1];
				min_node=m_node;
			}
		}
		ys[numnodes] = min*direction;
		m_node				=	min_node;
		m_nodes[numnodes] = m_node;
		btVector3 vel(0,0,0);
		psb->setVelocity(vel);
		numnodes ^= 1;
		return;
}

void dragovertable(int direction) {
	if (direction == 0) {
		ys[0] -= 0.02;
		ys[1] -= 0.02;
	}
	else if (direction == 1) {
		ys[0] += 0.02;
		ys[1] += 0.02;
	}
	else if (direction == 2)
	{
		xs[0] -=0.02;
		xs[1] -=0.02;
	}
	else if (direction == 3)
	{
		xs[0] +=0.02;
		xs[1] +=0.02;
	}
}

void lift() {
	zs[0] += 10;
}

void drop() {
	zs[0] -= 10;
}

void bringforward() {
	xs[0] -= 20;
	xs[1] -= 20;
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

btSoftBody* gen(Scene &scene, btScalar s, btScalar z) {
  btSoftBody* psb = btSoftBodyHelpers::CreatePatch(scene.env->bullet->softBodyWorldInfo,
						   btVector3(-s,-s,z+1),
						   btVector3(+s,-s,z+1),
						   btVector3(-s,+s,z+1),
						   btVector3(+s,+s,z+1),
						   31, 31,
						   0, true);

  //psb->getCollisionShape()->setMargin(0.4);
  psb->m_cfg.piterations = 2;
  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST = 0.4;
  // pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
  psb->generateBendingConstraints(2, pm);
  psb->setTotalMass(1000);
  //psb->m_collisionDisabledObjects.push_back(scene.pr2);
  psb->generateClusters(1024);
  psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;// + btSoftBody::fCollision::CL_SELF;
  
  
    scene.env->add(BulletSoftObject::Ptr(new BulletSoftObject(psb)));
  return psb;
}

void getClosestNode(btVector3& pos, btSoftBody* psb, int grabber) {
  int index = -1;
  btScalar dist = 0.;
  for (int i = 0; i < psb->m_nodes.size(); i++) {
	btScalar curr = psb->m_nodes[i].m_x.distance(pos);
	if (index == -1 || curr < dist) {
		index = i;
		dist = curr;
	}
  }
  m_nodes[grabber] = &psb->m_nodes[index];
  cout << index << endl;
  
}


using boost::shared_ptr;

int main() {

  const float table_height = 4;
  const float rope_radius = .01;
  const float segment_len = .025;
  const float table_thickness = .5;
  int nLinks = 50;

  btAlignedObjectArray<btVector3> ctrlPts;
  for (int i=0; i< nLinks; i++) {
    ctrlPts.push_back(btVector3(.5+segment_len*i,0,table_height+5*rope_radius));
  }
  


  shared_ptr <btMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,0,0))));
  shared_ptr<BulletObject> table(new BoxObject(0,btVector3(10,10,4),ms));
  shared_ptr <btMotionState> ms2(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,0,0))));
  shared_ptr<BulletObject> table2(new BoxObject(0,btVector3(50,50,table_thickness/2),ms2));


  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts,.01));

  Scene s = Scene(true, true);
  btSoftBody* psb = gen(s, 8, table_height+table_thickness);

  
  //s.env->add(ropePtr);
  s.env->add(table);
  s.env->add(table2);

  vector< vector<double> > joints;
  vector< int > inds;
  read_1d_array(inds, "../data/inds.txt");
  read_2d_array(joints,"../data/vals.txt");
  vector< vector<double> > transformsleft;
  vector< vector<double> > transformsright;
  read_2d_array(transformsleft,"../data/larm.txt");
  read_2d_array(transformsright,"../data/rarm.txt");
  int lind = 0;
  int rind = 0;

  int step = 0;
  ys[0] = 10;
  ys[1] = -5;
  zs[0] = 9;
  zs[1] = 9;
  xs[0] = 0;
  xs[1] = 0;
  

  //  btVector3 v = ropePtr->bodies[0]->getCenterOfMassPosition();
  RobotBase::ManipulatorPtr rarm(s.pr2->robot->GetManipulators()[5]);
  RobotBase::ManipulatorPtr larm(s.pr2->robot->GetManipulators()[7]);
  
  s.env->bullet->dynamicsWorld->setGravity(btVector3(0,0,-10.));
  Grab g;
  Grab g2;
  float currtime = 0;
  int iterations = 0;
  float dt = 1.0/60.0;
  btVector3 goalpost(10,0,17);
  btTransform origTranst;
  s.pr2Left->grabber->motionState->getWorldTransform(origTranst);
  btTransform transt(origTranst);
  transt.setOrigin(goalpost);
  s.pr2Right->moveByIK(transt);
  for (int i=0; i < 500000 && !s.viewer.done(); i++) {
    btScalar xg, yg, zg;
	btScalar maxdrag = 10;
	for (int i = 0; i < 2; i++) {
		if (m_nodes[i] == 0)
			continue;
		xg=xs[i%2];
		yg=ys[i%2];
		zg=zs[i%2];
		btVector3 mid = findCenterOfMass(psb);
		btVector3 goalpos(m_nodes[i]->m_x + btVector3(10,0,9));
		btVector3 relPos = goalpos - mid;
		float theta = -atan(relPos.x()/relPos.y());
		btMatrix3x3 mat(1,0,0,
		      0,cos(theta+3.1415),-sin(theta+3.1415),
		      0,sin(theta+3.1415),cos(theta+3.1415));
	        btTransform origTrans;
		if (i == 0) {
	        s.pr2Right->grabber->motionState->getWorldTransform(origTrans);
	        btTransform trans(origTrans);
		trans.setBasis(mat);
		trans.setOrigin(goalpos);
		s.pr2Right->moveByIK(trans);
		}
		else {
	        s.pr2Left->grabber->motionState->getWorldTransform(origTrans);
	        btTransform trans(origTrans);
		trans.setBasis(mat);
		trans.setOrigin(goalpos);
		s.pr2Left->moveByIK(trans);
		}
		btVector3 m_goal(xg,yg,zg);
		cout << xg << ',' << yg << ',' << zg << endl;
		btVector3 delta = m_goal-m_nodes[i]->m_x;
		if(delta.length2()>(1))
		{
			delta=delta.normalized()*maxdrag;
		}
		m_nodes[i]->m_v+=delta/dt;
	}
	currtime += dt;
	if (iterations == 2 || iterations == 3) {
		dragovertable(0);
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
    	s.step(dt,1,dt);
  }

}
