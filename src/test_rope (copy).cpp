#include "rope.h"
#include "simplescene.h"
#include "unistd.h"
#include "file_reading.h"
#include "grabbing.h"
#include "softbodies.h"

#include <BulletSoftBody/btSoftBodyHelpers.h>
static btSoftBody::Node* m_nodes[2];


btSoftBody* gen(Scene &scene, btScalar s, btScalar z) {
  btSoftBody* psb = btSoftBodyHelpers::CreatePatch(scene.env->bullet->softBodyWorldInfo,
						   btVector3(-s+s+5,-s,z+1),
						   btVector3(+s+s+5,-s,z+1),
						   btVector3(-s+s+5,+s,z+1),
						   btVector3(+s+s+5,+s,z+1),
						   63, 63,
						   0, true);

  psb->getCollisionShape()->setMargin(0.4);
  psb->m_cfg.piterations = 2;
  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST = 0.4;
  // pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
  psb->generateBendingConstraints(2, pm);
  psb->setTotalMass(1);
  //psb->m_collisionDisabledObjects.push_back(scene.pr2);
  //psb->generateClusters(1024);
  //psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;// + btSoftBody::fCollision::CL_SELF;
  
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


  shared_ptr <btMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(10,0,table_height-table_thickness/2))));
  shared_ptr<BulletObject> table(new BoxObject(0,btVector3(7.5,7.5,table_thickness/2),ms));

  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts,.01));

  Scene s = Scene(true, true);
  btSoftBody* psb = gen(s, 4, table_height+table_thickness);

  
  //s.env->add(ropePtr);
  s.env->add(table);

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
  

  //  btVector3 v = ropePtr->bodies[0]->getCenterOfMassPosition();
  RobotBase::ManipulatorPtr rarm(s.pr2->robot->GetManipulators()[5]);
  RobotBase::ManipulatorPtr larm(s.pr2->robot->GetManipulators()[7]);
  
  s.env->bullet->dynamicsWorld->setGravity(btVector3(0,0,-100.));
  Grab g;
  Grab g2;
  for (int i=0; i < 5000 && !s.viewer.done(); i++) {
    cout << i << endl;
    //vector<double> joint = joints[i];
    //s.pr2->setDOFValues(inds,joint);
    OpenRAVE::Transform targ = larm->GetEndEffectorTransform();
    vector<dReal> vsolution;
    btTransform transb = util::toBtTransform(targ);
    cout << transb.getOrigin().x() << ','
	 << transb.getOrigin().y() << ','
	 << transb.getOrigin().z() << endl;
    //trans.setOrigin(btVector3(0.76,0.55,1.15));
    //targ = util::toRaveTransform(trans); 
    vector <double> left = transformsleft[lind];
    vector <double> right = transformsright[rind]; 
    if (i  == left[0] - 550) {
      lind++;

      btMatrix3x3 mat(1,0,0,
		      0,cos(left[1]+3.1415),-sin(left[1]+3.1415),
		      0,sin(left[1]+3.1415),cos(left[1]+3.1415));
      btVector3 orig(left[2],left[3],left[4]);
      orig.setZ(orig.getY()/20+0.75);
      orig.setY(orig.getX()/30-0.188);
      orig.setX(orig.getZ());
      orig = orig*10;
      cout << orig.getX() << ',' << orig.getY() <<
	',' << orig.getZ() << endl;
      //trans.setIdentity();
      btTransform origTrans;
      s.pr2Right->grabber->motionState->getWorldTransform(origTrans);
      btTransform trans(origTrans);
      //btVector3 delta = orig-origTrans.getOrigin();
      //delta = delta.normalize()*0.2;
      trans.setOrigin(orig);
      //trans.setBasis(transb.getBasis());
      trans.setBasis(mat);
      //OpenRAVE::Transform targ = util::toRaveTransform(trans);
      //larm->FindIKSolution(IkParameterization(targ),vsolution,true);
      //s.pr2->setDOFValues(larm->GetArmIndices(),vsolution);
      s.pr2Right->moveByIK(trans);
    }
    if (i == right[0] - 550) {
      rind++;

      btMatrix3x3 mat(1,0,0,
		      0,cos(right[1]+3.1415),-sin(right[1]+3.1415),
		      0,sin(right[1]+3.1415),cos(right[1]+3.1415));
      //mat = mat*mat2;	
      btVector3 orig(right[2],right[3],right[4]);
      orig.setZ(orig.getY()/20+0.7);
      orig.setY(orig.getX()/30+0.188);
      orig.setX(orig.getZ());
      orig = orig*10;
      cout << orig.getX() << ',' << orig.getY() <<
	',' << orig.getZ() << endl;
      btTransform origTrans;
      s.pr2Left->grabber->motionState->getWorldTransform(origTrans);
      btTransform trans(origTrans);
      
      trans.setOrigin(orig);
      trans.setBasis(mat);
      s.pr2Left->moveByIK(trans);
    }
    if (i == 51) {
      getClosestNode(util::toBtTransform(larm->GetEndEffectorTransform()).getOrigin(),psb,0);
    }
    if (m_nodes[0]) {
      btTransform armpos;
      s.pr2Right->grabber->motionState->getWorldTransform(armpos);
      btVector3 des = armpos.getOrigin();
      cout << des.x() << ',' << des.y() << ',' << des.z() << endl;
      cout << m_nodes[0]->m_x.x() << ',' << m_nodes[0]->m_x.y() << ',' << m_nodes[0]->m_x.z() << endl;
      btVector3 delta = des - m_nodes[0]->m_x;
      // (delta.length2() > 100)
      	delta = delta.normalized()*10.;
      m_nodes[0]->m_v += delta*30.;
    }

    //larm->FindIKSolution(IkParameterization(targ), vsolution,true);
    //s.pr2->setDOFValues(larm->GetArmIndices(),vsolution);

    /*

    if (i == 160) {
      btVector3 rhpos = util::toBtTransform(rarm->GetEndEffectorTransform()).getOrigin();
      g = Grab(ropePtr->bodies[0].get(),rhpos,s.env->bullet->dynamicsWorld);
    }
    if (i > 160) {
      g.updatePosition(util::toBtTransform(rarm->GetEndEffectorTransform()).getOrigin());
    }


    if (i == 330) {
      btVector3 lhpos = util::toBtTransform(larm->GetEndEffectorTransform()).getOrigin();
      g2 = Grab(ropePtr->bodies[nLinks-2].get(),lhpos,s.env->bullet->dynamicsWorld);*/
    //}
    /*if (i > 330) {
      g2.updatePosition(util::toBtTransform(larm->GetEndEffectorTransform()).getOrigin());
      }*/



    /*
    btRigidBody* body = ropePtr->bodies[0].get();
    cout << "position at time " << ++step << ": " << 
      body->getCenterOfMassPosition().x() << " " <<
      body->getCenterOfMassPosition().y() << " " <<
      body->getCenterOfMassPosition().z() << " " << endl;
    */

    s.step(1./30.,300,.001);
    // usleep(10*1000);
  }

}
