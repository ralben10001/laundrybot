#include <osgViewer/Viewer>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgGA/TrackballManipulator>
#include "environment.h"
#include "basicobjects.h"
#include "openravesupport.h"

class Scene;

class EventHandler : public osgGA::TrackballManipulator {
private:
  Scene *scene;
  float lastX, lastY, dx, dy;
protected:
  void getTransformation( osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up ) const;
public:
  EventHandler(Scene *scene_) : scene(scene_), state() {}
  struct {
    bool debugDraw, moveGrabber0, moveGrabber1, startDragging, idling;
  } state;
  bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);

};

struct Scene {
  typedef boost::shared_ptr<Scene> Ptr;

  OSGInstance::Ptr osg;
  BulletInstance::Ptr bullet;
  RaveInstance::Ptr rave;
  Environment::Ptr env;
  boost::shared_ptr<osgbCollision::GLDebugDrawer> dbgDraw;
  osgViewer::Viewer viewer;
  osg::ref_ptr<EventHandler> manip;

  PlaneStaticObject::Ptr ground;
  RaveRobotKinematicObject::Ptr pr2;
  RaveRobotKinematicObject::Manipulator::Ptr pr2Left, pr2Right;

  struct {
    bool enableIK, enableHaptics;
  } options;

  Scene(bool enableIK, bool enableHaptics, btScalar pr2Scale=20.);

  void processHaptics();
  void step(float, int, float);
  void draw();
  void viewerLoop(int maxsteps=200, float internaldt=1./200.);
};



