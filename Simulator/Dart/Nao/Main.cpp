#include <Configuration.h>
#include <Simulator/Dart/Nao/NaoWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>


void displayJointFrames(const dart::simulation::WorldPtr& world,
                        const dart::dynamics::SkeletonPtr& robot) {
  for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
    dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
    for (std::size_t j = 0; j < bn->getNumChildJoints(); ++j) {
      const dart::dynamics::Joint* joint = bn->getChildJoint(j);
      const Eigen::Isometry3d offset = joint->getTransformFromParentBodyNode();

      dart::gui::osg::InteractiveFramePtr frame =
          std::make_shared<dart::gui::osg::InteractiveFrame>(
              bn, joint->getName() + "/frame", offset);

      for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
                              dart::gui::osg::InteractiveTool::PLANAR})
        for (std::size_t i = 0; i < 3; ++i)
          frame->getTool(type, i)->setEnabled(false);

      world->addSimpleFrame(frame);
    }
  }
}

class OneStepProgress : public osgGA::GUIEventHandler {
 public:
  OneStepProgress(NaoWorldNode* worldnode) : worldnode_(worldnode) {}

  /** Deprecated, Handle events, return true if handled, false otherwise. */
  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter& /*aa*/) {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {
      // custom buttons
      // World Node Buttons
      if (ea.getKey() == 'p') {
        worldnode_->enableButtonPFlag();
      }
      if (ea.getKey() == 'r') {
        worldnode_->enableButtonRFlag();
      }
      if (ea.getKey() == 'w') {
        worldnode_->enableButtonWFlag();
      }
      if (ea.getKey() == 'a') {
        worldnode_->enableButtonAFlag();
      }
      if (ea.getKey() == 's') {
        worldnode_->enableButtonSFlag();
      }
      if (ea.getKey() == 'd') {
        worldnode_->enableButtonDFlag();
      }
      if (ea.getKey() == 'q') {
        worldnode_->enableButtonQFlag();
      }
      if (ea.getKey() == 'e') {
        worldnode_->enableButtonEFlag();
      }
      if (ea.getKey() == 'x') {
        worldnode_->enableButtonXFlag();
      }
      if (ea.getKey() == 'j') {
        worldnode_->enableButtonJFlag();
      }
      if (ea.getKey() == 'k') {
        worldnode_->enableButtonKFlag();
      }

      if (ea.getKey() == 'f') {
        // changes
        int numStepProgress(1);
        for (int i = 0; i < numStepProgress; ++i) {
          worldnode_->customPreStep();
          // std::cout<< "prestep" << std::endl;
          worldnode_->getWorld()->step();
          // std::cout<< "step" << std::endl;
          worldnode_->customPostStep();
          // std::cout<< "post step" << std::endl;
        }
        return true;
      }
    }
    return false;
  }
  NaoWorldNode* worldnode_;
};

void _setJointLimitConstraint(dart::dynamics::SkeletonPtr robot) {
  for (int i = 0; i < robot->getNumJoints(); ++i) {
    dart::dynamics::Joint* joint = robot->getJoint(i);
    joint->setPositionLimitEnforced(true);
  }
}

void _printRobotModel(dart::dynamics::SkeletonPtr robot) {
   
  
  // for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
  // 	dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
  // 	std::cout << i << "th" << std::endl;
  // 	std::cout << bn->getName() << std::endl;
  // 	std::cout << bn->getMass() << std::endl;
  // }
  
  // for (int i = 0; i < robot->getNumJoints(); ++i) {
  // 	dart::dynamics::Joint* joint = robot->getJoint(i);
  // 	std::cout << i << "th" << std::endl;
  // 	std::cout << joint->getNumDofs() << std::endl;
  // }

  
  for (int i = 0; i < robot->getNumDofs(); ++i) {
    dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
    std::cout << i << "th" << std::endl;
    std::cout << "dof name : " << dof->getName() << std::endl;
     std::cout << "child body node name and mass : "
    << dof->getChildBodyNode()->getName() << " , "
    << dof->getChildBodyNode()->getMass() << std::endl;
  }
  
  std::cout << "num dof" << std::endl;
  std::cout << robot->getNumDofs() << std::endl;
  std::cout << robot->getNumJoints() << std::endl;
  // std::cout << "mass mat row" << std::endl;
  // std::cout << robot->getMassMatrix().rows() << std::endl;
  // std::cout << robot->getMassMatrix().cols() << std::endl;
  // std::cout << "q" << std::endl;
  // std::cout << robot->getPositions() << std::endl;
  // std::cout << "robot total mass" << std::endl;
  // std::cout << robot->getMass() << std::endl;
  // std::cout << "robot position" << std::endl;
  // std::cout << robot->getPositions() << std::endl;

  // std::cout << "right" << std::endl;
  // std::cout << robot->getBodyNode("rightCOP_Frame")
  // ->getWorldTransform()
  // .translation()
  // << std::endl;
  // std::cout << "left" << std::endl;
  // std::cout << robot->getBodyNode("leftCOP_Frame")
  // ->getWorldTransform()
  // .translation()
  // << std::endl;

  exit(0);
}

// Set these angles from qibullet example file
void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot) {
  
  int LHipYawPitch = robot->getDof("LHipYawPitch")->getIndexInSkeleton();
  int LHipPitch = robot->getDof("LHipPitch")->getIndexInSkeleton();
  int LHipRoll = robot->getDof("LHipRoll")->getIndexInSkeleton();
  int LKneePitch = robot->getDof("LKneePitch")->getIndexInSkeleton();
  int LAnklePitch = robot->getDof("LAnklePitch")->getIndexInSkeleton();
  int LAnkleRoll = robot->getDof("LAnkleRoll")->getIndexInSkeleton();
  
  int RHipYawPitch = robot->getDof("RHipYawPitch")->getIndexInSkeleton();
  int RHipPitch = robot->getDof("RHipPitch")->getIndexInSkeleton();
  int RHipRoll = robot->getDof("RHipRoll")->getIndexInSkeleton();
  int RKneePitch = robot->getDof("RKneePitch")->getIndexInSkeleton();
  int RAnklePitch = robot->getDof("RAnklePitch")->getIndexInSkeleton();
  int RAnkleRoll = robot->getDof("RAnkleRoll")->getIndexInSkeleton();
	
  int LShoulderPitch = robot->getDof("LShoulderPitch")->getIndexInSkeleton();
  int LShoulderRoll = robot->getDof("LShoulderRoll")->getIndexInSkeleton();
  int LElbowYaw = robot->getDof("LElbowYaw")->getIndexInSkeleton();
  int LElbowRoll = robot->getDof("LElbowRoll")->getIndexInSkeleton();
	
  int RShoulderPitch = robot->getDof("RShoulderPitch")->getIndexInSkeleton();
  int RShoulderRoll = robot->getDof("RShoulderRoll")->getIndexInSkeleton();
  int RElbowYaw = robot->getDof("RElbowYaw")->getIndexInSkeleton();
  int RElbowRoll = robot->getDof("RElbowRoll")->getIndexInSkeleton();
  int HeadPitch = robot->getDof("HeadPitch")->getIndexInSkeleton();
  int HeadYaw = robot->getDof("HeadYaw")->getIndexInSkeleton();


  Eigen::VectorXd q = robot->getPositions();

  // Height where the robot is initialized
  q[2] = 0.38 - 0.0530;
  // Setting orientation for stand pose
  q[LHipPitch] = -0.25;
  q[RHipPitch] =  -0.25;
  q[LKneePitch] =  0.5;
  q[RKneePitch] =  0.5;
  q[LHipRoll] = 0.0;
  q[RHipRoll] = 0.0;
  q[RHipYawPitch] = 0.0;
  q[LHipYawPitch] = 0.0;
  q[LAnkleRoll] = 0.0;
  q[LAnklePitch] =  -0.25;
  q[RAnkleRoll] = 0.0;
  q[RAnklePitch] =  -0.25;
  q[HeadYaw] = 0.0;
  q[HeadPitch] = 0.0;
  q[LShoulderRoll] = 0.349;
  q[LShoulderPitch] = 1.39;
  q[RShoulderRoll] = -0.349;
  q[RShoulderPitch] = 1.39;
  q[RElbowRoll] = 1.04;
  q[RElbowYaw] = 1.39;
  q[LElbowRoll] = -1.04;
  q[LElbowYaw] = -1.39;
  robot->setPositions(q);
}

int main(int argc, char** argv) {
  double servo_rate;
  bool isRecord;
  bool b_show_joint_frame;
  try {
    YAML::Node simulation_cfg =
    YAML::LoadFile(THIS_COM "Config/Nao/SIMULATION.yaml");
    myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate);
    myUtils::readParameter(simulation_cfg, "is_record", isRecord);
    myUtils::readParameter(simulation_cfg, "show_joint_frame",
                           b_show_joint_frame);
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
  // =========================================================================
  // Generate world and add skeletons
  // =========================================================================
  dart::simulation::WorldPtr world(new dart::simulation::World);
  dart::utils::DartLoader urdfLoader;
  dart::dynamics::SkeletonPtr ground = urdfLoader.parseSkeleton(
      THIS_COM "RobotModel/Ground/ground_terrain.urdf");
  dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
      THIS_COM "RobotModel/Robot/Nao/nao.urdf");

  world->addSkeleton(ground);
  world->addSkeleton(robot);

  // =========================================================================
  // Friction & Restitution Coefficient
  // =========================================================================
  double friction(1.0);
  double restit(0.0);
  ground->getBodyNode("ground_link")->setFrictionCoeff(friction);
  robot->getBodyNode("r_sole")->setFrictionCoeff(friction);
  robot->getBodyNode("l_sole")->setFrictionCoeff(friction);

  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  world->setGravity(gravity);
  world->setTimeStep(servo_rate);

  // =========================================================================
  // Display Joints Frame
  // =========================================================================
  if (b_show_joint_frame) displayJointFrames(world, robot);

  // =========================================================================
  // Initial configuration
  // =========================================================================
  _setInitialConfiguration(robot);

  // =========================================================================
  // Enabel Joit Limits
  // =========================================================================
  _setJointLimitConstraint(robot);

  // =========================================================================
  // Print Model Info
  // =========================================================================
  // _printRobotModel(robot);


  // =========================================================================
  // Wrap a worldnode
  // =========================================================================
  osg::ref_ptr<NaoWorldNode> node;
  node = new NaoWorldNode(world);
  node->setNumStepsPerCycle(30);
  node->setJointTargets(robot->getPositions());

  // =========================================================================
  // Create and Set Viewer
  // =========================================================================
  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.simulate(false);
  viewer.switchHeadlights(false);
  ::osg::Vec3 p1(1.0, 0.2, 1.0);
  p1 = p1 * 0.7;
  viewer.getLightSource(0)->getLight()->setPosition(
      ::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
  viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
  viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  viewer.addEventHandler(new OneStepProgress(node));

  if (isRecord) {
    std::cout << "[Video Record Enable]" << std::endl;
    viewer.record(THIS_COM "/ExperimentVideo");
  }

  // viewer.setUpViewInWindow(0, 0, 2880, 1800);
  viewer.setUpViewInWindow(1440, 0, 500, 500);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.14, 2.28, 3.0) * 1.5, ::osg::Vec3(0.0, 0.2, 0.5),
      ::osg::Vec3(0.0, 0.0, 1.0));
  viewer.setCameraManipulator(viewer.getCameraManipulator());
  viewer.run();

}
