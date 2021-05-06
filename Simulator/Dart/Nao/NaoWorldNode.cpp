#include <Configuration.h>
#include <Simulator/Dart/Nao/NaoWorldNode.hpp>
#include <PnC/NaoPnC/NaoInterface.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

NaoWorldNode::NaoWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
  world_ = _world;
  robot_ = world_->getSkeleton("nao");
  trq_lb_ = robot_->getForceLowerLimits();
  trq_ub_ = robot_->getForceUpperLimits();
  n_dof_ = robot_->getNumDofs();
  ground_ = world_->getSkeleton("ground_skeleton");
  
  trq_cmd_ = Eigen::VectorXd::Zero(n_dof_);

  interface_ = new NaoInterface();
  sensor_data_ = new NaoSensorData();
  command_ = new NaoCommand();

  setJointIndices();
  resetButtonFlags();
  SetParams_();

}

void NaoWorldNode::setJointIndices() {
  LHipYawPitch = robot_->getDof("LHipYawPitch")->getIndexInSkeleton();
  LHipPitch = robot_->getDof("LHipPitch")->getIndexInSkeleton();
  LHipRoll = robot_->getDof("LHipRoll")->getIndexInSkeleton();
  LKneePitch = robot_->getDof("LKneePitch")->getIndexInSkeleton();
  LAnklePitch = robot_->getDof("LAnklePitch")->getIndexInSkeleton();
  LAnkleRoll = robot_->getDof("LAnkleRoll")->getIndexInSkeleton();
  
  RHipYawPitch = robot_->getDof("RHipYawPitch")->getIndexInSkeleton();
  RHipPitch = robot_->getDof("RHipPitch")->getIndexInSkeleton();
  RHipRoll = robot_->getDof("RHipRoll")->getIndexInSkeleton();
  RKneePitch = robot_->getDof("RKneePitch")->getIndexInSkeleton();
  RAnklePitch = robot_->getDof("RAnklePitch")->getIndexInSkeleton();
  RAnkleRoll = robot_->getDof("RAnkleRoll")->getIndexInSkeleton();
  
  LShoulderPitch = robot_->getDof("LShoulderPitch")->getIndexInSkeleton();
  LShoulderRoll = robot_->getDof("LShoulderRoll")->getIndexInSkeleton();
  LElbowYaw = robot_->getDof("LElbowYaw")->getIndexInSkeleton();
  LElbowRoll = robot_->getDof("LElbowRoll")->getIndexInSkeleton();
  
  RShoulderPitch = robot_->getDof("RShoulderPitch")->getIndexInSkeleton();
  RShoulderRoll = robot_->getDof("RShoulderRoll")->getIndexInSkeleton();
  RElbowYaw = robot_->getDof("RElbowYaw")->getIndexInSkeleton();
  RElbowRoll = robot_->getDof("RElbowRoll")->getIndexInSkeleton();
  HeadPitch = robot_->getDof("HeadPitch")->getIndexInSkeleton();
  HeadYaw = robot_->getDof("HeadYaw")->getIndexInSkeleton();

  HeadJointIndices = {HeadPitch, HeadYaw};
  ShoulderJointIndices = {RShoulderPitch, RShoulderRoll, LShoulderPitch, LShoulderRoll};
  KneeJointIndices = {RKneePitch,  LKneePitch};
  AnkleJointIndices = {LAnklePitch, LAnkleRoll,RAnklePitch, RAnkleRoll};
  ElbowJointIndices = {LElbowYaw, LElbowRoll,  RElbowYaw, RElbowRoll};
  HipJointIndices =  {LHipYawPitch, LHipPitch, LHipRoll,  RHipYawPitch, RHipPitch, RHipRoll};
}


NaoWorldNode::~NaoWorldNode() {
  delete interface_;
  delete sensor_data_;
  delete command_;
}


// Main while loop
void NaoWorldNode::customPreStep() {
  t_ = (double)count_ * servo_rate_;

  sensor_data_->q = robot_->getPositions().tail(n_dof_ - 6);
  sensor_data_->virtual_q = robot_->getPositions().head(6);
  sensor_data_->qdot = robot_->getVelocities().tail(n_dof_ - 6);
  sensor_data_->virtual_qdot = robot_->getVelocities().head(6);

  
  // Compute local frame wrenches on the sensor
  GetForceTorqueData_();
  // Use force thresholding to detect contacts

  GetContactSwitchData_(sensor_data_->rfoot_contact,
                        sensor_data_->lfoot_contact);


  // Check for user button presses
  if (b_button_p) {
    interface_->interrupt->b_interrupt_button_p = true;
  }
  if (b_button_r) {
    interface_->interrupt->b_interrupt_button_r = true;
  }
  if (b_button_w) {
    interface_->interrupt->b_interrupt_button_w = true;
  }
  if (b_button_a) {
    interface_->interrupt->b_interrupt_button_a = true;
  }
  if (b_button_s) {
    interface_->interrupt->b_interrupt_button_s = true;
  }
  if (b_button_d) {
    interface_->interrupt->b_interrupt_button_d = true;
  }
  if (b_button_q) {
    interface_->interrupt->b_interrupt_button_q = true;
  }
  if (b_button_e) {
    interface_->interrupt->b_interrupt_button_e = true;
  }
  if (b_button_x) {
    interface_->interrupt->b_interrupt_button_x = true;
  }
  if (b_button_j) {
    interface_->interrupt->b_interrupt_button_j = true;
  }
  if (b_button_k) {
    interface_->interrupt->b_interrupt_button_k = true;
  }

  interface_->getCommand(sensor_data_, command_);
  // std::cout << "command_ size : " << command_->jtrq.size() << std::endl;
  trq_cmd_.tail(n_dof_ - 6) = command_->jtrq;
  
  Eigen::VectorXd q = robot_->getPositions();
  Eigen::VectorXd v = robot_->getVelocities();

  /*
  // Head Joints
  for (int joint : HeadJointIndices){
    dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
    trq_cmd_[joint] += head_scale_ * (kp_ * (command_->q[joint-6] - sensor_data_->q[joint-6]) + 
                                       kd_ * (command_->qdot[joint-6] - sensor_data_->qdot[joint-6]));
    std::cout << dof->getName() << "---" << trq_cmd_[joint-6] << "---" <<  command_->q[joint-6] - sensor_data_->q[joint] << std::endl;
  }

  for (int joint : ShoulderJointIndices){
    dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
    trq_cmd_[joint] += head_scale_ * (kp_ * (command_->q[joint-6] - sensor_data_->q[joint-6]) + 
                                       kd_ * (command_->qdot[joint-6] - sensor_data_->qdot[joint-6]));
    std::cout << dof->getName() << "---" << trq_cmd_[joint-6] << "---" <<  command_->q[joint-6] - sensor_data_->q[joint] << std::endl;
  }
  
  for (int joint : ElbowJointIndices){
    dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
    trq_cmd_[joint] += head_scale_ * (kp_ * (command_->q[joint-6] - sensor_data_->q[joint-6]) + 
                                       kd_ * (command_->qdot[joint-6] - sensor_data_->qdot[joint-6]));
    std::cout << dof->getName() << "---" << trq_cmd_[joint-6] << "---" <<  command_->q[joint-6] - sensor_data_->q[joint] << std::endl; 
  }
  
  for (int joint : HipJointIndices){
    dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
    trq_cmd_[joint] += head_scale_ * (kp_ * (command_->q[joint-6] - sensor_data_->q[joint-6]) + 
                                       kd_ * (command_->qdot[joint-6] - sensor_data_->qdot[joint-6]));
    std::cout << dof->getName() << "---" << trq_cmd_[joint-6] << "---" <<  command_->q[joint-6] - sensor_data_->q[joint] << std::endl;
    
  }
  
  for (int joint : KneeJointIndices){
    dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
    trq_cmd_[joint] += head_scale_ * (kp_ * (command_->q[joint-6] - sensor_data_->q[joint-6]) + 
                                       kd_ * (command_->qdot[joint-6] - sensor_data_->qdot[joint-6]));
    std::cout << dof->getName() << "---" << trq_cmd_[joint-6] << "---" <<  command_->q[joint-6] - sensor_data_->q[joint] << std::endl;
  }
  
  for (int joint : AnkleJointIndices){
    dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
    // std::cout<<"Joint Index: " << joint << std::endl;
    trq_cmd_[joint] += head_scale_ * (kp_ * (command_->q[joint-6] - sensor_data_->q[joint-6]) + 
                                       kd_ * (command_->qdot[joint-6] - sensor_data_->qdot[joint-6]));
    std::cout << dof->getName() << "---" << trq_cmd_[joint-6] << "---" <<  command_->q[joint-6] - sensor_data_->q[joint] << std::endl;
  }
  */
  
  
   // std::cout << "Joint Torque values ##################################" << std::endl;
  // Head Joints
  // for (int joint : HeadJointIndices){
  //   dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
  //   trq_cmd_[joint] = head_scale_*(kp_*(joint_targets_[joint] - q[joint]) - kd_*v[joint]);
  //   // std::cout << dof->getName() << "--------" << trq_cmd_[joint] << "---------" << joint_targets_[joint] - q[joint] << std::endl;
  // }

  // for (int joint : ShoulderJointIndices){
  //   dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
  //   trq_cmd_[joint] = shoulder_scale_*(kp_*(joint_targets_[joint] - q[joint]) - kd_*v[joint]);
  //   // std::cout << dof->getName() << "--------" << trq_cmd_[joint] << "---------" << joint_targets_[joint] - q[joint] << std::endl;
  // }
  
  // for (int joint : ElbowJointIndices){
  //   dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
  //   trq_cmd_[joint] = elbow_scale_*(kp_*(joint_targets_[joint] - q[joint]) - kd_*v[joint]);
  //   // std::cout << dof->getName() << "--------" << trq_cmd_[joint] << "---------" << joint_targets_[joint] - q[joint] << std::endl;
  // }
  
  // for (int joint : HipJointIndices){
  //   dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
  //   trq_cmd_[joint] = hip_scale_*(kp_*(joint_targets_[joint] - q[joint]) - kd_*v[joint]);
  //   // std::cout << dof->getName() << "--------" << trq_cmd_[joint] << "---------" << joint_targets_[joint] - q[joint] << std::endl;
  // }
  
  // for (int joint : KneeJointIndices){
  //   dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
  //   trq_cmd_[joint] = knee_scale_*(kp_*(joint_targets_[joint] - q[joint]) - kd_*v[joint]);
  //   // std::cout << dof->getName() << "--------" << trq_cmd_[joint] << "---------" << joint_targets_[joint] - q[joint] << std::endl;
  // }
  
  // for (int joint : AnkleJointIndices){
  //   dart::dynamics::DegreeOfFreedom* dof = robot_->getDof(joint);
  //   trq_cmd_[joint] = ankle_scale_*(kp_*(joint_targets_[joint] - q[joint]) - kd_*v[joint]);
  //   // std::cout << dof->getName() << "--------" << trq_cmd_[joint] << "---------" << joint_targets_[joint] - q[joint] << std::endl;
  // }
  
  
  trq_cmd_.tail(n_dof_ - 6) = myUtils::CropVector(
      trq_cmd_.tail(n_dof_ - 6),
      trq_lb_.segment(Nao::n_vdof, Nao::n_adof),
      trq_ub_.segment(Nao::n_vdof, Nao::n_adof), "clip trq in sim");
  

  trq_cmd_.head(6).setZero();
  robot_->setForces(trq_cmd_);
  count_++;
  Eigen::Isometry3d r_foot = robot_->getBodyNode("r_sole")->getWorldTransform();
  Eigen::Isometry3d l_foot = robot_->getBodyNode("l_sole")->getWorldTransform();
  // std::cout << "Right foot y location : " << r_foot.translation().y() << " Left foot y location : " << l_foot.translation().y() << std::endl; 


  // reset flags
  resetButtonFlags();
}


// Checking the contact -- reimplement with feet z position.
void NaoWorldNode::GetContactSwitchData_(bool& rfoot_contact,
                                              bool& lfoot_contact) {

  /*
  // Get Sensor Wrench Data
  Eigen::VectorXd rf_wrench = sensor_data_->rf_wrench;
  Eigen::VectorXd lf_wrench = sensor_data_->lf_wrench;

  // Local Z-Force Threshold
  double force_threshold = 10;  // 10 Newtons ~ 1kg. If sensor detects this
                                // force, then we are in contact

  if (fabs(rf_wrench[5]) >= force_threshold) {
    rfoot_contact = true;
  } else {
    rfoot_contact = false;
  }

  if (fabs(lf_wrench[5]) >= force_threshold) {
    lfoot_contact = true;
  } else {
    lfoot_contact = false;
  }*/

 
 // Query Foot Position
 Eigen::Isometry3d r_foot = robot_->getBodyNode("r_sole")->getWorldTransform();
 Eigen::Isometry3d l_foot = robot_->getBodyNode("l_sole")->getWorldTransform();

 // Check which one is greater than zero
 if (r_foot.translation().z() <= 0.0){
    rfoot_contact = true;
 } else {
    rfoot_contact = false;
 }
 if (l_foot.translation().z() <= 0.0){
    lfoot_contact = true;
 } else {
    lfoot_contact = false;
 }
 // std::cout << "Right foot z location : " << r_foot.translation().z() << " Left foot z location : " << l_foot.translation().z() << std::endl; 
 
}

void NaoWorldNode::SetParams_() {
  
 try {
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM "Config/Nao/SIMULATION.yaml");
    myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "kp", kp_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "kd", kd_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "hip_scale", hip_scale_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "knee_scale", knee_scale_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "elbow_scale", elbow_scale_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "ankle_scale", ankle_scale_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "shoulder_scale", shoulder_scale_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "head_scale", head_scale_);
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
  
}


void NaoWorldNode::GetForceTorqueData_() {
  Eigen::VectorXd rf_wrench = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd lf_wrench = Eigen::VectorXd::Zero(6);

  dart::dynamics::BodyNode* lfoot_bn = robot_->getBodyNode("l_sole");
  dart::dynamics::BodyNode* rfoot_bn = robot_->getBodyNode("r_sole");
  const dart::collision::CollisionResult& _result =
      world_->getLastCollisionResult();

  Eigen::VectorXd lf_contact_force_sum = Eigen::VectorXd::Zero(3);
  for (const auto& contact : _result.getContacts()) {
    for (const auto& shapeNode :
         lfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
      // Ensure that we view the force as external.
      double sgn = 1.0;
      if (shapeNode == contact.collisionObject1->getShapeFrame()) {
        sgn = 1.0;
      }
      if (shapeNode == contact.collisionObject2->getShapeFrame()) {
        sgn = -1.0;
      }
      // Perform Adjoint Map to local frame wrench
      if (shapeNode == contact.collisionObject1->getShapeFrame() ||
          shapeNode == contact.collisionObject2->getShapeFrame()) {
        Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
        w_c.tail(3) = (contact.force * sgn);
        lf_contact_force_sum += (contact.force * sgn);
        Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
        T_wc.translation() = contact.point;
        Eigen::Isometry3d T_wa =
            robot_->getBodyNode("l_sole")
                ->getTransform(dart::dynamics::Frame::World());
        Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
        Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
        Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
        w_a = AdT_ca.transpose() * w_c;
        lf_wrench += w_a;
      }
    }
    for (const auto& shapeNode :
         rfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
      // Conditional Check to ensure that we view the force as external.
      double sgn = 1.0;
      if (shapeNode == contact.collisionObject1->getShapeFrame()) {
        sgn = 1.0;
      }
      if (shapeNode == contact.collisionObject2->getShapeFrame()) {
        sgn = -1.0;
      }
      // Perform Adjoint Map to local frame wrench
      if (shapeNode == contact.collisionObject1->getShapeFrame() ||
          shapeNode == contact.collisionObject2->getShapeFrame()) {
        double normal(contact.normal(2));
        Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
        w_c.tail(3) = (contact.force * sgn);
        Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
        T_wc.translation() = contact.point;
        Eigen::Isometry3d T_wa =
            robot_->getBodyNode("r_sole")
                ->getTransform(dart::dynamics::Frame::World());
        Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
        Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
        Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
        w_a = AdT_ca.transpose() * w_c;
        // myUtils::pretty_print(w_a, std::cout, "right");
        rf_wrench += w_a;
      }
    }
  }

  // myUtils::pretty_print(lf_contact_force_sum, std::cout,
  // "lf_contact_force_sum");
  // myUtils::pretty_print(rf_wrench, std::cout, "sensor true local rf_wrench");
  // myUtils::pretty_print(lf_wrench, std::cout, "sensor true local lf_wrench
  // ");
  sensor_data_->lf_wrench = lf_wrench;
  sensor_data_->rf_wrench = rf_wrench;
}
