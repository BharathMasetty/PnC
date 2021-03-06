#include <PnC/ValkyriePnC/ValkyrieTaskAndForceContainer/ValkyrieTaskAndForceContainer.hpp>

ValkyrieTaskAndForceContainer::ValkyrieTaskAndForceContainer(
    RobotSystem* _robot)
    : TaskAndForceContainer(_robot) {
  _InitializeTasks();
  _InitializeContacts();
}

ValkyrieTaskAndForceContainer::~ValkyrieTaskAndForceContainer() {
  _DeleteTasks();
  _DeleteContacts();
}

void ValkyrieTaskAndForceContainer::_InitializeTasks() {
  myUtils::pretty_constructor(2, "Valkyrie Task And Force Container");

  // CoM and Pelvis Tasks
  // dcm_task_ = new DCMTask(robot_);
  com_task_ = new CoMxyz(robot_);
  ang_momentum_task_ = new AngularMomentumTask(robot_, ValkyrieAux::servo_rate);
  pelvis_ori_task_ = new BasicTask(robot_, BasicTaskType::LINKORI, 3,
                                   ValkyrieBodyNode::pelvis);

  // Set Upper Body Joint Tasks
  upper_body_joint_indices_.clear();
  for (int i = ValkyrieDoF::torsoYaw; i < (ValkyrieDoF::rightForearmYaw + 1);
       i++) {
    upper_body_joint_indices_.push_back(i);
  }
  upper_body_task_ = new SelectedJointTask(robot_, upper_body_joint_indices_);

  // Set Foot Motion Tasks
  rfoot_center_pos_task_ = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3,
                                         ValkyrieBodyNode::rightCOP_Frame);
  lfoot_center_pos_task_ = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3,
                                         ValkyrieBodyNode::leftCOP_Frame);
  rfoot_center_ori_task_ = new BasicTask(robot_, BasicTaskType::LINKORI, 3,
                                         ValkyrieBodyNode::rightCOP_Frame);
  lfoot_center_ori_task_ = new BasicTask(robot_, BasicTaskType::LINKORI, 3,
                                         ValkyrieBodyNode::leftCOP_Frame);

  // Add all tasks initially. Remove later as needed.
  // task_list_.push_back(dcm_task_);
  task_list_.push_back(com_task_);
  task_list_.push_back(ang_momentum_task_); // can ignore this
  task_list_.push_back(pelvis_ori_task_);
  task_list_.push_back(upper_body_task_);

  task_list_.push_back(rfoot_center_pos_task_);
  task_list_.push_back(lfoot_center_pos_task_);
  task_list_.push_back(rfoot_center_ori_task_);
  task_list_.push_back(lfoot_center_ori_task_);
}
void ValkyrieTaskAndForceContainer::_InitializeContacts() {
  rfoot_contact_ = new SurfaceContactSpec(
      robot_, ValkyrieBodyNode::rightCOP_Frame, 0.135, 0.08, 0.9 / sqrt(2.0));
  lfoot_contact_ = new SurfaceContactSpec(
      robot_, ValkyrieBodyNode::leftCOP_Frame, 0.135, 0.08, 0.9 / sqrt(2.0));
  dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();
  rfoot_max_z_ = 1500;
  lfoot_max_z_ = 1500;

  // Set desired reaction forces
  Fd_des_ = Eigen::VectorXd::Zero(dim_contact_);

  // Add all contacts initially. Remove later as needed.
  contact_list_.push_back(rfoot_contact_);
  contact_list_.push_back(lfoot_contact_);
}

void ValkyrieTaskAndForceContainer::_DeleteTasks() {
  // delete dcm_task_;
  delete com_task_;
  delete ang_momentum_task_;
  delete pelvis_ori_task_;
  delete upper_body_task_;
  delete rfoot_center_pos_task_;
  delete lfoot_center_pos_task_;
  delete rfoot_center_ori_task_;
  delete lfoot_center_ori_task_;
  task_list_.clear();
}

void ValkyrieTaskAndForceContainer::_DeleteContacts() {
  delete rfoot_contact_;
  delete lfoot_contact_;
  contact_list_.clear();
}

// Set Parameters
void ValkyrieTaskAndForceContainer::paramInitialization(
    const YAML::Node& node) {
  // Defaults
  // Task Gains
  // COM
  kp_com_ = 50 * Eigen::VectorXd::Ones(3);
  kd_com_ = 5.0 * Eigen::VectorXd::Ones(3);
  // Ang Momentum
  kp_ang_mom_ = Eigen::VectorXd::Zero(3);
  kd_ang_mom_ =
      50.0 * Eigen::VectorXd::Ones(3);  // 100.0*Eigen::VectorXd::Ones(3);
  // Pelvis
  kp_pelvis_ = 50 * Eigen::VectorXd::Ones(3);
  kd_pelvis_ = 5.0 * Eigen::VectorXd::Ones(3);
  // Upper Body Joint
  kp_upper_body_joint_ =
      50.0 * Eigen::VectorXd::Ones(upper_body_joint_indices_.size());
  kd_upper_body_joint_ =
      5.0 * Eigen::VectorXd::Ones(upper_body_joint_indices_.size());
  // Foot
  kp_foot_pos_ = 50 * Eigen::VectorXd::Ones(3);
  kd_foot_pos_ = 5.0 * Eigen::VectorXd::Ones(3);

  kp_foot_ori_ = 50 * Eigen::VectorXd::Ones(3);
  kd_foot_ori_ = 5.0 * Eigen::VectorXd::Ones(3);

  // Task Hierachies
  // Set Hierarchy
  w_task_com_ = 5.0;
  w_task_ang_mom_ = 3.0;
  w_task_pelvis_ = 5.0;
  w_task_upper_body_ = 2.0;
  w_task_foot_contact_ = 20.0;
  w_task_foot_swing_ = 20.0;

  // Try Loading Custom Parmams ----------------------------------
  try {
    double temp_double;
    Eigen::VectorXd temp_vec;

    // Load Maximum normal force
    myUtils::readParameter(node, "max_z_force", rfoot_max_z_);
    myUtils::readParameter(node, "max_z_force", lfoot_max_z_);

    // Load Task Gains
    myUtils::readParameter(node, "kp_com", kp_com_);
    myUtils::readParameter(node, "kd_com", kd_com_);
    myUtils::readParameter(node, "kd_ang_mom", kp_ang_mom_);
    myUtils::readParameter(node, "kp_pelvis", kp_pelvis_);
    myUtils::readParameter(node, "kd_pelvis", kd_pelvis_);
    myUtils::readParameter(node, "kp_upper_body_joint", temp_double);
    kp_upper_body_joint_ =
        temp_double * Eigen::VectorXd::Ones(upper_body_joint_indices_.size());
    myUtils::readParameter(node, "kd_upper_body_joint", temp_double);
    kd_upper_body_joint_ =
        temp_double * Eigen::VectorXd::Ones(upper_body_joint_indices_.size());
    myUtils::readParameter(node, "kp_foot_pos", kp_foot_pos_);
    myUtils::readParameter(node, "kd_foot_pos", kd_foot_pos_);
    myUtils::readParameter(node, "kp_foot_ori", kp_foot_ori_);
    myUtils::readParameter(node, "kd_foot_ori", kd_foot_ori_);

    // Load Task Hierarchies
    myUtils::readParameter(node, "w_task_com", w_task_com_);
    myUtils::readParameter(node, "w_task_ang_mom", w_task_ang_mom_);
    myUtils::readParameter(node, "w_task_pelvis", w_task_pelvis_);
    myUtils::readParameter(node, "w_task_upper_body", w_task_upper_body_);
    myUtils::readParameter(node, "w_task_foot_contact", w_task_foot_contact_);
    myUtils::readParameter(node, "w_task_foot_swing", w_task_foot_swing_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
  //  Loading Custom Parmams ----------------------------------

  // Set Task Gains
  // dcm_task_->setGain(kp_com_, kd_com_);
  com_task_->setGain(kp_com_, kd_com_);
  ang_momentum_task_->setGain(kp_ang_mom_, kd_ang_mom_);
  pelvis_ori_task_->setGain(kp_pelvis_, kd_pelvis_);
  upper_body_task_->setGain(kp_upper_body_joint_, kd_upper_body_joint_);
  rfoot_center_pos_task_->setGain(kp_foot_pos_, kd_foot_pos_);
  lfoot_center_pos_task_->setGain(kp_foot_pos_, kd_foot_pos_);
  rfoot_center_ori_task_->setGain(kp_foot_ori_, kd_foot_ori_);
  lfoot_center_ori_task_->setGain(kp_foot_ori_, kd_foot_ori_);

  // Set Task Hierarchies
  // dcm_task_->setHierarchyWeight(w_task_com_);
  com_task_->setHierarchyWeight(w_task_com_);
  ang_momentum_task_->setHierarchyWeight(w_task_ang_mom_);
  pelvis_ori_task_->setHierarchyWeight(w_task_pelvis_);
  upper_body_task_->setHierarchyWeight(w_task_upper_body_);
  rfoot_center_pos_task_->setHierarchyWeight(w_task_foot_contact_);
  rfoot_center_ori_task_->setHierarchyWeight(w_task_foot_contact_);
  lfoot_center_pos_task_->setHierarchyWeight(w_task_foot_contact_);
  lfoot_center_ori_task_->setHierarchyWeight(w_task_foot_contact_);

  // Set Maximum Forces
  ((SurfaceContactSpec*)rfoot_contact_)->setMaxFz(rfoot_max_z_);
  ((SurfaceContactSpec*)lfoot_contact_)->setMaxFz(lfoot_max_z_);
}
