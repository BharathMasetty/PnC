 #include <PnC/NaoPnC/NaoCtrlArchitecture/NaoControlArchitecture.hpp>
#include <PnC/NaoPnC/NaoStateMachine/DoubleSupportShift.hpp>

DoubleSupportShift::DoubleSupportShift(
    const StateIdentifier state_identifier_in,
    NaoControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Double Support Shift");

  // Set Pointer to Control Architecture
  nao_ctrl_arch_ = ((NaoControlArchitecture*)_ctrl_arch);
  taf_container_ = nao_ctrl_arch_->taf_container_;

  // Get State Provider
  sp_ = NaoStateProvider::getStateProvider(robot_);

  // Default time to max normal force
  time_to_max_normal_force_ = 0.1;

  // To Do: Belongs to trajectory manager.
  // COM
  ini_com_pos_ = Eigen::VectorXd::Zero(3);
  des_com_pos_ = Eigen::VectorXd::Zero(3);
  des_com_vel_ = Eigen::VectorXd::Zero(3);
  des_com_acc_ = Eigen::VectorXd::Zero(3);
  com_pos_target_ = Eigen::VectorXd::Zero(3);
}

DoubleSupportShift::~DoubleSupportShift() {}

void DoubleSupportShift::firstVisit() {
  std::cout << "[Double Support Shift] Start" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  // =========================================================================
  // Set CoM Trajectory
  // =========================================================================
  ini_com_pos_ = robot_->getCoMPosition();

  Eigen::Vector3d lfoot_pos =
      robot_->getBodyNodeCoMIsometry(NaoBodyNode::l_sole)
          .translation();
  Eigen::Vector3d rfoot_pos =
      robot_->getBodyNodeCoMIsometry(NaoBodyNode::r_sole)
          .translation();
  //std::cout << "DEBUG: initial foot position: \n" << lfoot_pos << "\n Right \n" << rfoot_pos << std::endl;
  des_com_pos_ = 0.5*lfoot_pos + com_pos_target_ ;
  //std::cout << "DEBUG: desired com position: \n" << des_com_pos_ << std::endl;

  _SetBspline(ini_com_pos_, des_com_pos_);

  ini_torso_quat_ = Eigen::Quaternion<double>(
      robot_->getBodyNodeIsometry(NaoBodyNode::torso).linear());

  // =========================================================================
  // Torso Ori Task: Maintain Starting Orientation
  // =========================================================================
  Eigen::VectorXd des_torso_quat = Eigen::VectorXd::Zero(4);
  des_torso_quat << ini_torso_quat_.w(), ini_torso_quat_.x(),
      ini_torso_quat_.y(), ini_torso_quat_.z();
  taf_container_->torso_ori_task_->updateDesired(
      des_torso_quat, Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3));

  // =========================================================================
  // Set Angular Momentum Tasks
  // =========================================================================
  // Eigen::VectorXd zero3 = Eigen::VectorXd::Zero(3);
  // Eigen::VectorXd des_ang_momentum = Eigen::VectorXd::Zero(3);
  // Eigen::VectorXd des_ang_momentum_rate = Eigen::VectorXd::Zero(3);
  // taf_container_->ang_momentum_task_->updateDesired(zero3, des_ang_momentum,
  //                                                   des_ang_momentum_rate);

  // =========================================================================
  // Joint Pos Task
  // =========================================================================
  Eigen::VectorXd jpos_des = sp_->jpos_ini;
  Eigen::VectorXd upper_body_jpos_des(taf_container_->upper_body_joint_indices_.size());
  for (int i=0; i<taf_container_->upper_body_joint_indices_.size(); i++){
    // std::cout << "DEBUG: upper body index : " << taf_container_->upper_body_joint_indices_[i]  << 
    //             " joint angle : " << jpos_des[taf_container_->upper_body_joint_indices_[i]-6] << std::endl;
    upper_body_jpos_des[i] = jpos_des[taf_container_->upper_body_joint_indices_[i]-6];
  }
  // Fixed
  taf_container_->upper_body_task_->updateDesired(
      //jpos_des.tail(taf_container_->upper_body_joint_indices_.size()),
      upper_body_jpos_des,
      Eigen::VectorXd::Zero(Nao::n_adof),
      Eigen::VectorXd::Zero(Nao::n_adof));
  // =========================================================================
  // Initialize Reaction Force Ramp to Max
  // =========================================================================
  nao_ctrl_arch_->lfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, time_to_max_normal_force_);
  nao_ctrl_arch_->rfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, time_to_max_normal_force_);
}

void DoubleSupportShift::_taskUpdate() {
  // =========================================================================
  // Update CoM Task
  // =========================================================================
  _GetBsplineTrajectory();
  for (int i = 0; i < 3; ++i) {
    (sp_->com_pos_des)[i] = des_com_pos_[i];
    (sp_->com_vel_des)[i] = des_com_vel_[i];
  }

  double dcm_omega = sqrt(9.81 / robot_->getCoMPosition()[2]);
  Eigen::VectorXd dcm_pos_des_ =
      sp_->com_pos_des + sp_->com_vel_des / dcm_omega;
  Eigen::VectorXd dcm_vel_des_ = sp_->com_vel_des + des_com_acc_ / dcm_omega;
  dcm_pos_des_[2] = sp_->com_pos_des[2];
  dcm_vel_des_[2] = sp_->com_vel_des[2];
  Eigen::VectorXd dcm_acc_des_ = Eigen::VectorXd::Zero(3);
  dcm_acc_des_[2] = des_com_acc_[2];

  // Print these for stand
  taf_container_->com_task_->updateDesired(sp_->com_pos_des, sp_->com_vel_des,
                                           des_com_acc_);
  
  // std::cout << "DEBUG: com desired: " << sp_->com_pos_des[0] << " " << sp_->com_pos_des[1] << " " << sp_->com_pos_des[2] <<  std::endl;

  // =========================================================================
  // Set Foot Motion Tasks
  // =========================================================================
  nao_ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  nao_ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void DoubleSupportShift::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Compute and update new maximum reaction forces
  nao_ctrl_arch_->lfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);
  nao_ctrl_arch_->rfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);

  _taskUpdate();
}

void DoubleSupportShift::lastVisit() {}

bool DoubleSupportShift::endOfState() {
  if (state_machine_time_ > end_time_) {
    // std::cout << "[DoubleSupportShift] State End" << std::endl;
    return true;
  }
  return false;
}

StateIdentifier DoubleSupportShift::getNextState() {
  return NAO_STATES::BALANCE;
}

void DoubleSupportShift::_SetBspline(const Eigen::VectorXd st_pos,
                                     const Eigen::VectorXd des_pos) {
  // Trajectory Setup
  double init[9];
  double fin[9];
  double** middle_pt = new double*[1];
  middle_pt[0] = new double[3];
  Eigen::Vector3d middle_pos;

  middle_pos = (st_pos + des_pos) / 2.;

  // Initial and final position & velocity & acceleration
  for (int i(0); i < 3; ++i) {
    // Initial
    init[i] = st_pos[i];
    init[i + 3] = 0.;
    init[i + 6] = 0.;
    // Final
    fin[i] = des_pos[i];
    fin[i + 3] = 0.;
    fin[i + 6] = 0.;
    // Middle
    middle_pt[0][i] = middle_pos[i];
  }
  // TEST
  // fin[5] = amplitude_[] * omega_;
  com_traj_.SetParam(init, fin, middle_pt, end_time_ / 2.0);

  delete[] * middle_pt;
  delete[] middle_pt;
}

void DoubleSupportShift::_GetBsplineTrajectory() {
  double pos[3];
  double vel[3];
  double acc[3];

  com_traj_.getCurvePoint(state_machine_time_, pos);
  com_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
  com_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

  for (int i(0); i < 3; ++i) {
    des_com_pos_[i] = pos[i];
    des_com_vel_[i] = vel[i];
    des_com_acc_[i] = acc[i];
  }
}

void DoubleSupportShift::initialization(const YAML::Node& node) {
  try {
    myUtils::readParameter(node, "target_pos_duration", end_time_);
    myUtils::readParameter(node, "time_to_max_normal_force",
                           time_to_max_normal_force_);
    myUtils::readParameter(node, "com_pos_target", com_pos_target_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}
