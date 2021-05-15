#include <PnC/NaoPnC/NaoCtrlArchitecture/NaoControlArchitecture.hpp>

NaoControlArchitecture::NaoControlArchitecture(RobotSystem* _robot)
    : ControlArchitecture(_robot) {
  b_state_first_visit_ = true;

  myUtils::pretty_constructor(1, "Nao Control Architecture");
  cfg_ = YAML::LoadFile(
      THIS_COM "Config/Nao/TEST/CONTROL_ARCHITECTURE_PARAMS.yaml");

  sp_ = NaoStateProvider::getStateProvider(robot_);

  // Initialize Main Controller

  // these for nao
  taf_container_ = new NaoTaskAndForceContainer(robot_);
  main_controller_ = new NaoMainController(taf_container_, robot_);


  // Initialize Planner
  dcm_planner_ = new DCMPlanner();

  // Initialize Trajectory managers
  rfoot_trajectory_manager_ = new FootSE3TrajectoryManager(
      taf_container_->rfoot_center_pos_task_,
      taf_container_->rfoot_center_ori_task_, robot_);
  lfoot_trajectory_manager_ = new FootSE3TrajectoryManager(
      taf_container_->lfoot_center_pos_task_,
      taf_container_->lfoot_center_ori_task_, robot_);
  rfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->rfoot_contact_, robot_);
  lfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->lfoot_contact_, robot_);
  upper_body_joint_trajectory_manager_ = new UpperBodyJointTrajectoryManager(
      taf_container_->upper_body_task_, robot_);

  rfoot_contact_pos_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->rfoot_center_pos_task_, robot_);
  rfoot_contact_ori_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->rfoot_center_ori_task_, robot_);
  lfoot_contact_pos_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->lfoot_center_pos_task_, robot_);
  lfoot_contact_ori_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->lfoot_center_ori_task_, robot_);

  dcm_trajectory_manger_ = new DCMTrajectoryManager(
      dcm_planner_, taf_container_->com_task_, taf_container_->torso_ori_task_,
      robot_, NaoBodyNode::l_sole,
      NaoBodyNode::r_sole);


  // Make these for nao
  // Initialize states: add all states to the state machine map
  state_machines_[NAO_STATES::STAND] =
      new DoubleSupportStand(NAO_STATES::STAND, this, robot_);
  state_machines_[NAO_STATES::BALANCE] =
      new DoubleSupportBalance(NAO_STATES::BALANCE, this, robot_);
  
  state_machines_[NAO_STATES::SHIFT] = 
     new DoubleSupportShift(NAO_STATES::SHIFT, this, robot_);

  state_machines_[NAO_STATES::RL_CONTACT_TRANSITION_START] =
      new ContactTransition(NAO_STATES::RL_CONTACT_TRANSITION_START,
                            RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[NAO_STATES::RL_CONTACT_TRANSITION_END] =
      new ContactTransitionEnd(NAO_STATES::RL_CONTACT_TRANSITION_END,
                               RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[NAO_STATES::RL_SWING] = new SwingControl(
      NAO_STATES::RL_SWING, RIGHT_ROBOT_SIDE, this, robot_);

  state_machines_[NAO_STATES::LL_CONTACT_TRANSITION_START] =
      new ContactTransition(NAO_STATES::LL_CONTACT_TRANSITION_START,
                            LEFT_ROBOT_SIDE, this, robot_);
  state_machines_[NAO_STATES::LL_CONTACT_TRANSITION_END] =
      new ContactTransitionEnd(NAO_STATES::LL_CONTACT_TRANSITION_END,
                               LEFT_ROBOT_SIDE, this, robot_);
  state_machines_[NAO_STATES::LL_SWING] = new SwingControl(
      NAO_STATES::LL_SWING, LEFT_ROBOT_SIDE, this, robot_);


  // Set Starting State
  state_ = NAO_STATES::STAND;
  prev_state_ = state_;

  _InitializeParameters();
}

NaoControlArchitecture::~NaoControlArchitecture() {
  delete taf_container_;
  delete main_controller_;
  delete dcm_planner_;

  // Delete the trajectory managers
  delete rfoot_trajectory_manager_;
  delete lfoot_trajectory_manager_;
  delete rfoot_max_normal_force_manager_;
  delete lfoot_max_normal_force_manager_;
  delete dcm_trajectory_manger_;

  delete rfoot_contact_pos_hierarchy_manager_;
  delete rfoot_contact_ori_hierarchy_manager_;
  delete lfoot_contact_pos_hierarchy_manager_;
  delete lfoot_contact_ori_hierarchy_manager_;

  // Delete the state machines
  delete state_machines_[NAO_STATES::STAND];
  delete state_machines_[NAO_STATES::BALANCE];
  delete state_machines_[NAO_STATES::RL_CONTACT_TRANSITION_START];
  delete state_machines_[NAO_STATES::RL_CONTACT_TRANSITION_END];
  delete state_machines_[NAO_STATES::RL_SWING];
  delete state_machines_[NAO_STATES::LL_CONTACT_TRANSITION_START];
  delete state_machines_[NAO_STATES::LL_CONTACT_TRANSITION_END];
  delete state_machines_[NAO_STATES::LL_SWING];
  delete state_machines_[NAO_STATES::SHIFT];
}

void NaoControlArchitecture::ControlArchitectureInitialization() {}

void NaoControlArchitecture::getCommand(void* _command) {
  // Initialize State
  if (b_state_first_visit_) {
    state_machines_[state_]->firstVisit();
    b_state_first_visit_ = false;
  }

  // Update State Machine
  state_machines_[state_]->oneStep();
  // Update Desired of state Independent trajectories
  upper_body_joint_trajectory_manager_->updateDesired();
  // Get Wholebody control commands
  main_controller_->getCommand(_command);

  // Check for State Transitions
  if (state_machines_[state_]->endOfState()) {
    state_machines_[state_]->lastVisit();
    prev_state_ = state_;
    state_ = state_machines_[state_]->getNextState();
    b_state_first_visit_ = true;
  }
  saveData();
};

void NaoControlArchitecture::_InitializeParameters() {
  taf_container_->paramInitialization(cfg_["task_parameters"]);
  main_controller_->ctrlInitialization(cfg_["controller_parameters"]);

  // Trajectory Managers initialization
  rfoot_trajectory_manager_->paramInitialization(
      cfg_["foot_trajectory_parameters"]);
  lfoot_trajectory_manager_->paramInitialization(
      cfg_["foot_trajectory_parameters"]);

  lfoot_max_normal_force_manager_->setMaxFz(55.0);
  rfoot_max_normal_force_manager_->setMaxFz(55.0);
  rfoot_contact_pos_hierarchy_manager_->setMaxGain(40.0);
  rfoot_contact_pos_hierarchy_manager_->setMinGain(10.0);
  rfoot_contact_ori_hierarchy_manager_->setMaxGain(40.0);
  rfoot_contact_ori_hierarchy_manager_->setMinGain(10.0);
  lfoot_contact_pos_hierarchy_manager_->setMaxGain(40.0);
  lfoot_contact_pos_hierarchy_manager_->setMinGain(10.0);
  lfoot_contact_ori_hierarchy_manager_->setMaxGain(40.0);
  lfoot_contact_ori_hierarchy_manager_->setMinGain(10.0);

  dcm_trajectory_manger_->paramInitialization(cfg_["dcm_planner_parameters"]);

  // States Initialization:
  state_machines_[NAO_STATES::STAND]->initialization(
      cfg_["state_stand_params"]);
  state_machines_[NAO_STATES::RL_SWING]->initialization(
      cfg_["state_swing"]);
  state_machines_[NAO_STATES::LL_SWING]->initialization(
      cfg_["state_swing"]);
  state_machines_[NAO_STATES::SHIFT]->initialization(
      cfg_["state_stand_params"]);
}

void NaoControlArchitecture::saveData() {
  // Task weights, Reaction force weights
  /*
  sp_->w_rfoot_pos = rfoot_pos_hierarchy_manager_->current_w_;
  sp_->w_rfoot_ori = rfoot_ori_hierarchy_manager_->current_w_;
  sp_->w_lfoot_pos = lfoot_pos_hierarchy_manager_->current_w_;
  sp_->w_lfoot_ori = lfoot_ori_hierarchy_manager_->current_w_;
  sp_->w_com = com_hierarchy_manager_->current_w_;
  sp_->w_base_ori = base_ori_hierarchy_manager_->current_w_;
  */
  /*  sp_->w_rf_rffront =*/
  // rfoot_front_max_normal_force_manager_->current_max_normal_force_z_;
  // sp_->w_rf_rfback =
  // rfoot_back_max_normal_force_manager_->current_max_normal_force_z_;
  // sp_->w_rf_lffront =
  // lfoot_front_max_normal_force_manager_->current_max_normal_force_z_;
  // sp_->w_rf_lfback =
  /*lfoot_back_max_normal_force_manager_->current_max_normal_force_z_;*/
  /*
  sp_->w_rfoot_fr =
      rfoot_max_normal_force_manager_->current_max_normal_force_z_;
  sp_->w_lfoot_fr =
      lfoot_max_normal_force_manager_->current_max_normal_force_z_;
  */
  // Task desired
  /*
  sp_->rfoot_center_pos_des = rfoot_trajectory_manager_->foot_pos_des_;
  sp_->rfoot_center_vel_des = rfoot_trajectory_manager_->foot_vel_des_;
  sp_->lfoot_center_pos_des = lfoot_trajectory_manager_->foot_pos_des_;
  sp_->lfoot_center_vel_des = lfoot_trajectory_manager_->foot_vel_des_;
  sp_->rfoot_center_quat_des = rfoot_trajectory_manager_->foot_quat_des_;
  sp_->rfoot_center_so3_des = rfoot_trajectory_manager_->foot_ang_vel_des_;
  sp_->lfoot_center_quat_des = lfoot_trajectory_manager_->foot_quat_des_;
  sp_->lfoot_center_so3_des = lfoot_trajectory_manager_->foot_ang_vel_des_;
  sp_->q_task_des = joint_trajectory_manager_->joint_pos_des_;
  sp_->qdot_task_des = joint_trajectory_manager_->joint_vel_des_;
  sp_->q_task = sp_->q.tail(Nao::n_adof);
  sp_->qdot_task = sp_->qdot.tail(Nao::n_adof);
  */
  if (state_ == NAO_STATES::STAND ||
      prev_state_ == NAO_STATES::STAND) {
    //sp_->com_pos_des = floating_base_lifting_up_manager_->com_pos_des_;
    //sp_->com_vel_des = floating_base_lifting_up_manager_->com_vel_des_;
    //sp_->dcm_des = floating_base_lifting_up_manager_->dcm_pos_des_;
    //sp_->dcm_vel_des = floating_base_lifting_up_manager_->dcm_vel_des_;
    //sp_->base_quat_des = floating_base_lifting_up_manager_->base_ori_quat_des_;
    //sp_->base_ang_vel_des =
     //   floating_base_lifting_up_manager_->base_ang_vel_des_;
  } else {
    sp_->com_pos_des = dcm_trajectory_manger_->des_com_pos;
    sp_->com_vel_des = dcm_trajectory_manger_->des_com_vel;
    //sp_->base_quat_des = dcm_trajectory_manager_->des_quat;
    //sp_->base_ang_vel_des = dcm_trajectory_manager_->des_ang_vel;
    //sp_->dcm_des = dcm_trajectory_manager_->des_dcm;
    //sp_->dcm_vel_des = dcm_trajectory_manager_->des_dcm_vel;
  }
}
