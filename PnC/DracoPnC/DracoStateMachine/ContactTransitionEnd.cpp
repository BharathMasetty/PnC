#include <PnC/DracoPnC/DracoCtrlArchitecture/DracoCtrlArchitecture.hpp>
#include <PnC/DracoPnC/DracoStateMachine/ContactTransitionEnd.hpp>

ContactTransitionEnd::ContactTransitionEnd(
    const StateIdentifier state_identifier_in, const int _leg_side,
    DracoControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Contact Transition End");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((DracoControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = DracoStateProvider::getStateProvider(robot_);

  // Set Leg Side
  leg_side_ = _leg_side;
}

ContactTransitionEnd::~ContactTransitionEnd() {}

void ContactTransitionEnd::firstVisit() {
  // Set control Starting time
  if (state_identity_ == DRACO_STATES::RL_CONTACT_TRANSITION_END) {
    std::cout << "[Right Foot Contact Transition End]" << std::endl;
  } else {
    std::cout << "[Left Foot Contact Transition End]" << std::endl;
  }
  ctrl_start_time_ = sp_->curr_time;

  // Ramp Down Reaction force for the upcoming swing foot
  // Ramp to minimum the foot task hierarchy weight in prep for swing
  end_time_ = ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampDownTime();
  if (leg_side_ == LEFT_ROBOT_SIDE) {
    /*    ctrl_arch_->lfoot_front_max_normal_force_manager_->initializeRampToZero(*/
    // 0.0, end_time_);
    // ctrl_arch_->lfoot_back_max_normal_force_manager_->initializeRampToZero(
    /*0.0, end_time_);*/
    ctrl_arch_->lfoot_max_normal_force_manager_->initializeRampToZero(
        0.0, end_time_);
    ctrl_arch_->lfoot_pos_hierarchy_manager_->initializeRampToMin(
        0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampDownTime());
    ctrl_arch_->lfoot_ori_hierarchy_manager_->initializeRampToMin(
        0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampDownTime());
  } else {
    /*    ctrl_arch_->rfoot_front_max_normal_force_manager_->initializeRampToZero(*/
    // 0.0, end_time_);
    // ctrl_arch_->rfoot_back_max_normal_force_manager_->initializeRampToZero(
    /*0.0, end_time_);*/
    ctrl_arch_->rfoot_max_normal_force_manager_->initializeRampToZero(
        0.0, end_time_);
    ctrl_arch_->rfoot_pos_hierarchy_manager_->initializeRampToMin(
        0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampDownTime());
    ctrl_arch_->rfoot_ori_hierarchy_manager_->initializeRampToMin(
        0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampDownTime());
  }
}

void ContactTransitionEnd::_taskUpdate() {
  // =========================================================================
  // Floating Base
  // =========================================================================
  ctrl_arch_->dcm_trajectory_manager_->updateDCMTasksDesired(sp_->curr_time);

  // For TOWR+
  // Eigen::Vector3d des_com_pos =
  // sp_->one_step_sol->get_des_com_pos(sp_->curr_time - sp_->planning_time);
  // Eigen::Vector3d des_com_vel =
  // sp_->one_step_sol->get_des_com_pos(sp_->curr_time - sp_->planning_time);
  // Eigen::Quaternion<double> des_quat =
  // sp_->one_step_sol->get_des_base_quat(sp_->curr_time - sp_->planning_time);
  // Eigen::Vector3d des_base_ang_vel = sp_->one_step_sol->get_des_base_ang_vel(
  // sp_->curr_time - sp_->planning_time);
  // ctrl_arch_->dcm_trajectory_manager_->updateDCMTasksDesired(
  // des_com_pos, des_com_vel, des_quat, des_base_ang_vel);

  // =========================================================================
  // Foot
  // =========================================================================
  ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void ContactTransitionEnd::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // =========================================================================
  // - Compute and update new maximum reaction forces
  // - Ramp to minimum the foot task hierarchy weight in prep for swing
  // =========================================================================
  if (leg_side_ == LEFT_ROBOT_SIDE) {
    /*    ctrl_arch_->lfoot_front_max_normal_force_manager_->updateRampToZeroDesired(*/
    // state_machine_time_);
    // ctrl_arch_->lfoot_back_max_normal_force_manager_->updateRampToZeroDesired(
    /*state_machine_time_);*/
    ctrl_arch_->lfoot_max_normal_force_manager_->updateRampToZeroDesired(
        state_machine_time_);
    ctrl_arch_->lfoot_pos_hierarchy_manager_->updateRampToMinDesired(
        state_machine_time_);
    ctrl_arch_->lfoot_ori_hierarchy_manager_->updateRampToMinDesired(
        state_machine_time_);
  } else {
    /*    ctrl_arch_->rfoot_front_max_normal_force_manager_->updateRampToZeroDesired(*/
    // state_machine_time_);
    // ctrl_arch_->rfoot_back_max_normal_force_manager_->updateRampToZeroDesired(
    /*state_machine_time_);*/
    ctrl_arch_->rfoot_max_normal_force_manager_->updateRampToZeroDesired(
        state_machine_time_);
    ctrl_arch_->rfoot_pos_hierarchy_manager_->updateRampToMinDesired(
        state_machine_time_);
    ctrl_arch_->rfoot_ori_hierarchy_manager_->updateRampToMinDesired(
        state_machine_time_);
  }

  _taskUpdate();
}

void ContactTransitionEnd::lastVisit() {}

bool ContactTransitionEnd::endOfState() {
  // if time exceeds transition time, switch state
  if (state_machine_time_ >= end_time_) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier ContactTransitionEnd::getNextState() {
  if (leg_side_ == LEFT_ROBOT_SIDE) {
    // Switch to left leg swing
    return DRACO_STATES::LL_SWING;
  } else {
    // Switch to right leg swing
    return DRACO_STATES::RL_SWING;
  }
}

void ContactTransitionEnd::initialization(const YAML::Node& node) {}
