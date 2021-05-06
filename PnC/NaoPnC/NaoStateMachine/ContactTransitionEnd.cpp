#include <PnC/NaoPnC/NaoCtrlArchitecture/NaoControlArchitecture.hpp>
#include <PnC/NaoPnC/NaoStateMachine/ContactTransitionEnd.hpp>

ContactTransitionEnd::ContactTransitionEnd(
    const StateIdentifier state_identifier_in, const int _leg_side,
    NaoControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Contact Transition End");

  // Set Pointer to Control Architecture
  val_ctrl_arch_ = ((NaoControlArchitecture*)_ctrl_arch);
  taf_container_ = val_ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = NaoStateProvider::getStateProvider(robot_);

  // Set Leg Side
  leg_side_ = _leg_side;
}

ContactTransitionEnd::~ContactTransitionEnd() {}

void ContactTransitionEnd::firstVisit() {
  std::cout << "[Contact Transition End] Leg Side: " << leg_side_ << std::endl;
  // Set control Starting time
  ctrl_start_time_ = sp_->curr_time;

  // Ramp Down Reaction force for the upcoming swing foot
  // Ramp to minimum the foot task hierarchy weight in prep for swing
  end_time_ =
      val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampDownTime();
  if (leg_side_ == LEFT_ROBOT_SIDE) {
    val_ctrl_arch_->lfoot_max_normal_force_manager_->initializeRampToZero(
        0.0, end_time_);
    val_ctrl_arch_->lfoot_contact_pos_hierarchy_manager_->initializeRampToMin(
        0.0,
        val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampDownTime());
    val_ctrl_arch_->lfoot_contact_ori_hierarchy_manager_->initializeRampToMin(
        0.0,
        val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampDownTime());
  } else {
    val_ctrl_arch_->rfoot_max_normal_force_manager_->initializeRampToZero(
        0.0, end_time_);
    val_ctrl_arch_->rfoot_contact_pos_hierarchy_manager_->initializeRampToMin(
        0.0,
        val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampDownTime());
    val_ctrl_arch_->rfoot_contact_ori_hierarchy_manager_->initializeRampToMin(
        0.0,
        val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampDownTime());
  }
}

void ContactTransitionEnd::_taskUpdate() {
  // =========================================================================
  // - Compute and update new maximum reaction forces
  // - Ramp to minimum the foot task hierarchy weight in prep for swing
  // =========================================================================
  if (leg_side_ == LEFT_ROBOT_SIDE) {
    val_ctrl_arch_->lfoot_max_normal_force_manager_->updateRampToZeroDesired(
        state_machine_time_);
    val_ctrl_arch_->lfoot_contact_pos_hierarchy_manager_
        ->updateRampToMinDesired(state_machine_time_);
    val_ctrl_arch_->lfoot_contact_ori_hierarchy_manager_
        ->updateRampToMinDesired(state_machine_time_);
  } else {
    val_ctrl_arch_->rfoot_max_normal_force_manager_->updateRampToZeroDesired(
        state_machine_time_);
    val_ctrl_arch_->rfoot_contact_pos_hierarchy_manager_
        ->updateRampToMinDesired(state_machine_time_);
    val_ctrl_arch_->rfoot_contact_ori_hierarchy_manager_
        ->updateRampToMinDesired(state_machine_time_);
  }

  // =========================================================================
  // Set DCM tasks from trajectory manager
  // =========================================================================
  val_ctrl_arch_->dcm_trajectory_manger_->updateDCMTasksDesired(sp_->curr_time);

  // =========================================================================
  // Set Foot Motion Tasks
  // =========================================================================
  val_ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  val_ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void ContactTransitionEnd::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
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
    return NAO_STATES::LL_SWING;
  } else {
    // Switch to right leg swing
    return NAO_STATES::RL_SWING;
  }
}

void ContactTransitionEnd::initialization(const YAML::Node& node) {}
