#include <PnC/ValkyriePnC/ValkyrieCtrlArchitecture/ValkyrieControlArchitecture.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateMachine/DoubleSupportBalance.hpp>

DoubleSupportBalance::DoubleSupportBalance(
    const StateIdentifier state_identifier_in,
    ValkyrieControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Double Support Balance");

  // Set Trigger to false
  state_switch_button_trigger_ = false;

  // Set Pointer to Control Architecture
  val_ctrl_arch_ = ((ValkyrieControlArchitecture*)_ctrl_arch);
  taf_container_ = val_ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = ValkyrieStateProvider::getStateProvider(robot_);
}

DoubleSupportBalance::~DoubleSupportBalance() {}

void DoubleSupportBalance::firstVisit() {
  // Reset Flags
  state_switch_button_trigger_ = false;
  std::cout << "[Double Support Balance] Ready for user commands" << std::endl;
  ctrl_start_time_ = sp_->curr_time;
}

void DoubleSupportBalance::_taskUpdate() {
  // =========================================================================
  // Set Foot Motion Tasks
  // =========================================================================
  val_ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  val_ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void DoubleSupportBalance::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void DoubleSupportBalance::lastVisit() {}

bool DoubleSupportBalance::endOfState() {
  // Also check if footstep list is non-zero
  if (state_switch_button_trigger_ &&
      (val_ctrl_arch_->dcm_trajectory_manger_->footstep_list_.size() > 0) &&
      (!val_ctrl_arch_->dcm_trajectory_manger_->noRemainingSteps())) {
    return true;
  }
  return false;
}

StateIdentifier DoubleSupportBalance::getNextState() {
  int robot_side;
  // Check if there's a valid step
  if (val_ctrl_arch_->dcm_trajectory_manger_->nextStepRobotSide(robot_side)) {
    // Check which side is the next footstep
    if (robot_side == LEFT_ROBOT_SIDE) {
      std::cout << "[DoubleSupportBalance] To Left Contact Transition"
                << std::endl;
      return VALKYRIE_STATES::LL_CONTACT_TRANSITION_START;
    } else {
      std::cout << "[DoubleSupportBalance] To Right Contact Transition"
                << std::endl;
      return VALKYRIE_STATES::RL_CONTACT_TRANSITION_START;
    }
  }
}

void DoubleSupportBalance::initialization(const YAML::Node& node) {}
