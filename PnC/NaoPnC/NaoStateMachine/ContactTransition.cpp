#include <PnC/NaoPnC/NaoCtrlArchitecture/NaoControlArchitecture.hpp>
#include <PnC/NaoPnC/NaoStateMachine/ContactTransition.hpp>

ContactTransition::ContactTransition(const StateIdentifier state_identifier_in,
                                     const int _leg_side,
                                     NaoControlArchitecture* _ctrl_arch,
                                     RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Contact Transition");

  // Set Pointer to Control Architecture
  val_ctrl_arch_ = ((NaoControlArchitecture*)_ctrl_arch);
  taf_container_ = val_ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = NaoStateProvider::getStateProvider(robot_);

  // Set Leg Side
  leg_side_ = _leg_side;
}

ContactTransition::~ContactTransition() {}

void ContactTransition::firstVisit() {
  std::cout << "[Contact Transition Start] Leg Side: " << leg_side_
            << std::endl;
  // Set control Starting time
  ctrl_start_time_ = sp_->curr_time;

  // For all contact transitions, initially ramp up the reaction forces to max
  val_ctrl_arch_->lfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());
  val_ctrl_arch_->rfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());

  // Ramp to max the contact hierarchy weight
  val_ctrl_arch_->lfoot_contact_pos_hierarchy_manager_->initializeRampToMax(
      0.0, val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());
  val_ctrl_arch_->lfoot_contact_ori_hierarchy_manager_->initializeRampToMax(
      0.0, val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());
  val_ctrl_arch_->rfoot_contact_pos_hierarchy_manager_->initializeRampToMax(
      0.0, val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());
  val_ctrl_arch_->rfoot_contact_ori_hierarchy_manager_->initializeRampToMax(
      0.0, val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());

  // Check if it's the last footstep
  if (val_ctrl_arch_->dcm_trajectory_manger_->noRemainingSteps()) {
    // std::cout << "Final Step. Settling..." << std::endl;
    // If this is the last footstep, then we will just wait until we settle.
    end_time_ =
        val_ctrl_arch_->dcm_trajectory_manger_->getFinalContactTransferTime();
  } else {
    // std::cout << "Not the last step. Compute DCM trajectory" << std::endl;
    // This is not the last footstep. We need to recompute the remaining DCM
    // trajectories.
    // Set transfer type to midstep
    int transfer_type = DCM_TRANSFER_TYPES::MIDSTEP;
    end_time_ =
        val_ctrl_arch_->dcm_trajectory_manger_
            ->getMidStepContactTransferTime() -
        val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampDownTime();

    // If coming from a balancing state, use initial transfer time
    if (val_ctrl_arch_->getPrevState() == NAO_STATES::BALANCE) {
      transfer_type = DCM_TRANSFER_TYPES::INITIAL;
      end_time_ =
          val_ctrl_arch_->dcm_trajectory_manger_
              ->getInitialContactTransferTime() -
          val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampDownTime();
    }

    // Recompute DCM Trajectories
    double t_walk_start = ctrl_start_time_;
    Eigen::Quaterniond torso_ori(
        robot_->getBodyNodeCoMIsometry(NaoBodyNode::torso).linear());
    val_ctrl_arch_->dcm_trajectory_manger_->initialize(
        t_walk_start, transfer_type, torso_ori, sp_->dcm, sp_->dcm_vel);
  }
}

void ContactTransition::_taskUpdate() {
  // =========================================================================
  // Compute and update new maximum reaction forces
  // =========================================================================
  val_ctrl_arch_->lfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);
  val_ctrl_arch_->rfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);

  // =========================================================================
  // Ramp task hierarchy weights
  // =========================================================================
  val_ctrl_arch_->lfoot_contact_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  val_ctrl_arch_->lfoot_contact_ori_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  val_ctrl_arch_->rfoot_contact_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  val_ctrl_arch_->rfoot_contact_ori_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);

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

void ContactTransition::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void ContactTransition::lastVisit() {}

bool ContactTransition::endOfState() {
  // if time exceeds transition time, switch state
  if (state_machine_time_ >= end_time_) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier ContactTransition::getNextState() {
  if (val_ctrl_arch_->dcm_trajectory_manger_
          ->noRemainingSteps()) {  // or pause walking
    // Return to balancing
    return NAO_STATES::BALANCE;
  } else {
    if (leg_side_ == LEFT_ROBOT_SIDE) {
      // To left leg contact transition end
      return NAO_STATES::LL_CONTACT_TRANSITION_END;
    } else if (leg_side_ == RIGHT_ROBOT_SIDE) {
      // To right leg contact transition end
      return NAO_STATES::RL_CONTACT_TRANSITION_END;
    }
  }
}

void ContactTransition::initialization(const YAML::Node& node) {}
