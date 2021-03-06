#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/StateMachine.hpp>

class DracoControlArchitecture;
class DracoTaskAndForceContainer;

class DoubleSupportBalance : public StateMachine {
 public:
  DoubleSupportBalance(const StateIdentifier state_identifier_in,
                       DracoControlArchitecture* _ctrl_arch,
                       RobotSystem* _robot);
  ~DoubleSupportBalance();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

  void switchStateButtonTrigger() { state_switch_button_trigger_ = true; }

 protected:
  DracoStateProvider* sp_;
  DracoControlArchitecture* ctrl_arch_;
  DracoTaskAndForceContainer* taf_container_;

  double ctrl_start_time_;
  double end_time_;

  bool state_switch_button_trigger_;

  void _taskUpdate();
};
