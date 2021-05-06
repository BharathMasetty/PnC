#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/StateMachine.hpp>
#include <PnC/NaoPnC/NaoStateProvider.hpp>

class NaoControlArchitecture;
class NaoTaskAndForceContainer;
class NaoMainController;

class DoubleSupportBalance : public StateMachine {
 public:
  DoubleSupportBalance(const StateIdentifier state_identifier_in,
                       NaoControlArchitecture* _ctrl_arch,
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
  NaoStateProvider* sp_;
  NaoControlArchitecture* nao_ctrl_arch_;
  NaoTaskAndForceContainer* taf_container_;

  double ctrl_start_time_;
  double end_time_;

  bool state_switch_button_trigger_;

  void _taskUpdate();
};
