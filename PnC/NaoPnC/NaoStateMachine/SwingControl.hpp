#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/StateMachine.hpp>
#include <PnC/NaoPnC/NaoStateProvider.hpp>

class NaoControlArchitecture;
class NaoTaskAndForceContainer;
class NaoMainController;

class SwingControl : public StateMachine {
 public:
  SwingControl(const StateIdentifier state_identifier_in, const int _leg_side,
               NaoControlArchitecture* _ctrl_arch, RobotSystem* _robot);
  ~SwingControl();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

 protected:
  NaoStateProvider* sp_;
  NaoControlArchitecture* val_ctrl_arch_;
  NaoTaskAndForceContainer* taf_container_;

  int leg_side_;

  double ctrl_start_time_;
  double end_time_;

  double swing_time_percent_early_contact_check_;
  double early_contact_force_threshold_;

  void _taskUpdate();
};
