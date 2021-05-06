#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/StateMachine.hpp>
#include <PnC/NaoPnC/NaoStateProvider.hpp>

class NaoControlArchitecture;
class NaoTaskAndForceContainer;
class NaoMainController;

class DoubleSupportStand : public StateMachine {
 public:
  DoubleSupportStand(const StateIdentifier state_identifier_in,
                     NaoControlArchitecture* _ctrl_arch,
                     RobotSystem* _robot);
  ~DoubleSupportStand();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

 protected:
  NaoStateProvider* sp_;
  NaoControlArchitecture* nao_ctrl_arch_;
  NaoTaskAndForceContainer* taf_container_;

  double ctrl_start_time_;
  double end_time_;
  double time_to_max_normal_force_;

  void _taskUpdate();

  // // Stand Trajectory
  Eigen::VectorXd ini_com_pos_;
  Eigen::VectorXd des_com_pos_;
  Eigen::VectorXd des_com_vel_;
  Eigen::VectorXd des_com_acc_;
  Eigen::VectorXd com_pos_target_;

  BS_Basic<3, 3, 1, 2, 2> com_traj_;
  Eigen::Quaternion<double> ini_torso_quat_;

  void _SetBspline(const Eigen::VectorXd st_pos, const Eigen::VectorXd des_pos);
  void _GetBsplineTrajectory();
};
