#pragma once
#include <utility>

#include <Configuration.h>
#include <Utils/General/Clock.hpp>
#include <Utils/IO/IOUtilities.hpp>

class RobotSystem;

class NaoStateProvider {
 public:
  static NaoStateProvider* getStateProvider(RobotSystem* _robot);
  ~NaoStateProvider() {}

  void saveCurrentData();

  Clock clock;

  double curr_time;
  double prev_state_machine_time;
  double planning_moment;

  int stance_foot;

  Eigen::VectorXd q;
  Eigen::VectorXd qdot;

  Eigen::VectorXd jpos_ini;

  int b_rfoot_contact;
  int b_lfoot_contact;

  int num_step_copy;
  int phase_copy;
  int planning_id;

  // API related variable
  double ft_length;
  double r_ft_width;
  double l_ft_width;
  double ft_ori_inc;
  int num_total_step;
  int num_residual_step;

  // data manager
  Eigen::VectorXd com_pos;
  Eigen::VectorXd com_vel;
  Eigen::VectorXd mom;

  Eigen::VectorXd com_pos_des;
  Eigen::VectorXd com_vel_des;
  Eigen::VectorXd mom_des;

  Eigen::VectorXd dcm;
  Eigen::VectorXd prev_dcm;
  Eigen::VectorXd dcm_vel;
  double dcm_omega;

  Eigen::VectorXd rf_pos;
  Eigen::VectorXd rf_vel;
  Eigen::VectorXd lf_pos;
  Eigen::VectorXd lf_vel;

  Eigen::VectorXd rf_pos_des;
  Eigen::VectorXd rf_vel_des;
  Eigen::VectorXd lf_pos_des;
  Eigen::VectorXd lf_vel_des;

  Eigen::Quaternion<double> rf_ori_quat;
  Eigen::VectorXd rf_ang_vel;
  Eigen::Quaternion<double> lf_ori_quat;
  Eigen::VectorXd lf_ang_vel;

  Eigen::Quaternion<double> rf_ori_quat_des;
  Eigen::VectorXd rf_ang_vel_des;
  Eigen::Quaternion<double> lf_ori_quat_des;
  Eigen::VectorXd lf_ang_vel_des;

  Eigen::Quaternion<double> pelvis_ori;
  Eigen::VectorXd pelvis_ang_vel;
  Eigen::Quaternion<double> torso_ori;
  Eigen::VectorXd torso_ang_vel;

  Eigen::Quaternion<double> pelvis_ori_des;
  Eigen::VectorXd pelvis_ang_vel_des;
  Eigen::Quaternion<double> torso_ori_des;
  Eigen::VectorXd torso_ang_vel_des;

  Eigen::VectorXd r_rf_des;
  Eigen::VectorXd l_rf_des;
  Eigen::VectorXd r_rf;
  Eigen::VectorXd l_rf;

  Eigen::VectorXd des_jacc_cmd;

 private:
  NaoStateProvider(RobotSystem* _robot);
  RobotSystem* robot_;
};
