#pragma once

#include <PnC/TaskAndForceContainer.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieTask/AngularMomentumTask.hpp>
#include <PnC/ValkyriePnC/ValkyrieTask/CoMxyz.hpp>
#include <PnC/ValkyriePnC/ValkyrieTask/DCMTask.hpp>
#include <PnC/ValkyriePnC/ValkyrieTask/SelectedJointTask.hpp>
#include <PnC/WBC/BasicContactSpec.hpp>
#include <PnC/WBC/BasicTask.hpp>

// Object which publicly contains all the tasks, contacts and reaction forces
class ValkyrieTaskAndForceContainer : public TaskAndForceContainer {
 public:
  ValkyrieTaskAndForceContainer(RobotSystem* _robot);
  ~ValkyrieTaskAndForceContainer();
  void paramInitialization(const YAML::Node& node);

 protected:
  void _InitializeTasks();
  void _InitializeContacts();
  void _DeleteTasks();
  void _DeleteContacts();

 public:
  // -------------------------------------------------------
  // Task Member variables
  // -------------------------------------------------------
  // Task* dcm_task_;
  Task* com_task_;
  Task* pelvis_ori_task_;
  Task* ang_momentum_task_;
  Task* upper_body_task_;
  std::vector<int> upper_body_joint_indices_;  // list of upperbody joint names

  Task* rfoot_center_pos_task_;
  Task* lfoot_center_pos_task_;
  Task* rfoot_center_ori_task_;
  Task* lfoot_center_ori_task_;

  // -------------------------------------------------------
  // Contact Member variables
  // -------------------------------------------------------
  ContactSpec* rfoot_contact_;
  ContactSpec* lfoot_contact_;
  int dim_contact_;
  double lfoot_max_z_;
  double rfoot_max_z_;
  Eigen::VectorXd Fd_des_;

  // -------------------------------------------------------
  // Parameters
  // -------------------------------------------------------
  // Task Hierarchy Weights
  Eigen::VectorXd w_task_hierarchy_;
  double w_task_com_;
  double w_task_ang_mom_;
  double w_task_upper_body_;
  double w_task_pelvis_;
  double w_task_foot_contact_;
  double w_task_foot_swing_;

  // Task Gains
  Eigen::VectorXd kp_com_;
  Eigen::VectorXd kd_com_;
  Eigen::VectorXd kp_ang_mom_;
  Eigen::VectorXd kd_ang_mom_;
  Eigen::VectorXd kp_pelvis_;
  Eigen::VectorXd kd_pelvis_;
  Eigen::VectorXd kp_joint_;
  Eigen::VectorXd kd_joint_;
  Eigen::VectorXd kp_upper_body_joint_;
  Eigen::VectorXd kd_upper_body_joint_;
  Eigen::VectorXd kp_foot_pos_;
  Eigen::VectorXd kd_foot_pos_;
  Eigen::VectorXd kp_foot_ori_;
  Eigen::VectorXd kd_foot_ori_;
};
