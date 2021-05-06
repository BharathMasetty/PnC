#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/NaoPnC/NaoDefinition.hpp>
#include <PnC/NaoPnC/NaoInterface.hpp>
#include <PnC/NaoPnC/NaoStateEstimator.hpp>
#include <PnC/NaoPnC/NaoStateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>

NaoStateEstimator::NaoStateEstimator(RobotSystem* robot) {
  myUtils::pretty_constructor(1, "Nao State Estimator");

  robot_ = robot;
  sp_ = NaoStateProvider::getStateProvider(robot_);
  curr_config_ = Eigen::VectorXd::Zero(Nao::n_dof);
  curr_qdot_ = Eigen::VectorXd::Zero(Nao::n_dof);
}

NaoStateEstimator::~NaoStateEstimator() {}

void NaoStateEstimator::Initialization(NaoSensorData* data) {
  _JointUpdate(data);
  _ConfigurationAndModelUpdate();
  sp_->jpos_ini = curr_config_.segment(Nao::n_vdof, Nao::n_adof);
  _FootContactUpdate(data);
  _UpdateDCM();
  sp_->saveCurrentData();
}

void NaoStateEstimator::Update(NaoSensorData* data) {
  _JointUpdate(data);
  _ConfigurationAndModelUpdate();
  _FootContactUpdate(data);
  _UpdateDCM();
  sp_->saveCurrentData();
}

void NaoStateEstimator::_JointUpdate(NaoSensorData* data) {
  curr_qdot_.setZero();
  for (int i = 0; i < Nao::n_vdof; ++i) {
    curr_config_[i] = data->virtual_q[i];
    curr_qdot_[i] = data->virtual_qdot[i];
  }
  for (int i(0); i < Nao::n_adof; ++i) {
    curr_config_[Nao::n_vdof + i] = data->q[i];
    curr_qdot_[Nao::n_vdof + i] = data->qdot[i];
  }
  sp_->l_rf = data->lf_wrench;
  sp_->r_rf = data->rf_wrench;

}

void NaoStateEstimator::_ConfigurationAndModelUpdate() {
  robot_->updateSystem(curr_config_, curr_qdot_, true);

  sp_->q = curr_config_;
  sp_->qdot = curr_qdot_;
}

void NaoStateEstimator::_FootContactUpdate(NaoSensorData* data) {
  if (data->rfoot_contact)
    sp_->b_rfoot_contact = 1;
  else
    sp_->b_rfoot_contact = 0;
  if (data->lfoot_contact)
    sp_->b_lfoot_contact = 1;
  else
    sp_->b_lfoot_contact = 0;
}

void NaoStateEstimator::_UpdateDCM() {
  sp_->com_pos = robot_->getCoMPosition();
  sp_->com_vel = robot_->getCoMVelocity();
  sp_->dcm_omega = sqrt(9.81 / sp_->com_pos[2]);

  sp_->prev_dcm = sp_->dcm;
  sp_->dcm = robot_->getCoMPosition() + sp_->com_vel / sp_->dcm_omega;

  double alpha_vel = 0.1;
  sp_->dcm_vel =
      alpha_vel * ((sp_->dcm - sp_->prev_dcm) / NaoAux::servo_rate) +
      (1.0 - alpha_vel) * sp_->dcm_vel;
}
