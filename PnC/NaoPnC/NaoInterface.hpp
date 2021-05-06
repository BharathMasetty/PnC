#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/NaoPnC/NaoDefinition.hpp"


class NaoStateProvider;
class NaoStateEstimator;

class NaoSensorData {
 public:
  NaoSensorData() {
    q = Eigen::VectorXd::Zero(Nao::n_adof);
    qdot = Eigen::VectorXd::Zero(Nao::n_adof);
    virtual_q = Eigen::VectorXd::Zero(Nao::n_vdof);
    virtual_qdot = Eigen::VectorXd::Zero(Nao::n_vdof);

    // Ignore
    lf_wrench = Eigen::VectorXd::Zero(6);
    rf_wrench = Eigen::VectorXd::Zero(6);
    
    rfoot_contact = false;
    lfoot_contact = false;
  }
  virtual ~NaoSensorData() {}

  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd virtual_q;
  Eigen::VectorXd virtual_qdot;
  Eigen::VectorXd lf_wrench;
  Eigen::VectorXd rf_wrench;
  bool rfoot_contact;
  bool lfoot_contact;
};


class NaoCommand {
 public:
  NaoCommand() {
    q = Eigen::VectorXd::Zero(Nao::n_adof);
    qdot = Eigen::VectorXd::Zero(Nao::n_adof);
    jtrq = Eigen::VectorXd::Zero(Nao::n_adof);
  }
  virtual ~NaoCommand() {}

  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd jtrq;
};



class NaoInterface : public EnvInterface {
 protected:
  void _ParameterSetting();
  NaoStateEstimator* state_estimator_;
  NaoStateProvider* sp_;

  void CropTorque_(NaoCommand*);
  bool Initialization_(NaoSensorData*, NaoCommand*);

  int count_;
  int waiting_count_;
  Eigen::VectorXd cmd_jpos_;
  Eigen::VectorXd cmd_jvel_;
  Eigen::VectorXd cmd_jtrq_;

 public:
  NaoInterface();
  virtual ~NaoInterface();
  // Main method
  virtual void getCommand(void* _sensor_data, void* _command_data);
};
