#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/ValkyriePnC/ValkyrieDefinition.hpp"

class ValkyrieStateProvider;
class ValkyrieStateEstimator;

class ValkyrieSensorData {
 public:
  ValkyrieSensorData() {
    q = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    qdot = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    virtual_q = Eigen::VectorXd::Zero(Valkyrie::n_vdof);
    virtual_qdot = Eigen::VectorXd::Zero(Valkyrie::n_vdof);

    // Ignore
    lf_wrench = Eigen::VectorXd::Zero(6);
    rf_wrench = Eigen::VectorXd::Zero(6);
    
    rfoot_contact = false;
    lfoot_contact = false;
  }
  virtual ~ValkyrieSensorData() {}

  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd virtual_q;
  Eigen::VectorXd virtual_qdot;
  Eigen::VectorXd lf_wrench;
  Eigen::VectorXd rf_wrench;
  bool rfoot_contact;
  bool lfoot_contact;
};

// Exactly same
class ValkyrieCommand {
 public:
  ValkyrieCommand() {
    q = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    qdot = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    jtrq = Eigen::VectorXd::Zero(Valkyrie::n_adof);
  }
  virtual ~ValkyrieCommand() {}

  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd jtrq;
};


class ValkyrieInterface : public EnvInterface {
 protected:
  void _ParameterSetting();

  ValkyrieStateEstimator* state_estimator_;
  ValkyrieStateProvider* sp_;

  void CropTorque_(ValkyrieCommand*);
  bool Initialization_(ValkyrieSensorData*, ValkyrieCommand*);

  int count_;
  int waiting_count_;
  Eigen::VectorXd cmd_jpos_;
  Eigen::VectorXd cmd_jvel_;
  Eigen::VectorXd cmd_jtrq_;

 public:
  ValkyrieInterface();
  virtual ~ValkyrieInterface();
  // Main method
  virtual void getCommand(void* _sensor_data, void* _command_data);
};
