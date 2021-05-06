#pragma once

#include <Configuration.h>
#include <Eigen/Dense>

class NaoStateProvider;
class RobotSystem;
class NaoSensorData;

class NaoStateEstimator {
 public:
  NaoStateEstimator(RobotSystem* robot);
  ~NaoStateEstimator();

  void Initialization(NaoSensorData*);
  void Update(NaoSensorData*);

 protected:
  NaoStateProvider* sp_;
  RobotSystem* robot_;

  Eigen::VectorXd curr_config_;
  Eigen::VectorXd curr_qdot_;

  void _JointUpdate(NaoSensorData* data);
  void _ConfigurationAndModelUpdate();
  void _FootContactUpdate(NaoSensorData* data);
  void _UpdateDCM();
};
