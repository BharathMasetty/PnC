#pragma once

#include <PnC/Controller.hpp>
#include <PnC/ControlArchitecture.hpp>

#include <PnC/WBC/IHWBC/IHWBC.hpp>
#include <PnC/WBC/IHWBC/IHWBC_JointIntegrator.hpp>

#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>

// Forward Declare Inter-dependent Classes
class ValkyrieControlArchitecture;

class ValkyrieMainController : public Controller {
  public:
    ValkyrieMainController(ControlArchitecture* _ctrl_arch, RobotSystem* _robot);
    virtual ~ValkyrieMainController();

    void getCommand(void* _cmd);
    virtual void ctrlInitialization(const YAML::Node& node);

  protected:
    ValkyrieControlArchitecture* val_ctrl_arch_;
    ValkyrieStateProvider* sp_;

    // -------------------------------------------------------
    // Controller Objects
    // -------------------------------------------------------
    IHWBC* ihwbc_;
    Eigen::VectorXd tau_cmd_;
    Eigen::VectorXd qddot_cmd_;

    IHWBC_JointIntegrator* ihwbc_joint_integrator_;
    Eigen::VectorXd des_jacc_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;

    // -------------------------------------------------------
    // Parameters
    // -------------------------------------------------------   
    // IHWBC Controller parameters
    double ihwbc_dt_; // Joint integration time
    double w_contact_weight_; // contact weights hierarchy (relative to task weights)
    double lambda_qddot_; // Joint acceleration variable normalization
    double lambda_Fr_; //  Reaction Force normalization


  // Parent Functions not used
  virtual void oneStep(void* _cmd);
  virtual void firstVisit();
  virtual void lastVisit();
  virtual bool endOfPhase();
};