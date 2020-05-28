#pragma once

#include <PnC/StateMachine.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>

#include <Utils/Math/BSplineBasic.h>

// Forward Declare. Will need to be defined in cpp file.
class ValkyrieControlArchitecture;
class ValkyrieTaskAndForceContainer;
class ValkyrieMainController;

class InitialTransfer : public StateMachine{
  public:
  	InitialTransfer(const StateIdentifier state_identifier_in, 
  					       ValkyrieControlArchitecture* _ctrl_arch, 
  					       RobotSystem* _robot);
  	~InitialTransfer();

    void oneStep(); 
    void firstVisit(); 
    void lastVisit(); 
    bool endOfState(); 
    void initialization(const YAML::Node& node); 
    StateIdentifier getNextState(); 

  protected:
    ValkyrieStateProvider* sp_;
    ValkyrieControlArchitecture* val_ctrl_arch_;
    ValkyrieTaskAndForceContainer* taf_container_;

    double ctrl_start_time_;
    double end_time_;

    void _taskUpdate();
};