#pragma once
#include <PnC/ControlArchitecture.hpp>

#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>

#include <PnC/ValkyriePnC/StateMachines/StateMachineSet.hpp>
#include <PnC/ValkyriePnC/CtrlSet/CtrlSet.hpp>

#include <PnC/ValkyriePnC/CtrlSet/ValkyrieMainController.hpp>
#include <PnC/ValkyriePnC/TaskAndForceContainers/ValkyrieTaskAndForceContainer.hpp>

namespace VALKYRIE_STATES {
    constexpr int BALANCE = 0;
};  

class ValkyrieControlArchitecture : public ControlArchitecture {
  public:
    ValkyrieControlArchitecture(RobotSystem* _robot);
    virtual ~ValkyrieControlArchitecture();
    virtual void ControlArchitectureInitialization();
    virtual void getCommand(void* _command);

  protected:
    ValkyrieStateProvider* sp_;
    YAML::Node cfg_;

    void _InitializeParameters();

    // Temporary -------------
    bool b_first_visit_;
    Controller* balance_ctrl_;
    // -----------------------

  public:
    // Task and Force Containers
    ValkyrieTaskAndForceContainer* taf_container_;
    // Controller Object
    ValkyrieMainController* main_controller_;
};
