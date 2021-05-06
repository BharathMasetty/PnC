#pragma once

#include <PnC/InterruptLogic.hpp>

// Forward Declare Control Architecture
class NaoControlArchitecture;

class WalkingInterruptLogic : public InterruptLogic {
 public:
  WalkingInterruptLogic(NaoControlArchitecture* val_ctrl_arch_);
  ~WalkingInterruptLogic();

  void processInterrupts();

  NaoControlArchitecture* val_ctrl_arch_;
};
