#include <math.h>
#include <stdio.h>
#include <string>

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/NaoPnC/NaoCtrlArchitecture/NaoControlArchitecture.hpp>
#include <PnC/NaoPnC/NaoInterface.hpp>
#include <PnC/NaoPnC/NaoLogicInterrupt/WalkingInterruptLogic.hpp>
#include <PnC/NaoPnC/NaoStateEstimator.hpp>
#include <PnC/NaoPnC/NaoStateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

NaoInterface::NaoInterface() : EnvInterface() {
  std::string border = "=";
  for (int i = 0; i < 79; ++i) {
    border += "=";
  }
  myUtils::color_print(myColor::BoldCyan, border);
  myUtils::pretty_constructor(0, "Nao Interface");

  robot_ = new RobotSystem(
      6, THIS_COM "RobotModel/Robot/Nao/nao.urdf");
 // use this to make naodefinition.hpp
  // robot_->printRobotInfo();
  
  // Make similar to Valkarie
  state_estimator_ = new NaoStateEstimator(robot_);
  sp_ = NaoStateProvider::getStateProvider(robot_);

  // Initialize empty interrupt class
  interrupt = new InterruptLogic();

  sp_->stance_foot = NaoBodyNode::l_sole;

  count_ = 0;
  waiting_count_ = 2;
  cmd_jpos_ = Eigen::VectorXd::Zero(Nao::n_adof);
  cmd_jvel_ = Eigen::VectorXd::Zero(Nao::n_adof);
  cmd_jtrq_ = Eigen::VectorXd::Zero(Nao::n_adof);

  _ParameterSetting();

  myUtils::color_print(myColor::BoldCyan, border);

  // Ignore for now
  DataManager* data_manager = DataManager::GetDataManager();
  data_manager->RegisterData(&cmd_jpos_, VECT, "jpos_des", Nao::n_adof);
  data_manager->RegisterData(&cmd_jvel_, VECT, "jvel_des", Nao::n_adof);
  data_manager->RegisterData(&cmd_jtrq_, VECT, "command", Nao::n_adof);
  
}

NaoInterface::~NaoInterface() {
  delete robot_;
 delete state_estimator_;
  delete interrupt;
  delete control_architecture_;
}


void NaoInterface::getCommand(void* _data, void* _command) {
  NaoCommand* cmd = ((NaoCommand*)_command);
  NaoSensorData* data = ((NaoSensorData*)_data);

  
  if (!Initialization_(data, cmd)) {
    state_estimator_->Update(data);
    interrupt->processInterrupts();
    control_architecture_->getCommand(cmd);
    CropTorque_(cmd);
  }

  cmd_jtrq_ = cmd->jtrq;
  cmd_jvel_ = cmd->qdot;
  cmd_jpos_ = cmd->q;

  ++count_;
  running_time_ = (double)(count_)*NaoAux::servo_rate;
  sp_->curr_time = running_time_;
  sp_->phase_copy = control_architecture_->getState();
  
}

void NaoInterface::CropTorque_(NaoCommand* cmd) {
  cmd->jtrq = myUtils::CropVector(cmd->jtrq,
                                  robot_->GetTorqueLowerLimits().segment(
                                      Nao::n_vdof, Nao::n_adof),
                                  robot_->GetTorqueUpperLimits().segment(
                                      Nao::n_vdof, Nao::n_adof),
                                  "clip trq in interface");
}


void NaoInterface::_ParameterSetting() {
  try {
    YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/Nao/INTERFACE.yaml");
    std::string test_name =
        myUtils::readParameter<std::string>(cfg, "test_name");
    if (test_name == "walking") {
      // comment this out for nao.
      control_architecture_ = new NaoControlArchitecture(robot_);
      delete interrupt;
      interrupt = new WalkingInterruptLogic(
          static_cast<NaoControlArchitecture*>(control_architecture_));
    } else {
      printf(
          "[Nao Interface] There is no test matching test with "
          "the name\n");
      exit(0);
    }
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}


bool NaoInterface::Initialization_(NaoSensorData* _sensor_data,
                                        NaoCommand* _command) {
  static bool test_initialized(false);
  if (!test_initialized) {
    control_architecture_->ControlArchitectureInitialization();
    test_initialized = true;
  }
  if (count_ < waiting_count_) {
    // Set the waiting cout to 4 
    state_estimator_->Initialization(_sensor_data);
    DataManager::GetDataManager()->start();
    return true;
  }
  return false;
}
