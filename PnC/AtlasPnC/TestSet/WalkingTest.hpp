#pragma once

#include <PnC/Test.hpp>

class AtlasStateProvider;
class TVRPlanner;

namespace WkPhase {
constexpr int double_contact_1 = 0;
constexpr int right_swing_start_trans = 1;
constexpr int right_swing = 2;
constexpr int right_swing_end_trans = 3;
constexpr int double_contact_2 = 4;
constexpr int left_swing_start_trans = 5;
constexpr int left_swing = 6;
constexpr int left_swing_end_trans = 7;
constexpr int NUM_WALKING_PHASE = 8;
};  // namespace WkPhase

class WalkingTest : public Test {
   public:
    WalkingTest(RobotSystem*);
    virtual ~WalkingTest();
    virtual void TestInitialization();

   protected:
    int num_step_;
    AtlasStateProvider* sp_;
    virtual int _NextPhase(const int& phase);
    virtual void AdditionalUpdate_();
    void _SettingParameter();

    TVRPlanner* reversal_planner_;

    Controller* body_fix_ctrl_;
    // Right
    Controller* right_swing_start_trans_ctrl_;
    Controller* right_swing_ctrl_;
    Controller* right_swing_end_trans_ctrl_;
    // Left
    Controller* left_swing_start_trans_ctrl_;
    Controller* left_swing_ctrl_;
    Controller* left_swing_end_trans_ctrl_;

    YAML::Node cfg_;

    // Locomotion Behavior
    double walking_start_time_;
    Eigen::Vector2d walking_velocity_;
    double turning_rate_;
    Eigen::Quaternion<double> delta_quat_;
};
