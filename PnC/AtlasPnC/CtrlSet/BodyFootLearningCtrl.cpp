#include <Configuration.h>
#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/ContactSet/ContactSet.hpp>
#include <PnC/AtlasPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <PnC/PlannerSet/LIPMPlanner/TVRPlanner.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <ReinforcementLearning/RLInterface/RLInterface.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

BodyFootLearningCtrl::BodyFootLearningCtrl(RobotSystem* robot, int swing_foot,
                                           TVRPlanner* planner)
    : SwingPlanningCtrl(robot, swing_foot, planner),
      push_down_height_(0.),
      des_jpos_(Atlas::n_adof),
      des_jvel_(Atlas::n_adof),
      des_jacc_(Atlas::n_adof),
      waiting_time_limit_(0.02),
      Kp_(Atlas::n_adof),
      Kd_(Atlas::n_adof) {
    myUtils::pretty_constructor(2, "Body Foot Learning Ctrl");
    des_jacc_.setZero();
    rfoot_contact_ = new SurfaceContactSpec(robot_, AtlasBodyNode::r_sole,
                                            0.125, 0.075, 0.9);
    lfoot_contact_ = new SurfaceContactSpec(robot_, AtlasBodyNode::l_sole,
                                            0.125, 0.075, 0.9);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    body_pos_task_ =
        new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, AtlasBodyNode::pelvis);
    body_ori_task_ =
        new BasicTask(robot_, BasicTaskType::LINKORI, 3, AtlasBodyNode::pelvis);
    torso_ori_task_ =
        new BasicTask(robot_, BasicTaskType::LINKORI, 3, AtlasBodyNode::utorso);

    foot_pos_task_ =
        new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, swing_foot);
    foot_ori_task_ =
        new BasicTask(robot_, BasicTaskType::LINKORI, 3, swing_foot);
    total_joint_task_ =
        new BasicTask(robot_, BasicTaskType::JOINT, Atlas::n_adof);

    std::vector<bool> act_list;
    act_list.resize(Atlas::n_dof, true);
    for (int i(0); i < Atlas::n_vdof; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(Atlas::n_dof, 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] =
        0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Atlas::n_adof, -2500.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Atlas::n_adof, 2500.);

    kin_wbc_contact_list_.clear();
    int rf_idx_offset(0);
    if (swing_foot == AtlasBodyNode::l_sole) {
        rf_idx_offset = rfoot_contact_->getDim();
        for (int i(0); i < lfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + rf_idx_offset] = 5.0;
            wblc_data_->W_xddot_[i + rf_idx_offset] = 0.001;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + rf_idx_offset] = 0.5;

        ((SurfaceContactSpec*)lfoot_contact_)->setMaxFz(0.0001);
        kin_wbc_contact_list_.push_back(rfoot_contact_);
    } else if (swing_foot == AtlasBodyNode::r_sole) {
        for (int i(0); i < rfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + rf_idx_offset] = 5.0;
            wblc_data_->W_xddot_[i + rf_idx_offset] = 0.0001;
        }
        wblc_data_->W_rf_[rfoot_contact_->getFzIndex() + rf_idx_offset] = 0.5;

        ((SurfaceContactSpec*)rfoot_contact_)->setMaxFz(0.0001);
        kin_wbc_contact_list_.push_back(lfoot_contact_);
    } else
        printf("[Warnning] swing foot is not foot: %i\n", swing_foot);

    // Create Minimum jerk objects
    for (size_t i = 0; i < 3; i++) {
        min_jerk_offset_.push_back(new MinJerk_OneDimension());
    }
}

void BodyFootLearningCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma;

    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for (int i(0); i < Atlas::n_adof; ++i) {
        ((AtlasCommand*)_cmd)->jtrq[i] = gamma[i];
        ((AtlasCommand*)_cmd)->q[i] = des_jpos_[i];
        ((AtlasCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void BodyFootLearningCtrl::_contact_setup() {
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();
    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BodyFootLearningCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Atlas::n_vdof, Atlas::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Atlas::n_adof));

    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);

    // myUtils::pretty_print(wblc_data_->Fr_, std::cout, "Fr");
    // myUtils::pretty_print(des_jpos_, std::cout, "des_jpos");
    // myUtils::pretty_print(des_jvel_, std::cout, "des_jvel");
    // myUtils::pretty_print(des_jacc_, std::cout, "des_jacc");
    // myUtils::pretty_print(des_jacc_cmd, std::cout, "des_jacc");
    // myUtils::pretty_print(gamma, std::cout, "gamma");
}

void BodyFootLearningCtrl::_task_setup() {
    double t(myUtils::smooth_changing(0, 1, end_time_, state_machine_time_));
    double tdot(
        myUtils::smooth_changing_vel(0, 1, end_time_, state_machine_time_));

    // =========================================================================
    // Body Pos Task
    // =========================================================================
    double body_height_cmd;
    if (b_set_height_target_)
        body_height_cmd = target_body_height_;
    else
        body_height_cmd = ini_body_pos_[2];

    Eigen::VectorXd vel_des(3);
    vel_des.setZero();
    Eigen::VectorXd acc_des(3);
    acc_des.setZero();
    Eigen::VectorXd des_pos = ini_body_pos_;
    des_pos[2] = body_height_cmd;
    body_pos_task_->updateTask(des_pos, vel_des, acc_des);

    // =========================================================================
    // Body Ori Task
    // =========================================================================
    Eigen::Quaternion<double> curr_body_delta_quat =
        dart::math::expToQuat(body_delta_so3_ * t);
    Eigen::Quaternion<double> curr_body_quat_des =
        curr_body_delta_quat * ini_body_quat_;
    Eigen::Vector3d curr_body_so3_des = body_delta_so3_ * tdot;

    Eigen::VectorXd des_body_quat = Eigen::VectorXd::Zero(4);
    des_body_quat << curr_body_quat_des.w(), curr_body_quat_des.x(),
        curr_body_quat_des.y(), curr_body_quat_des.z();
    Eigen::VectorXd des_body_so3 = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) des_body_so3[i] = curr_body_so3_des[i];

    Eigen::VectorXd ang_acc_des = Eigen::VectorXd::Zero(3);

    body_ori_task_->updateTask(des_body_quat, des_body_so3, ang_acc_des);

    // =========================================================================
    // Torso Ori Task
    // =========================================================================
    Eigen::Quaternion<double> curr_torso_delta_quat =
        dart::math::expToQuat(torso_delta_so3_ * t);
    Eigen::Quaternion<double> curr_torso_quat_des =
        curr_torso_delta_quat * ini_torso_quat_;
    Eigen::Vector3d curr_torso_so3_des = torso_delta_so3_ * tdot;

    Eigen::VectorXd des_torso_quat = Eigen::VectorXd::Zero(4);
    des_torso_quat << curr_torso_quat_des.w(), curr_torso_quat_des.x(),
        curr_torso_quat_des.y(), curr_torso_quat_des.z();
    Eigen::VectorXd des_torso_so3 = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) des_torso_so3[i] = curr_torso_so3_des[i];

    torso_ori_task_->updateTask(des_torso_quat, des_torso_so3, ang_acc_des);

    // =========================================================================
    // Foot Ori Task
    // =========================================================================
    Eigen::Quaternion<double> curr_foot_delta_quat =
        dart::math::expToQuat(foot_delta_so3_ * t);
    Eigen::Quaternion<double> curr_foot_quat_des =
        curr_foot_delta_quat * ini_foot_quat_;
    Eigen::Vector3d curr_foot_so3_des = foot_delta_so3_ * tdot;

    Eigen::VectorXd des_foot_quat = Eigen::VectorXd::Zero(4);
    des_foot_quat << curr_foot_quat_des.w(), curr_foot_quat_des.x(),
        curr_foot_quat_des.y(), curr_foot_quat_des.z();
    Eigen::VectorXd des_foot_so3 = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) des_foot_so3[i] = curr_foot_so3_des[i];

    foot_ori_task_->updateTask(des_foot_quat, des_foot_so3, ang_acc_des);

    // =========================================================================
    // Foot Pos Task
    // =========================================================================
    _CheckPlanning();
    _foot_pos_task_setup();

    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = sp_->jpos_ini;
    Eigen::VectorXd zero(Atlas::n_adof);
    zero.setZero();
    total_joint_task_->updateTask(jpos_des, zero, zero);

    // =========================================================================
    // Task List Update
    // =========================================================================
    task_list_.push_back(foot_pos_task_);
    task_list_.push_back(body_pos_task_);
    task_list_.push_back(body_ori_task_);
    task_list_.push_back(torso_ori_task_);
    task_list_.push_back(foot_ori_task_);
    task_list_.push_back(total_joint_task_);

    // =========================================================================
    // Solve Inv Kinematics
    // =========================================================================
    kin_wbc_->FindConfiguration(sp_->q, task_list_, kin_wbc_contact_list_,
                                des_jpos_, des_jvel_, des_jacc_);
}

void BodyFootLearningCtrl::_foot_pos_task_setup() {
    //_GetSinusoidalSwingTrajectory();
    _GetBsplineSwingTrajectory();

    double traj_time = state_machine_time_ - half_swing_time_;
    if (state_machine_time_ > half_swing_time_) {
        double pos, vel, acc;
        for (int i(0); i < 3; ++i) {
            min_jerk_offset_[i]->getPos(traj_time, pos);
            min_jerk_offset_[i]->getVel(traj_time, vel);
            min_jerk_offset_[i]->getAcc(traj_time, acc);

            curr_foot_pos_des_[i] += pos;
            curr_foot_vel_des_[i] += vel;
            curr_foot_acc_des_[i] += acc;
        }
    }
    if (state_machine_time_ > end_time_) {
        for (int i(0); i < foot_pos_task_->getDim(); ++i) {
            curr_foot_vel_des_[i] = 0;
            curr_foot_acc_des_[i] = 0;
        }
        curr_foot_pos_des_[2] =
            -push_down_height_ - 0.1 * (state_machine_time_ - end_time_);
    }

    Eigen::VectorXd foot_pos_des(foot_pos_task_->getDim());
    for (int i = 0; i < 3; ++i) foot_pos_des[i] = curr_foot_pos_des_[i];
    Eigen::VectorXd foot_vel_des(foot_pos_task_->getDim());
    foot_vel_des.setZero();
    Eigen::VectorXd foot_acc_des(foot_pos_task_->getDim());
    foot_acc_des.setZero();
    foot_vel_des = curr_foot_vel_des_;
    foot_acc_des = curr_foot_acc_des_;

    foot_pos_task_->updateTask(curr_foot_pos_des_, foot_vel_des, foot_acc_des);
}

void BodyFootLearningCtrl::_CheckPlanning() {
    if ((state_machine_time_ > 0.5 * end_time_) && b_replanning_ &&
        !b_replaned_) {
        Eigen::Vector3d target_loc;
        _Replanning(target_loc);

        Eigen::Vector3d target_offset;
        // X, Y target is originally set by intial_traget_loc
        for (int i(0); i < 2; ++i)
            target_offset[i] = target_loc[i] - initial_target_loc_[i];

        // Foot height (z) is set by the initial height
        target_offset[2] = 0.;  // target_loc[2] - ini_foot_pos_[2];

        _SetMinJerkOffset(target_offset);
        b_replaned_ = true;
    }
}

void BodyFootLearningCtrl::_Replanning(Eigen::Vector3d& target_loc) {
    bool b_nan(false);
    // =========================================================================
    // 0. Check Nan
    // =========================================================================
    if (dart::math::isNan(robot_->getQ())) {
        myUtils::color_print(myColor::BoldMagneta,
                             "[[Nan Detected, Send Previous Data]]");
        RLInterface::GetRLInterface()->GetRLData()->b_data_filled = true;
        RLInterface::GetRLInterface()->GetRLData()->done = true;
        RLInterface::GetRLInterface()->GetRLData()->reward -= alive_reward_;
        RLInterface::GetRLInterface()->SendData();
        b_nan = true;
    }

    // =========================================================================
    // 1. count
    // =========================================================================
    RLInterface::GetRLInterface()->GetRLData()->count = sp_->rl_count;
    ++(sp_->rl_count);

    // =========================================================================
    // 2. observation
    // =========================================================================
    Eigen::Vector3d com_pos, com_vel;
    // Direct value used
    com_pos = robot_->getCoMPosition();
    com_vel = robot_->getCoMVelocity();

    printf("planning com state: %f, %f, %f, %f\n", com_pos[0], com_pos[1],
           com_vel[0], com_vel[1]);

    double des_yaw =
        dart::math::matrixToEulerZYX(sp_->des_quat.toRotationMatrix())[0];
    Eigen::MatrixXd obs(1, nn_policy_->GetNumInput());
    Eigen::VectorXd obs_vec(nn_policy_->GetNumInput());
    obs_vec << com_pos[0], com_pos[1], target_body_height_ - sp_->q[2],
        sp_->q[5], sp_->q[4], sp_->q[3], des_yaw - sp_->q[3], sp_->qdot[0],
        sp_->qdot[1], sp_->qdot[2];
    obs_vec = myUtils::GetRelativeVector(obs_vec, terminate_obs_lower_bound_,
                                         terminate_obs_upper_bound_);
    for (int i = 0; i < obs_vec.size(); ++i) obs(0, i) = obs_vec(i);
    RLInterface::GetRLInterface()->GetRLData()->observation = obs_vec;
    // =========================================================================
    // 3. nn outputs : actions, action_mean, neglogp, value
    // =========================================================================
    Eigen::MatrixXd output, mean, mean_cropped;
    Eigen::VectorXd neglogp;
    nn_policy_->GetOutput(obs, action_lower_bound_, action_upper_bound_, output,
                          mean, neglogp);
    mean_cropped = myUtils::CropMatrix(mean, action_lower_bound_mat_,
                                       action_upper_bound_mat_, "atlas bfl");
    int n_output(output.cols());
    Eigen::VectorXd output_vec = Eigen::VectorXd::Zero(n_output);
    Eigen::VectorXd mean_vec = Eigen::VectorXd::Zero(n_output);
    float neglogp_val(0);
    for (int i = 0; i < n_output; ++i) {
        // !! Stochastic !! //
        output_vec(i) = output(0, i);
        // !! Deterministic !! //
        // output_vec(i) = mean_cropped(0, i);
        mean_vec(i) = mean(0, i);
    }
    neglogp_val = neglogp(0);
    RLInterface::GetRLInterface()->GetRLData()->action = output_vec;
    RLInterface::GetRLInterface()->GetRLData()->action_mean = mean_vec;
    RLInterface::GetRLInterface()->GetRLData()->neglogp = neglogp_val;
    RLInterface::GetRLInterface()->GetRLData()->value =
        (nn_valfn_->GetOutput(obs))(0, 0);
    // =========================================================================
    // 4. done
    // =========================================================================
    bool done;
    if (myUtils::isInBoundingBox(obs_vec,
                                 -1.0 * Eigen::VectorXd::Ones(obs_vec.size()),
                                 1.0 * Eigen::VectorXd::Ones(obs_vec.size()))) {
        done = false;
    } else {
        done = true;
    }
    RLInterface::GetRLInterface()->GetRLData()->done = done;
    // =========================================================================
    // 5. reward
    // =========================================================================
    float reward(0.0);
    double input_pen = quad_input_penalty_ * output_vec.squaredNorm();
    double yaw_dev_pen =
        deviation_penalty_ * (des_yaw - sp_->q[3]) * (des_yaw - sp_->q[3]) /
        (terminate_obs_upper_bound_[5] * terminate_obs_upper_bound_[5]);
    Eigen::Vector2d act_location;
    act_location << sp_->q[0] + sp_->global_pos_local[0],
        sp_->q[1] + sp_->global_pos_local[1];
    double loc_pen =
        deviation_penalty_ * (sp_->des_location - act_location).squaredNorm();

    reward = -input_pen - yaw_dev_pen - loc_pen;
    if (!done) {
        reward += alive_reward_;
    }

    std::cout << "// ===============================" << std::endl;
    std::cout << "// Reward info " << std::endl;
    std::cout << "// ===============================" << std::endl;
    std::cout << "total rew : " << reward << "|| input pen : " << input_pen
              << ", yaw_dev_pen : " << yaw_dev_pen << ", loc_pen : " << loc_pen
              << std::endl;

    std::cout << "// ===============================" << std::endl;
    std::cout << "// Observation info " << std::endl;
    std::cout << "// ===============================" << std::endl;
    myUtils::pretty_print(obs_vec, std::cout, "observation");

    RLInterface::GetRLInterface()->GetRLData()->reward = reward_scale_ * reward;
    RLInterface::GetRLInterface()->GetRLData()->b_data_filled = true;

    int num_ignore(4);
    if (sp_->num_step_copy < num_ignore) {
        RLInterface::GetRLInterface()->GetRLData()->b_data_filled = false;
        sp_->rl_count = 0;
        myUtils::color_print(myColor::BoldMagneta, "[[Skip First Step Data]]");
    } else {
        myUtils::color_print(myColor::BoldMagneta, "[[Send Data]]");
        RLInterface::GetRLInterface()->SendData();
    }

    // =========================================================================
    // Foot Step Guider
    // =========================================================================
    TVROutput pl_output;
    TVRParameter pl_param;
    pl_param.swing_time = end_time_ - state_machine_time_ +
                          transition_time_ * transition_phase_ratio_ +
                          stance_time_ * double_stance_ratio_;

    pl_param.des_loc = sp_->des_location;
    pl_param.stance_foot_loc = sp_->global_pos_local;
    pl_param.yaw_angle = sp_->q[AtlasDoF::baseRotZ];

    if (swing_foot_ == AtlasBodyNode::l_sole)
        pl_param.b_positive_sidestep = true;
    else
        pl_param.b_positive_sidestep = false;

    Eigen::Vector3d tmp_global_pos_local = sp_->global_pos_local;

    planner_->getNextFootLocation(com_pos + tmp_global_pos_local, com_vel,
                                  target_loc, &pl_param, &pl_output);
    // Time Modification
    replan_moment_ = state_machine_time_;
    end_time_ += pl_output.time_modification;
    target_loc -= sp_->global_pos_local;

    target_loc[2] = initial_target_loc_[2];

    for (int i(0); i < 2; ++i) {
        target_loc[i] += foot_landing_offset_[i];
    }
    // =========================================================================
    // Step adjustment
    // =========================================================================
    myUtils::pretty_print(target_loc, std::cout, "guided next foot location");
    // myUtils::pretty_print(target_loc2, std::cout,
    //"guided next foot location without body offset");
    sp_->guided_foot = target_loc + sp_->global_pos_local;
    if (sp_->num_step_copy < num_ignore) {
    } else {
        for (int i = 0; i < 2; ++i) {
            target_loc[i] += action_scale_[i] * output_vec[i];
        }
    }
    sp_->adjusted_foot = target_loc + sp_->global_pos_local;
    myUtils::pretty_print(target_loc, std::cout, "adjusted next foot location");
}

void BodyFootLearningCtrl::firstVisit() {
    b_replaned_ = false;

    ini_body_pos_ =
        robot_->getBodyNodeIsometry(AtlasBodyNode::pelvis).translation();

    ini_body_quat_ = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(AtlasBodyNode::pelvis).linear());
    body_delta_quat_ = sp_->des_quat * ini_body_quat_.inverse();
    body_delta_so3_ = dart::math::quatToExp(body_delta_quat_);

    ini_torso_quat_ = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(AtlasBodyNode::utorso).linear());
    torso_delta_quat_ = sp_->des_quat * ini_torso_quat_.inverse();
    torso_delta_so3_ = dart::math::quatToExp(torso_delta_quat_);

    ini_foot_quat_ = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(swing_foot_).linear());
    foot_delta_quat_ = sp_->des_quat * ini_foot_quat_.inverse();
    foot_delta_so3_ = dart::math::quatToExp(foot_delta_quat_);

    ini_foot_pos_ = robot_->getBodyNodeIsometry(swing_foot_).translation();

    ctrl_start_time_ = sp_->curr_time;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;

    initial_target_loc_[0] = sp_->q[0];
    initial_target_loc_[1] = sp_->q[1] + default_target_loc_[1];
    initial_target_loc_[2] = -push_down_height_;

    _SetBspline(ini_foot_pos_, initial_target_loc_);

    Eigen::Vector3d foot_pos_offset;
    foot_pos_offset.setZero();
    foot_pos_offset[2] = 0.;
    _SetMinJerkOffset(foot_pos_offset);
}

void BodyFootLearningCtrl::_SetMinJerkOffset(const Eigen::Vector3d& offset) {
    // Initialize Minimum Jerk Parameter Containers
    Eigen::Vector3d init_params;
    Eigen::Vector3d final_params;

    // Set Minimum Jerk Boundary Conditions
    for (size_t i = 0; i < 3; i++) {
        // Set Dimension i's initial pos, vel and acceleration
        init_params.setZero();
        // Set Dimension i's final pos, vel, acceleration
        final_params.setZero();
        final_params[0] = offset[i];

        min_jerk_offset_[i]->setParams(init_params, final_params, 0.,
                                       half_swing_time_);
    }
}

bool BodyFootLearningCtrl::endOfPhase() {
    // if (state_machine_time_ > (end_time_ + waiting_time_limit_)) {
    if (state_machine_time_ > end_time_) {
        printf("(state_machine time, end time) : (%f, %f) \n",
               state_machine_time_, end_time_);
        return true;
    }
    // Swing foot contact = END
    if (b_contact_switch_check_) {
        bool contact_happen(false);
        if (swing_foot_ == AtlasBodyNode::l_sole && sp_->b_lfoot_contact) {
            contact_happen = true;
        }
        if (swing_foot_ == AtlasBodyNode::r_sole && sp_->b_rfoot_contact) {
            contact_happen = true;
        }
        if (state_machine_time_ > end_time_ * 0.5 && contact_happen) {
            printf("(state_machine time, end time) : (%f, %f) \n",
                   state_machine_time_, end_time_);
            return true;
        }
    }
    return false;
}

void BodyFootLearningCtrl::ctrlInitialization(const YAML::Node& node) {
    ini_base_height_ = sp_->q[AtlasDoF::basePosZ];
    std::vector<double> tmp_vec;

    // Setting Parameters
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);
        myUtils::readParameter(node, "swing_height", swing_height_);
        myUtils::readParameter(node, "push_down_height", push_down_height_);
        myUtils::readParameter(node, "default_target_foot_location",
                               default_target_loc_);
        myUtils::readParameter(node, "body_pt_offset", body_pt_offset_);
        myUtils::readParameter(node, "foot_landing_offset",
                               foot_landing_offset_);

        myUtils::readParameter(node, "quad_input_penalty", quad_input_penalty_);
        myUtils::readParameter(node, "alive_reward", alive_reward_);
        myUtils::readParameter(node, "deviation_penalty", deviation_penalty_);
        myUtils::readParameter(node, "reward_scale", reward_scale_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
    static bool b_bodypute_eigenvalue(true);
    if (b_bodypute_eigenvalue) {
        ((TVRPlanner*)planner_)
            ->CheckEigenValues(double_stance_ratio_ * stance_time_ +
                               transition_phase_ratio_ * transition_time_ +
                               end_time_);
        b_bodypute_eigenvalue = false;
    }
    // printf("[Body Foot JPos Planning Ctrl] Parameter Setup Completed\n");
}

BodyFootLearningCtrl::~BodyFootLearningCtrl() {
    delete wblc_;
    delete lfoot_contact_;
    delete rfoot_contact_;
}

void BodyFootLearningCtrl::_GetBsplineSwingTrajectory() {
    double pos[3];
    double vel[3];
    double acc[3];

    foot_traj_.getCurvePoint(state_machine_time_, pos);
    foot_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
    foot_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

    for (int i(0); i < 3; ++i) {
        curr_foot_pos_des_[i] = pos[i];
        curr_foot_vel_des_[i] = vel[i];
        curr_foot_acc_des_[i] = acc[i];
    }
}
void BodyFootLearningCtrl::_GetSinusoidalSwingTrajectory() {
    curr_foot_acc_des_.setZero();
    for (int i(0); i < 2; ++i) {
        curr_foot_pos_des_[i] =
            myUtils::smooth_changing(ini_foot_pos_[i], initial_target_loc_[i],
                                     end_time_, state_machine_time_);
        curr_foot_vel_des_[i] = myUtils::smooth_changing_vel(
            ini_foot_pos_[i], initial_target_loc_[i], end_time_,
            state_machine_time_);
        curr_foot_acc_des_[i] = myUtils::smooth_changing_acc(
            ini_foot_pos_[i], initial_target_loc_[i], end_time_,
            state_machine_time_);
    }
    // for Z (height)
    double amp(swing_height_ / 2.);
    double omega(2. * M_PI / end_time_);

    curr_foot_pos_des_[2] =
        ini_foot_pos_[2] + amp * (1 - cos(omega * state_machine_time_));
    curr_foot_vel_des_[2] = amp * omega * sin(omega * state_machine_time_);
    curr_foot_acc_des_[2] =
        amp * omega * omega * cos(omega * state_machine_time_);
}

void BodyFootLearningCtrl::_SetBspline(const Eigen::Vector3d& st_pos,
                                       const Eigen::Vector3d& des_pos) {
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];
    Eigen::Vector3d middle_pos;

    middle_pos = (st_pos + des_pos) / 2.;
    middle_pos[2] = swing_height_;

    // Initial and final position & velocity & acceleration
    for (int i(0); i < 3; ++i) {
        // Initial
        init[i] = st_pos[i];
        init[i + 3] = 0.;
        init[i + 6] = 0.;
        // Final
        fin[i] = des_pos[i];
        fin[i + 3] = 0.;
        fin[i + 6] = 0.;
        // Middle
        middle_pt[0][i] = middle_pos[i];
    }
    // TEST
    fin[5] = -0.5;
    fin[8] = 5.;
    foot_traj_.SetParam(init, fin, middle_pt, end_time_);

    delete[] * middle_pt;
    delete[] middle_pt;
}

