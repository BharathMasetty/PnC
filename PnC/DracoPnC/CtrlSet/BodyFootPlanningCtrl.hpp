#pragma once

#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <Utils/minjerk_one_dim.hpp>
#include <Utils/BSplineBasic.h>

class KinWBC;

class BodyFootPlanningCtrl:public SwingPlanningCtrl{
   public:
        BodyFootPlanningCtrl(RobotSystem* robot,
                std::string swing_foot_, FootStepPlanner* planner);
        virtual ~BodyFootPlanningCtrl();
        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit(){ sp_->des_jpos_prev = des_jpos_; }
        virtual bool endOfPhase();

        virtual void ctrlInitialization(const std::string & setting_file_name);
    protected:
        double waiting_time_limit_;
        double ini_base_height_;
        int swing_leg_jidx_;
        double push_down_height_; // push foot below the ground at landing
        Eigen::Vector3d default_target_loc_;
        Eigen::Vector3d initial_target_loc_;

        int dim_contact_;
        ContactSpec* rfoot_contact_;
        ContactSpec* lfoot_contact_;

        void _CheckPlanning();
        void _Replanning(Eigen::Vector3d & target_loc);
        void _contact_setup();
        void _task_setup();
        void _compute_torque_wblc(Eigen::VectorXd & gamma);
        void _SetMinJerkOffset(const Eigen::Vector3d & offset);
        void _SetBspline(
            const Eigen::Vector3d & st_pos,
            const Eigen::Vector3d & des_pos);

        void _GetSinusoidalSwingTrajectory();
        void _GetBsplineSwingTrajectory();
        std::vector<ContactSpec*> kin_wbc_contact_list_;

        std::vector<int> selected_jidx_;
        Task* base_task_;
        Task* selected_joint_task_;
        Task* foot_task_;

        KinWBC* kin_wbc_;
        Eigen::VectorXd des_jpos_;
        Eigen::VectorXd des_jvel_;
        Eigen::VectorXd des_jacc_;

        Eigen::VectorXd Kp_;
        Eigen::VectorXd Kd_;

        Eigen::Vector3d ini_body_pos_;
        Eigen::Vector3d ini_com_pos_;
        Eigen::Vector3d ini_foot_pos_;
        Eigen::Vector2d body_pt_offset_;

        Eigen::VectorXd ini_config_;

        std::vector<double> foot_landing_offset_;

        std::vector<MinJerk_OneDimension*> min_jerk_offset_;
        BS_Basic<3, 3, 1, 2, 2> foot_traj_;
};