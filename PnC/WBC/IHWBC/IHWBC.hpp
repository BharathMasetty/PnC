#include <PnC/WBC/Task.hpp>
#include <PnC/WBC/ContactSpec.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>

// Implicit Hierarchical Whole Body Controller
class IHWBC{
public:
	IHWBC(const std::vector<bool> & act_list);
    virtual ~IHWBC();

    void updateSetting(const Eigen::MatrixXd & A,
                    const Eigen::MatrixXd & Ainv,
                    const Eigen::VectorXd & cori,
                    const Eigen::VectorXd & grav,
                    void* extra_setting = NULL);

    void solve(const std::vector<Task*> & task_list,
               const std::vector<ContactSpec*> & contact_list,
               const Eigen::VectorXd & Fd,                
               Eigen::VectorXd & tau_cmd, Eigen::VectorXd & qddot_cmd);

    // w_task_heirarchy_in:
    //  - sets relative weight between task priority
    //  - must have dimension equal to the number of tasks.
    // w_rf_contacts_in (for target wrench minimization only)
    //  - sets relative weight to distribute the foces between contacts 
    //  - must have dimension equal to the number of contacts
    // w_contact_weight_in  
    //  - sets the relative weight of the contact forces and the task hierarchies
    void setQPWeights(const Eigen::VectorXd & w_task_heirarchy_in, const Eigen::VectorXd & w_rf_contacts_in, const double & w_contact_weight_in);
    void setQPWeights(const Eigen::VectorXd & w_task_heirarchy_in, const double & w_contact_weight_in);
    void setRegularizationTerms(const double lambda_qddot_in, const double lambda_Fr_in); 

    // If true, we try to minimize for a target wrench value. Fd \in mathbf{R}^6.
    // If false, we try to minimize the desired contact forces term by term: Fd \in mathbf{R}^(n). n = dim of reaction force
    void setTargetWrenchMinimization(const bool target_wrench_minimization_in); 
    bool target_wrench_minimization;

    // Weight of task hierarchies
    Eigen::VectorXd w_task_heirarchy;
    // Weights of contact reaction forces
    Eigen::VectorXd w_rf_contacts;
    double w_contact_weight;

    // Tikhonov regularization
    double lambda_qddot; 
    double lambda_Fr; 

private:
    int num_qdot_; // Number of degrees of freedom
    int num_act_joint_; // Actuated Joints
    int num_passive_; // Passive Joints
    int dim_contacts_;

    // Selection Matrices
    Eigen::MatrixXd Sa_; // Actuated joint
    Eigen::MatrixXd Sv_; // Virtual joint
    Eigen::MatrixXd Sf_; // Floating base 

    Eigen::MatrixXd A_;
    Eigen::MatrixXd Ainv_;
    Eigen::VectorXd cori_;
    Eigen::VectorXd grav_;	

    // Contact Jacobians
    Eigen::MatrixXd Jc_;
    Eigen::MatrixXd Jc_weighted_;

    bool b_weights_set_;
    bool b_updatesetting_;

    // Quadprog sizes
    int n_quadprog_ = 1; // Number of Decision Variables
    int p_quadprog_ = 0; // Number of Inequality Constraints
    int m_quadprog_ = 0; // Number of Equality Constraints

    // Quadprog Variables
    GolDIdnani::GVect<double> x;
    // Cost
    GolDIdnani::GMatr<double> G;
    GolDIdnani::GVect<double> g0;

    // Equality
    GolDIdnani::GMatr<double> CE;
    GolDIdnani::GVect<double> ce0;

    // Inequality
    GolDIdnani::GMatr<double> CI;
    GolDIdnani::GVect<double> ci0;


    // Quadprog Result Containers
    Eigen::VectorXd qp_dec_vars_;
    Eigen::VectorXd qddot_result_;
    Eigen::VectorXd Fr_result_;

    void buildContactStacks(const std::vector<ContactSpec*> & contact_list, const Eigen::VectorXd & w_rf_contacts_in);

    void prepareQPSizes();
    void setQuadProgCosts(const Eigen::MatrixXd & P_cost, const Eigen::VectorXd & v_cost);
    void setEqualityConstraints(const Eigen::MatrixXd & Eq_mat, const Eigen::VectorXd & Eq_vec);

    void solveQP();

};

