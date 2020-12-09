//
// Created by han on 2020/12/4.
//

#ifndef CASSIE_MPC_MPC_H
#define CASSIE_MPC_MPC_H

#include <memory>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/solve.h>

#include "OptNode.h"

class MPC {
public:
    MPC(int var_num_per_node, int node_num, double spring_stiffness, double total_mass, double double_support_duration,
    double single_support_duration);
    MPC(MPC& ) = delete;
    MPC(MPC&& ) = delete;

    void build();

    void update(double rest_time, int current_state, std::vector<double>& data);

    void print_var(const drake::solvers::MathematicalProgramResult& result);

    std::unique_ptr<Eigen::VectorXd> var_sol_;
    std::shared_ptr<drake::solvers::MathematicalProgram> program_;

private:
    int var_num_per_node_;
    int node_num_;
    double spring_stiffness_;
    double total_mass_;
    double double_support_duration_;
    double single_support_duration_;

    double toe_position_{0.08};
    double heel_position_{-0.08};

    double omega_;

    std::vector<std::unique_ptr<OptNode>> node_list_;

    std::vector<drake::solvers::LinearConstraint*> initial_constraints_;
    std::vector<drake::solvers::LinearConstraint*> final_constraints_;

    drake::solvers::VectorXDecisionVariable duration_var_ptr_;
    std::vector<drake::solvers::LinearConstraint*> duration_constraints_;

    std::vector<drake::solvers::Constraint*> dynamics_constraints_;
    std::vector<drake::solvers::LinearConstraint*> foot_constraints_;

    std::vector<double> time_vec_;
};


#endif //CASSIE_MPC_MPC_H
