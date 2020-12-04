//
// Created by han on 2020/12/4.
//

#ifndef CASSIE_MPC_MPC_H
#define CASSIE_MPC_MPC_H

#include <memory>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/solve.h>

class MPC {
public:
    MPC(int var_num_per_node, int node_num, double spring_stiffness, double total_mass, double double_support_duration,
    double single_support_duration);
    MPC(MPC& ) = delete;
    MPC(MPC&& ) = delete;

    void build();

private:
    std::unique_ptr<Eigen::VectorXd> var_sol_;
    int var_num_per_node_;
    int node_num_;
    double spring_stiffness_;
    double total_mass_;
    double double_support_duration_;
    double single_support_duration_;

    std::shared_ptr<drake::solvers::MathematicalProgram> program_;
};


#endif //CASSIE_MPC_MPC_H