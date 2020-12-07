//
// Created by han on 2020/12/4.
//

#include "MPC.h"

MPC::MPC(int var_num_per_node, int node_num, double spring_stiffness, double total_mass, double double_support_duration,
    double single_support_duration):
    var_num_per_node_(var_num_per_node),
    node_num_(node_num),
    spring_stiffness_(spring_stiffness),
    total_mass_(total_mass),
    double_support_duration_(double_support_duration),
    single_support_duration_(single_support_duration){
    program_ = std::make_shared<drake::solvers::MathematicalProgram>();
    var_sol_ = std::make_unique<Eigen::VectorXd>(var_num_per_node_*node_num_);

}

void MPC::build() {
    for (int i = 0; i < node_num_; ++i) {
        node_list_.emplace_back(std::make_unique<OptNode>(std::to_string(i), var_num_per_node_, program_));
    }
}
