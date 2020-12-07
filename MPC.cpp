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

    //initial state constraint
    // 1: x y z
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(0),
                                          0, 0).evaluator().get());
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(1),
                                          0, 0).evaluator().get());
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(2),
                                          0, 0).evaluator().get());
    // 2: dotx doty dotz
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(3),
                                          0, 0).evaluator().get());
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(4),
                                          0, 0).evaluator().get());
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(5),
                                          0, 0).evaluator().get());
    // 3: theta dottheta
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(6),
                                          0, 0).evaluator().get());
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(7),
                                          0, 0).evaluator().get());

    // 4 left foot
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(8),
                                          0, 0).evaluator().get());
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(9),
                                          0, 0).evaluator().get());
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(10),
                                          0, 0).evaluator().get());

    // 5 right foot
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(11),
                                          0, 0).evaluator().get());
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(12),
                                          0, 0).evaluator().get());
    initial_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[0]->decision_var_ptr_(13),
                                          0, 0).evaluator().get());

    // final state check z
    final_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[node_num_-1]->decision_var_ptr_(0),
                                          0, 0).evaluator().get());
    final_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[node_num_-1]->decision_var_ptr_(1),
                                          0, 0).evaluator().get());
    final_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[node_num_-1]->decision_var_ptr_(6),
                                          0, 0).evaluator().get());
}
