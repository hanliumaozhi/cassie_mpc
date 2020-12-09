//
// Created by han on 2020/12/3.
//

#include "OptNode.h"

OptNode::OptNode(std::string name, int var_size, std::shared_ptr<drake::solvers::MathematicalProgram> program)
{
    var_name_ = name;
    var_size_ = var_size;
    program_ = program;
    decision_var_ptr_ = program_->NewContinuousVariables(var_size_, var_name_);
}

void OptNode::build()
{
    Eigen::VectorXd one(1);
    one << 1;
    Eigen::VectorXd ones(4);
    ones << 1, 1, 1, 1;
    // 1. zmp constraints left front -> right back
    zmp_constraints_.push_back(
            program_->AddLinearConstraint(one.transpose(), 0,
                                          1, decision_var_ptr_.segment(15, 1)).evaluator().get());
    zmp_constraints_.push_back(
            program_->AddLinearConstraint(one.transpose(), 0,
                                          1, decision_var_ptr_.segment(16, 1)).evaluator().get());
    zmp_constraints_.push_back(
            program_->AddLinearConstraint(one.transpose(), 0,
                                          1, decision_var_ptr_.segment(17, 1)).evaluator().get());
    zmp_constraints_.push_back(
            program_->AddLinearConstraint(one.transpose(), 0,
                                          1, decision_var_ptr_.segment(18, 1)).evaluator().get());
    zmp_constraints_.push_back(
            program_->AddLinearConstraint(ones.transpose(), 0,
                                          1, decision_var_ptr_.segment(15, 4)).evaluator().get());
    zmp_constraints_.push_back(
            program_->AddLinearConstraint(one.transpose(), 0,
                                          1, decision_var_ptr_.segment(19, 1)).evaluator().get());
    zmp_constraints_.push_back(
            program_->AddLinearConstraint(one.transpose(), 0,
                                          1, decision_var_ptr_.segment(20, 1)).evaluator().get());
    zmp_constraints_.push_back(
            program_->AddLinearConstraint(one.transpose(), 0,
                                          1, decision_var_ptr_.segment(21, 1)).evaluator().get());
    zmp_constraints_.push_back(
            program_->AddLinearConstraint(one.transpose(), 0,
                                          1, decision_var_ptr_.segment(22, 1)).evaluator().get());
    zmp_constraints_.push_back(
            program_->AddLinearConstraint(ones.transpose(), 0,
                                          1, decision_var_ptr_.segment(19, 4)).evaluator().get());

    // foot motion constraints: part 1

    Eigen::Matrix<drake::symbolic::Expression, 3, 3> rot_matrix;

    rot_matrix(0, 0) = drake::symbolic::cos(-decision_var_ptr_(6));
    rot_matrix(0, 1) = -drake::symbolic::sin(-decision_var_ptr_(6));
    rot_matrix(1, 0) = drake::symbolic::sin(-decision_var_ptr_(6));
    rot_matrix(1, 1) = drake::symbolic::cos(-decision_var_ptr_(6));

    //x 10-30 y -40-40
    Eigen::Matrix<drake::symbolic::Expression, 3, 1> com_position;
    com_position(0) = decision_var_ptr_(0);
    com_position(1) = decision_var_ptr_(1);
    com_position(2) = decision_var_ptr_(2);

    // left
    Eigen::Matrix<drake::symbolic::Expression, 3, 1> left_foot;
    left_foot(0) = decision_var_ptr_(8);
    left_foot(1) = decision_var_ptr_(9);
    auto left_com = rot_matrix*(left_foot-com_position);

    foot_motion_constraints_.push_back(
            program_->AddConstraint(left_com(0) >= 5).evaluator().get());
    foot_motion_constraints_.push_back(
            program_->AddConstraint(left_com(0) <= 40).evaluator().get());

    foot_motion_constraints_.push_back(
            program_->AddConstraint(left_com(1) >= -40).evaluator().get());
    foot_motion_constraints_.push_back(
            program_->AddConstraint(left_com(1) <= 40).evaluator().get());

    //right
    Eigen::Matrix<drake::symbolic::Expression, 3, 1> right_foot;
    right_foot(0) = decision_var_ptr_(11);
    right_foot(1) = decision_var_ptr_(12);
    auto right_com = rot_matrix*(right_foot-com_position);

    foot_motion_constraints_.push_back(
            program_->AddConstraint(right_com(0) >= -40).evaluator().get());
    foot_motion_constraints_.push_back(
            program_->AddConstraint(right_com(0) <= -5).evaluator().get());

    foot_motion_constraints_.push_back(
            program_->AddConstraint(right_com(1) >= -40).evaluator().get());
    foot_motion_constraints_.push_back(
            program_->AddConstraint(right_com(1) <= 40).evaluator().get());

    //com z constraint

    com_z_constraints_.push_back(program_->AddLinearConstraint(
            decision_var_ptr_(2) >= 0.8).evaluator().get()
            );

    com_z_constraints_.push_back(program_->AddLinearConstraint(
            decision_var_ptr_(2) <= 0.9).evaluator().get()
    );



}