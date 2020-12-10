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
    var_sol_ = std::make_unique<Eigen::VectorXd>(var_size);
    var_sol_->setZero();
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
            program_->AddLinearConstraint(ones.transpose(), 1,
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
            program_->AddLinearConstraint(ones.transpose(), 1,
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
            program_->AddConstraint(left_com(0) , -0.02, 0.4).evaluator().get());
    //foot_motion_constraints_.push_back(
    //        program_->AddConstraint(left_com(0) <= 0.4).evaluator().get());

    foot_motion_constraints_.push_back(
            program_->AddConstraint(left_com(1), -0.4, 0.4).evaluator().get());
    //foot_motion_constraints_.push_back(
    //        program_->AddConstraint(left_com(1) <= 0.4).evaluator().get());

    //right
    Eigen::Matrix<drake::symbolic::Expression, 3, 1> right_foot;
    right_foot(0) = decision_var_ptr_(11);
    right_foot(1) = decision_var_ptr_(12);
    auto right_com = rot_matrix*(right_foot-com_position);

    foot_motion_constraints_.push_back(
            program_->AddConstraint(right_com(0) , -0.4, 0.02).evaluator().get());
    //foot_motion_constraints_.push_back(
    //        program_->AddConstraint(right_com(0) <= 0.02).evaluator().get());

    foot_motion_constraints_.push_back(
            program_->AddConstraint(right_com(1) , -0.4, 0.4).evaluator().get());
    //foot_motion_constraints_.push_back(
    //        program_->AddConstraint(right_com(1) <= 0.4).evaluator().get());

    //com z constraint

    com_z_constraints_.push_back(program_->AddLinearConstraint(
            decision_var_ptr_(2) , 0.83, 0.9).evaluator().get()
            );

    //com_z_constraints_.push_back(program_->AddLinearConstraint(
    //        decision_var_ptr_(2) <= 0.92).evaluator().get()
    //);

    com_z_constraints_.push_back(program_->AddLinearConstraint(
            decision_var_ptr_(23) == 0.868).evaluator().get()
    );

    //com_z_constraints_.push_back(program_->AddLinearConstraint(
    //        decision_var_ptr_(23) <= 0.92).evaluator().get()
    //);

    com_z_constraints_.push_back(program_->AddLinearConstraint(
            decision_var_ptr_(24) == 0.868).evaluator().get()
    );

    //com_z_constraints_.push_back(program_->AddLinearConstraint(
    //        decision_var_ptr_(24) <= 0.92).evaluator().get()
    //);

    geo_constraints_.push_back(program_->AddLinearConstraint(
            decision_var_ptr_(10), -0.7, 0.7
            ).evaluator().get());

    geo_constraints_.push_back(program_->AddLinearConstraint(
            decision_var_ptr_(13), -0.7, 0.7
            ).evaluator().get());
}

void OptNode::print_var(const drake::solvers::MathematicalProgramResult& result)
{
    *var_sol_ = result.GetSolution(decision_var_ptr_);
    std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
    std::cout<<var_name_<<std::endl;
    std::cout<<"com: "<<(*var_sol_)(0)<<" "<<(*var_sol_)(1)<<" "<<(*var_sol_)(2)<<std::endl;
    std::cout<<"com vel: "<<(*var_sol_)(3)<<" "<<(*var_sol_)(4)<<" "<<(*var_sol_)(5)<<std::endl;
    std::cout<<"head and vel: "<<(*var_sol_)(6)<<" "<<(*var_sol_)(7)<<std::endl;
    std::cout<<"left foot x y yaw: "<<(*var_sol_)(8)<<" "<<(*var_sol_)(9)<<" "<<(*var_sol_)(10)<<std::endl;
    std::cout<<"right foot x y yaw: "<<(*var_sol_)(11)<<" "<<(*var_sol_)(12)<<" "<<(*var_sol_)(13)<<std::endl;
    std::cout<<"head ddot: "<<(*var_sol_)(14)<<std::endl;
    std::cout<<"lambda begin: "<<(*var_sol_)(15)<<" "<<(*var_sol_)(16)<<" "<<(*var_sol_)(17)<<" "<<(*var_sol_)(18)<<std::endl;
    std::cout<<"lambda end: "<<(*var_sol_)(19)<<" "<<(*var_sol_)(20)<<" "<<(*var_sol_)(21)<<" "<<(*var_sol_)(22)<<std::endl;
    std::cout<<"r: "<<(*var_sol_)(23)<<" "<<(*var_sol_)(24)<<std::endl;
}