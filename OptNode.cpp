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
