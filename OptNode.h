//
// Created by han on 2020/12/3.
//

#ifndef CASSIE_MPC_OPTNODE_H
#define CASSIE_MPC_OPTNODE_H

#include <drake/solvers/solve.h>

class OptNode {
public:
    OptNode(std::string name, int var_size, std::shared_ptr<drake::solvers::MathematicalProgram> program);
    OptNode(OptNode &) = delete;
    OptNode(OptNode &&) = delete;

    std::shared_ptr<drake::solvers::MathematicalProgram> program_;
    drake::solvers::VectorXDecisionVariable decision_var_ptr_;
    std::string var_name_;
    int var_size_;

};


#endif //CASSIE_MPC_OPTNODE_H
