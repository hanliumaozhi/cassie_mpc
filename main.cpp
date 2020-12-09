#include <iostream>
#include <drake/solvers/choose_best_solver.h>
#include "MPC.h"

int main() {
    MPC mpc(25, 10, 7000, 34, 0.02, 0.35);

    std::vector<double> test_data;
    test_data.push_back(-0.0114496);
    test_data.push_back(0.000170439);
    test_data.push_back(0.868318);
    test_data.push_back(0);
    test_data.push_back(0);
    test_data.push_back(0);

    test_data.push_back(0);
    test_data.push_back(0);

    test_data.push_back(0);
    test_data.push_back(0.2);
    test_data.push_back(0);

    test_data.push_back(0);
    test_data.push_back(-0.2);
    test_data.push_back(0);

    test_data.push_back(0.48);
    test_data.push_back(0);
    test_data.push_back(0);

    mpc.build();

    mpc.update(0.02, 0, test_data);


    //std::cout<<drake::solvers::ChooseBestSolver(*mpc.program_).name()<<std::endl;

    mpc.program_->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                           "../mpc_snopt.out");
    //mpc.program_->SetSolverOption(drake::solvers::SnoptSolver::id(), "Scale option",
    //                              0);

    auto ss = drake::solvers::SnoptSolver();
    const drake::solvers::MathematicalProgramResult result = ss.Solve(*mpc.program_);

    std::cout<<result.is_success()<<std::endl;
    return 0;
}
