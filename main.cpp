#include <iostream>
#include <drake/solvers/choose_best_solver.h>
#include "MPC.h"
#include <chrono>

int main() {
    MPC mpc(25, 4, 10000, 34, 0.01, 0.35);

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

    test_data.push_back(0.1);
    test_data.push_back(0);
    test_data.push_back(0);

    mpc.build();


    //std::cout<<drake::solvers::ChooseBestSolver(*mpc.program_).name()<<std::endl;
    //mpc.program_->SetSolverOption(drake::solvers::SnoptSolver::id(), "Scale option",
    //                              0);

    auto ss = drake::solvers::SnoptSolver();

    mpc.program_->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                                  "../mpc_snopt.out");
    //mpc.program_->SetSolverOption(drake::solvers::SnoptSolver::id(), "Scale option",
    //                              1);
    //auto ii = drake::solvers::IpoptSolver();


    auto t1 = std::chrono::high_resolution_clock::now();
    mpc.update(0.01, 0, test_data);
    const drake::solvers::MathematicalProgramResult result = ss.Solve(*mpc.program_);
    auto t2 = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

    std::cout << duration<<std::endl;

    std::cout<<result.is_success()<<std::endl;
    mpc.print_var(result);
    return 0;
}
