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
    var_sol_ = std::make_unique<Eigen::VectorXd>(node_num_);

    duration_var_ptr_ = program_->NewContinuousVariables((node_num_), "duration_var");

    omega_ = std::sqrt(spring_stiffness_/total_mass_);

    time_vec_.push_back(double_support_duration_);
    time_vec_.push_back(single_support_duration_);

    end_var_ptr_ = program_->NewContinuousVariables(3, "end_var");


}

void MPC::build() {
    for (int i = 0; i < node_num_; ++i) {
        node_list_.emplace_back(std::make_unique<OptNode>(std::to_string(i), var_num_per_node_, program_));
        node_list_[i]->build();
    }

    //0: time constraint
    for (int i = 0; i < node_num_; ++i) {
        duration_constraints_.push_back(
                program_->AddLinearConstraint(duration_var_ptr_[i],
                                              0, 0).evaluator().get());
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

    //final state check z
    /*final_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[node_num_-1]->decision_var_ptr_(0),
                                          0, 0).evaluator().get());
    final_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[node_num_-1]->decision_var_ptr_(1),
                                          0, 0).evaluator().get());
    final_constraints_.push_back(
            program_->AddLinearConstraint(node_list_[node_num_-1]->decision_var_ptr_(6),
                                          0, 0).evaluator().get());*/

    // dynamics constraint
    for (int i = 0; i < (node_num_-1); ++i) {
        //1. x
        //std::cout<<9.81/node_list_[i]->decision_var_ptr_(2)<<std::endl;
        drake::symbolic::Expression alpha = drake::symbolic::sqrt(9.81/node_list_[i]->decision_var_ptr_(2));

        // begin with left
        Eigen::Matrix<drake::symbolic::Expression, 2, 1> left_toe;
        Eigen::Matrix<drake::symbolic::Expression, 2, 1> left_heel;

        left_toe(0) =  node_list_[i]->decision_var_ptr_(8) + toe_position_*drake::symbolic::cos(
                node_list_[i]->decision_var_ptr_(10));

        left_toe(1) =  node_list_[i]->decision_var_ptr_(8) + toe_position_*drake::symbolic::sin(
                node_list_[i]->decision_var_ptr_(10));

        left_heel(0) =  node_list_[i]->decision_var_ptr_(9) + heel_position_*drake::symbolic::cos(
                node_list_[i]->decision_var_ptr_(10));

        left_heel(1) =  node_list_[i]->decision_var_ptr_(9) + heel_position_*drake::symbolic::sin(
                node_list_[i]->decision_var_ptr_(10));

        Eigen::Matrix<drake::symbolic::Expression, 2, 1> right_toe;
        Eigen::Matrix<drake::symbolic::Expression, 2, 1> right_heel;

        right_toe(0) = node_list_[i]->decision_var_ptr_(11) + toe_position_*drake::symbolic::cos(
                node_list_[i]->decision_var_ptr_(13));

        right_toe(1) = node_list_[i]->decision_var_ptr_(11) + toe_position_*drake::symbolic::sin(
                node_list_[i]->decision_var_ptr_(13));

        right_heel(0) = node_list_[i]->decision_var_ptr_(12) + heel_position_*drake::symbolic::cos(
                node_list_[i]->decision_var_ptr_(13));

        right_heel(1) = node_list_[i]->decision_var_ptr_(12) + heel_position_*drake::symbolic::sin(
                node_list_[i]->decision_var_ptr_(13));

        drake::symbolic::Expression zmp_begin_x = node_list_[i]->decision_var_ptr_(15)*left_toe(0)+
                node_list_[i]->decision_var_ptr_(16)*left_heel(0)+
                node_list_[i]->decision_var_ptr_(17)*right_toe(0)+
                node_list_[i]->decision_var_ptr_(18)*right_heel(0);

        drake::symbolic::Expression zmp_end_x = node_list_[i]->decision_var_ptr_(19)*left_toe(0)+
                                                  node_list_[i]->decision_var_ptr_(20)*left_heel(0)+
                                                  node_list_[i]->decision_var_ptr_(21)*right_toe(0)+
                                                  node_list_[i]->decision_var_ptr_(22)*right_heel(0);

        drake::symbolic::Expression beta_1_x = (node_list_[i]->decision_var_ptr_(0)-zmp_begin_x)/2+
                (node_list_[i]->decision_var_ptr_(3)*duration_var_ptr_(i)-(zmp_end_x-zmp_begin_x))/(2*alpha*duration_var_ptr_(i));

        drake::symbolic::Expression beta_2_x = (node_list_[i]->decision_var_ptr_(0)-zmp_begin_x)/2-
                (node_list_[i]->decision_var_ptr_(3)*duration_var_ptr_(i)-(zmp_end_x-zmp_begin_x))/(2*alpha*duration_var_ptr_(i));

        // x
        dynamics_constraints_.push_back(
                program_->AddConstraint(node_list_[i+1]->decision_var_ptr_(0) -
                (beta_1_x*drake::symbolic::exp(alpha*duration_var_ptr_(i))+
                beta_2_x*drake::symbolic::exp(-alpha*duration_var_ptr_(i))+zmp_end_x) == 0).evaluator().get());


        drake::symbolic::Expression zmp_begin_y = node_list_[i]->decision_var_ptr_(15)*left_toe(1)+
                                                  node_list_[i]->decision_var_ptr_(16)*left_heel(1)+
                                                  node_list_[i]->decision_var_ptr_(17)*right_toe(1)+
                                                  node_list_[i]->decision_var_ptr_(18)*right_heel(1);

        drake::symbolic::Expression zmp_end_y = node_list_[i]->decision_var_ptr_(19)*left_toe(1)+
                                                node_list_[i]->decision_var_ptr_(20)*left_heel(1)+
                                                node_list_[i]->decision_var_ptr_(21)*right_toe(1)+
                                                node_list_[i]->decision_var_ptr_(22)*right_heel(1);

        drake::symbolic::Expression beta_1_y = (node_list_[i]->decision_var_ptr_(1)-zmp_begin_y)/2+
                (node_list_[i]->decision_var_ptr_(4)*duration_var_ptr_(i)-(zmp_end_y-zmp_begin_y))/(2*alpha*duration_var_ptr_(i));

        drake::symbolic::Expression beta_2_y = (node_list_[i]->decision_var_ptr_(1)-zmp_begin_y)/2-
                (node_list_[i]->decision_var_ptr_(4)*duration_var_ptr_(i)-(zmp_end_y-zmp_begin_y))/(2*alpha*duration_var_ptr_(i));

        // y
        dynamics_constraints_.push_back(
                program_->AddConstraint(node_list_[i+1]->decision_var_ptr_(1) -
                (beta_1_y*drake::symbolic::exp(alpha*duration_var_ptr_(i))+
                beta_2_y*drake::symbolic::exp(-alpha*duration_var_ptr_(i))+zmp_end_y) == 0).evaluator().get());

        drake::symbolic::Expression d1 = node_list_[i]->decision_var_ptr_(2) -
                node_list_[i]->decision_var_ptr_(23) + 9.81/(omega_*omega_);

        drake::symbolic::Expression d2 = node_list_[i]->decision_var_ptr_(5)/(omega_) -
                (node_list_[i]->decision_var_ptr_(24)-node_list_[i]->decision_var_ptr_(23))/(duration_var_ptr_(i)*omega_);

        // z
        dynamics_constraints_.push_back(
                program_->AddConstraint(node_list_[i+1]->decision_var_ptr_(2) -
                (d1*drake::symbolic::cos(omega_*duration_var_ptr_(i)) +
                d2*drake::symbolic::sin(omega_*duration_var_ptr_(i))+node_list_[i]->decision_var_ptr_(24)
                -9.81/(omega_*omega_)) == 0).evaluator().get());

        // theta
        dynamics_constraints_.push_back(
                program_->AddConstraint(node_list_[i+1]->decision_var_ptr_(6) -
                (node_list_[i]->decision_var_ptr_(6)+node_list_[i]->decision_var_ptr_(7)*duration_var_ptr_(i)
                +0.5*node_list_[i]->decision_var_ptr_(14)*duration_var_ptr_(i)) == 0).evaluator().get());

        foot_constraints_.push_back(
                program_->AddLinearConstraint(node_list_[i]->decision_var_ptr_(8)-
                                                node_list_[i+1]->decision_var_ptr_(8),
                                              0, 0).evaluator().get());
        foot_constraints_.push_back(
                program_->AddLinearConstraint(node_list_[i]->decision_var_ptr_(9)-
                                              node_list_[i+1]->decision_var_ptr_(9),
                                              0, 0).evaluator().get());
        foot_constraints_.push_back(
                program_->AddLinearConstraint(node_list_[i]->decision_var_ptr_(10)-
                                              node_list_[i+1]->decision_var_ptr_(10),
                                              0, 0).evaluator().get());
        foot_constraints_.push_back(
                program_->AddLinearConstraint(node_list_[i]->decision_var_ptr_(11)-
                                              node_list_[i+1]->decision_var_ptr_(11),
                                              0, 0).evaluator().get());
        foot_constraints_.push_back(
                program_->AddLinearConstraint(node_list_[i]->decision_var_ptr_(12)-
                                              node_list_[i+1]->decision_var_ptr_(12),
                                              0, 0).evaluator().get());
        foot_constraints_.push_back(
                program_->AddLinearConstraint(node_list_[i]->decision_var_ptr_(13)-
                                              node_list_[i+1]->decision_var_ptr_(13),
                                              0, 0).evaluator().get());
    }

    // final state check z

    //1. x
    //std::cout<<9.81/node_list_[i]->decision_var_ptr_(2)<<std::endl;
    drake::symbolic::Expression alpha = drake::symbolic::sqrt(9.81/node_list_[node_num_-1]->decision_var_ptr_(2));

    // begin with left
    Eigen::Matrix<drake::symbolic::Expression, 2, 1> left_toe;
    Eigen::Matrix<drake::symbolic::Expression, 2, 1> left_heel;

    left_toe(0) =  node_list_[node_num_-1]->decision_var_ptr_(8) + toe_position_*drake::symbolic::cos(
            node_list_[node_num_-1]->decision_var_ptr_(10));

    left_toe(1) =  node_list_[node_num_-1]->decision_var_ptr_(8) + toe_position_*drake::symbolic::sin(
            node_list_[node_num_-1]->decision_var_ptr_(10));

    left_heel(0) =  node_list_[node_num_-1]->decision_var_ptr_(9) + heel_position_*drake::symbolic::cos(
            node_list_[node_num_-1]->decision_var_ptr_(10));

    left_heel(1) =  node_list_[node_num_-1]->decision_var_ptr_(9) + heel_position_*drake::symbolic::sin(
            node_list_[node_num_-1]->decision_var_ptr_(10));

    Eigen::Matrix<drake::symbolic::Expression, 2, 1> right_toe;
    Eigen::Matrix<drake::symbolic::Expression, 2, 1> right_heel;

    right_toe(0) = node_list_[node_num_-1]->decision_var_ptr_(11) + toe_position_*drake::symbolic::cos(
            node_list_[node_num_-1]->decision_var_ptr_(13));

    right_toe(1) = node_list_[node_num_-1]->decision_var_ptr_(11) + toe_position_*drake::symbolic::sin(
            node_list_[node_num_-1]->decision_var_ptr_(13));

    right_heel(0) = node_list_[node_num_-1]->decision_var_ptr_(12) + heel_position_*drake::symbolic::cos(
            node_list_[node_num_-1]->decision_var_ptr_(13));

    right_heel(1) = node_list_[node_num_-1]->decision_var_ptr_(12) + heel_position_*drake::symbolic::sin(
            node_list_[node_num_-1]->decision_var_ptr_(13));

    drake::symbolic::Expression zmp_begin_x = node_list_[node_num_-1]->decision_var_ptr_(15)*left_toe(0)+
                                              node_list_[node_num_-1]->decision_var_ptr_(16)*left_heel(0)+
                                              node_list_[node_num_-1]->decision_var_ptr_(17)*right_toe(0)+
                                              node_list_[node_num_-1]->decision_var_ptr_(18)*right_heel(0);

    drake::symbolic::Expression zmp_end_x = node_list_[node_num_-1]->decision_var_ptr_(19)*left_toe(0)+
                                            node_list_[node_num_-1]->decision_var_ptr_(20)*left_heel(0)+
                                            node_list_[node_num_-1]->decision_var_ptr_(21)*right_toe(0)+
                                            node_list_[node_num_-1]->decision_var_ptr_(22)*right_heel(0);

    drake::symbolic::Expression beta_1_x = (node_list_[node_num_-1]->decision_var_ptr_(0)-zmp_begin_x)/2+
                                           (node_list_[node_num_-1]->decision_var_ptr_(3)*duration_var_ptr_(node_num_-1)-(zmp_end_x-zmp_begin_x))/(2*alpha*duration_var_ptr_(node_num_-1));

    drake::symbolic::Expression beta_2_x = (node_list_[node_num_-1]->decision_var_ptr_(0)-zmp_begin_x)/2-
                                           (node_list_[node_num_-1]->decision_var_ptr_(3)*duration_var_ptr_(node_num_-1)-(zmp_end_x-zmp_begin_x))/(2*alpha*duration_var_ptr_(node_num_-1));

    //x
    final_constraints_.push_back(
            program_->AddConstraint(end_var_ptr_(0) -
            (beta_1_x*drake::symbolic::exp(alpha*duration_var_ptr_(node_num_-1))+
            beta_2_x*drake::symbolic::exp(-alpha*duration_var_ptr_(node_num_-1))+zmp_end_x) == 0
            ).evaluator().get());

    drake::symbolic::Expression zmp_begin_y = node_list_[node_num_-1]->decision_var_ptr_(15)*left_toe(1)+
                                              node_list_[node_num_-1]->decision_var_ptr_(16)*left_heel(1)+
                                              node_list_[node_num_-1]->decision_var_ptr_(17)*right_toe(1)+
                                              node_list_[node_num_-1]->decision_var_ptr_(18)*right_heel(1);

    drake::symbolic::Expression zmp_end_y = node_list_[node_num_-1]->decision_var_ptr_(19)*left_toe(1)+
                                            node_list_[node_num_-1]->decision_var_ptr_(20)*left_heel(1)+
                                            node_list_[node_num_-1]->decision_var_ptr_(21)*right_toe(1)+
                                            node_list_[node_num_-1]->decision_var_ptr_(22)*right_heel(1);

    drake::symbolic::Expression beta_1_y = (node_list_[node_num_-1]->decision_var_ptr_(1)-zmp_begin_y)/2+
                                           (node_list_[node_num_-1]->decision_var_ptr_(4)*duration_var_ptr_(node_num_-1)-(zmp_end_y-zmp_begin_y))/(2*alpha*duration_var_ptr_(node_num_-1));

    drake::symbolic::Expression beta_2_y = (node_list_[node_num_-1]->decision_var_ptr_(1)-zmp_begin_y)/2-
                                           (node_list_[node_num_-1]->decision_var_ptr_(4)*duration_var_ptr_(node_num_-1)-(zmp_end_y-zmp_begin_y))/(2*alpha*duration_var_ptr_(node_num_-1));
    //y
    final_constraints_.push_back(
            program_->AddConstraint(end_var_ptr_(1) -
            (beta_1_y*drake::symbolic::exp(alpha*duration_var_ptr_(node_num_-1))+
            beta_2_y*drake::symbolic::exp(-alpha*duration_var_ptr_(node_num_-1))+zmp_end_y) == 0
            ).evaluator().get());

    //theta
    final_constraints_.push_back(
            program_->AddConstraint(end_var_ptr_(2) - (node_list_[node_num_-1]->decision_var_ptr_(6)+
            node_list_[node_num_-1]->decision_var_ptr_(7)*duration_var_ptr_(node_num_-1)+
            0.5*node_list_[node_num_-1]->decision_var_ptr_(14)*duration_var_ptr_(node_num_-1)) == 0
            ).evaluator().get());

    end_constraints_.push_back(
            program_->AddLinearConstraint(end_var_ptr_(0), 0, 0).evaluator().get());
    end_constraints_.push_back(
            program_->AddLinearConstraint(end_var_ptr_(1), 0, 0).evaluator().get());
    end_constraints_.push_back(
            program_->AddLinearConstraint(end_var_ptr_(2), 0, 0).evaluator().get());
}

void MPC::update(double rest_time, int current_state, std::vector<double>& data)
{
    // 0 left double 1 left single 2 right double 3 right single

    // 1. update const constraint
    Eigen::VectorXd tmp_word(1);
    Eigen::VectorXd tmp_word2(1);
    for (int i = 0; i < 14; ++i) {
        tmp_word[0] = data[i];
        initial_constraints_[i]->UpdateLowerBound(tmp_word);
        initial_constraints_[i]->UpdateUpperBound(tmp_word);
    }

    for (int i = 0; i < 3; ++i) {
        tmp_word[0] = data[i+14];
        end_constraints_[i]->UpdateLowerBound(tmp_word);
        end_constraints_[i]->UpdateUpperBound(tmp_word);
    }

    // 2. update duration constraint
    tmp_word[0] = rest_time;
    duration_constraints_[0]->UpdateUpperBound(tmp_word);
    duration_constraints_[0]->UpdateLowerBound(tmp_word);
    int phase = 0;
    if (current_state == 1 || current_state == 3){
        phase = 1;
    }
    for (int i = 1; i < node_num_; ++i) {
        tmp_word[0] = time_vec_[(i+phase)%2];
        duration_constraints_[i]->UpdateUpperBound(tmp_word);
        duration_constraints_[i]->UpdateLowerBound(tmp_word);
    }

    tmp_word(0) = 0;
    tmp_word2(0) = 1;


    Eigen::VectorXd inf_vector(1);
    inf_vector << std::numeric_limits<double>::infinity();
    for (int i = 0; i < (node_num_-1); ++i) {
        int loop_state = (current_state+i) % 4;
        // update lambda
        if (loop_state == 0 || loop_state == 2){
            node_list_[i]->zmp_constraints_[0]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[0]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[1]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[1]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[2]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[2]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[3]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[3]->UpdateUpperBound(tmp_word2);

            node_list_[i]->zmp_constraints_[5]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[5]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[6]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[6]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[7]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[7]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[8]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[8]->UpdateUpperBound(tmp_word2);
        }

        if (loop_state == 1){
            node_list_[i]->zmp_constraints_[0]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[0]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[1]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[1]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[2]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[2]->UpdateUpperBound(tmp_word);
            node_list_[i]->zmp_constraints_[3]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[3]->UpdateUpperBound(tmp_word);

            node_list_[i]->zmp_constraints_[5]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[5]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[6]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[6]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[7]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[7]->UpdateUpperBound(tmp_word);
            node_list_[i]->zmp_constraints_[8]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[8]->UpdateUpperBound(tmp_word);
        }

        if ( loop_state == 3){
            node_list_[i]->zmp_constraints_[0]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[0]->UpdateUpperBound(tmp_word);
            node_list_[i]->zmp_constraints_[1]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[1]->UpdateUpperBound(tmp_word);
            node_list_[i]->zmp_constraints_[2]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[2]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[3]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[3]->UpdateUpperBound(tmp_word2);

            node_list_[i]->zmp_constraints_[5]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[5]->UpdateUpperBound(tmp_word);
            node_list_[i]->zmp_constraints_[6]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[6]->UpdateUpperBound(tmp_word);
            node_list_[i]->zmp_constraints_[7]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[7]->UpdateUpperBound(tmp_word2);
            node_list_[i]->zmp_constraints_[8]->UpdateLowerBound(tmp_word);
            node_list_[i]->zmp_constraints_[8]->UpdateUpperBound(tmp_word2);
        }

        // update foot position

        if (loop_state == 1 || loop_state == 3){
            foot_constraints_[i*6+0]->UpdateLowerBound(tmp_word);
            foot_constraints_[i*6+0]->UpdateUpperBound(tmp_word);
            foot_constraints_[i*6+1]->UpdateLowerBound(tmp_word);
            foot_constraints_[i*6+1]->UpdateUpperBound(tmp_word);
            foot_constraints_[i*6+2]->UpdateLowerBound(tmp_word);
            foot_constraints_[i*6+2]->UpdateUpperBound(tmp_word);
            foot_constraints_[i*6+3]->UpdateLowerBound(tmp_word);
            foot_constraints_[i*6+3]->UpdateUpperBound(tmp_word);
            foot_constraints_[i*6+4]->UpdateLowerBound(tmp_word);
            foot_constraints_[i*6+4]->UpdateUpperBound(tmp_word);
            foot_constraints_[i*6+5]->UpdateLowerBound(tmp_word);
            foot_constraints_[i*6+5]->UpdateUpperBound(tmp_word);
        }

        // left double
        if(loop_state == 0){
          foot_constraints_[i*6+0]->UpdateLowerBound(tmp_word);
          foot_constraints_[i*6+0]->UpdateUpperBound(tmp_word);
          foot_constraints_[i*6+1]->UpdateLowerBound(tmp_word);
          foot_constraints_[i*6+1]->UpdateUpperBound(tmp_word);
          foot_constraints_[i*6+2]->UpdateLowerBound(tmp_word);
          foot_constraints_[i*6+2]->UpdateUpperBound(tmp_word);

          foot_constraints_[i*6+3]->UpdateLowerBound(-inf_vector);
          foot_constraints_[i*6+3]->UpdateUpperBound(inf_vector);
          foot_constraints_[i*6+4]->UpdateLowerBound(-inf_vector);
          foot_constraints_[i*6+4]->UpdateUpperBound(inf_vector);
          foot_constraints_[i*6+5]->UpdateLowerBound(-inf_vector);
          foot_constraints_[i*6+5]->UpdateUpperBound(inf_vector);

        }

      if(loop_state == 2){
        foot_constraints_[i*6+0]->UpdateLowerBound(-inf_vector);
        foot_constraints_[i*6+0]->UpdateUpperBound(inf_vector);
        foot_constraints_[i*6+1]->UpdateLowerBound(-inf_vector);
        foot_constraints_[i*6+1]->UpdateUpperBound(inf_vector);
        foot_constraints_[i*6+2]->UpdateLowerBound(-inf_vector);
        foot_constraints_[i*6+2]->UpdateUpperBound(inf_vector);

        foot_constraints_[i*6+3]->UpdateLowerBound(tmp_word);
        foot_constraints_[i*6+3]->UpdateUpperBound(tmp_word);
        foot_constraints_[i*6+4]->UpdateLowerBound(tmp_word);
        foot_constraints_[i*6+4]->UpdateUpperBound(tmp_word);
        foot_constraints_[i*6+5]->UpdateLowerBound(tmp_word);
        foot_constraints_[i*6+5]->UpdateUpperBound(tmp_word);

      }
    }
}

void MPC::print_var(const drake::solvers::MathematicalProgramResult& result)
{
    for (int i = 0; i < node_num_; ++i) {
        node_list_[i]->print_var(result);
    }
    *var_sol_ = result.GetSolution(duration_var_ptr_);
    for (int i = 0; i < node_num_; ++i) {
        std::cout<<(*var_sol_)(i)<<" ";
    }
    std::cout<<std::endl;
}