#ifndef M20IA_TRAJ_OPT_ROBOT_MOCAP_OPT_PROBLEM_H
#define M20IA_TRAJ_OPT_ROBOT_MOCAP_OPT_PROBLEM_H

#ifndef SPARSE_GRADIENT
#define SPARSE_GRADIENT true
#endif

#include <vector>
#include <eigen3/Eigen/Dense>
#include "RobotKDL.h"

class Robot_mocap_opt_problem {
public:
    // constructor
    Robot_mocap_opt_problem(int dummy);// pagmo requires dummy constructor
    Robot_mocap_opt_problem(const RobotKDL robot_kdl, const std::vector<Eigen::MatrixXd> frames, const Eigen::VectorXd dt, const Eigen::VectorXd vel_limits,
    const std::vector<double> &posLB, const std::vector<double> &posUB, const double rotLB, const double rotUB);

    // functions
    double objFunction(const std::vector<double> &x) const;
    std::vector<double> inEqConstraints(const std::vector<double> &x) const;
    std::vector<double> EqConstraints(const std::vector<double> &x) const;
    std::vector<double> gradient(const std::vector<double> &x) const;
    std::vector<double> objGradient(const std::vector<double> &x) const;
    std::vector<double> inEqGradient(const std::vector<double> &x) const;
    std::vector<double> eqGradient(const std::vector<double> &x) const;
    std::vector<std::vector<double>> bounds() const;

    #if SPARSE_GRADIENT == true
    std::vector<double> sparse_gradient(const std::vector<double> &x) const;
    std::vector<std::pair<std::vector<double>::size_type, std::vector<double>::size_type>> gradient_sparsity() const;
    #endif
    int m_dim, m_nec, m_nic;
private:
    RobotKDL _robot_kdl;
    std::vector<Eigen::MatrixXd> _frames;
    Eigen::VectorXd _dt;
    Eigen::VectorXd _vel_limits;
    unsigned int _timesteps;
    std::vector<double> _posLB;
    std::vector<double> _posUB;
    double _rotLB;
    double _rotUB;
};

#endif //M20IA_TRAJ_OPT_ROBOT_MOCAP_OPT_PROBLEM_H
