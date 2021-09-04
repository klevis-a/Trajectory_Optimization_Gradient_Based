//
// Created by klevis on 1/30/18.
//

#ifndef M20IA_TRAJ_OPT_ROBOTKDL_H
#define M20IA_TRAJ_OPT_ROBOTKDL_H

#include <string>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <urdf/model.h>

class RobotKDL {
public:
    RobotKDL();
    RobotKDL(const std::string &file_name, const std::string &base, const std::string &end_effector);
    Eigen::MatrixXd getFKFrame(const Eigen::VectorXd &jointPos) const;
    Eigen::MatrixXd getFKFrame(const std::vector<double> &jointPos) const;
    Eigen::Matrix<double, 6, Eigen::Dynamic> getJacobian(const std::vector<double> &jointPos) const;
    Eigen::Matrix<double, 6, Eigen::Dynamic> getJacobian(const Eigen::VectorXd &jointPos) const;
    std::vector<double> low_bounds;
    std::vector<double> up_bounds;
    unsigned int dof;
private:
    KDL::Tree robot_tree_;
    KDL::ChainFkSolverPos* fk_solver;
    KDL::ChainJntToJacSolver* jacobian_solver;
    KDL::Chain kdl_chain;
    KDL::Jacobian* jacobian_;
};


#endif //M20IA_TRAJ_OPT_ROBOTKDL_H
