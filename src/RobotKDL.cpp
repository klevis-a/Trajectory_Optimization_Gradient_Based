//
// Created by klevis on 1/30/18.
//

#include "RobotKDL.h"

using std::string;
using Eigen::MatrixXd;
using Eigen::VectorXd;

//here empty constructor is needed so problem can be initialized empty - see mocap_traj_problem.cpp
RobotKDL::RobotKDL()
{
}

RobotKDL::RobotKDL(const string &file_name, const string &base, const string &end_effector)
{
    if(!kdl_parser::treeFromFile(file_name, robot_tree_))
    {
        throw std::runtime_error("Could not parse KDL file.");
    }

    if(!robot_tree_.getChain(base,end_effector, kdl_chain))
    {
        throw std::runtime_error("Could not construct KDL chain.");
    }

    fk_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain);
    jacobian_ = new KDL::Jacobian(kdl_chain.getNrOfJoints());
    jacobian_solver = new KDL::ChainJntToJacSolver(kdl_chain);

    urdf::Model model;
    if (!model.initFile(file_name))
    {
        throw std::runtime_error("Could not parse KDL file.");
    }

    dof=0;
    for(std::map<string, boost::shared_ptr<urdf::Joint>>::iterator it = model.joints_.begin(); it != model.joints_.end(); ++it)
    {
        if (model.joints_[it->first]->type != urdf::Joint::UNKNOWN && model.joints_[it->first]->type != urdf::Joint::FIXED)
        {
            low_bounds.emplace_back(model.joints_[it->first]->limits->lower);
            up_bounds.emplace_back(model.joints_[it->first]->limits->upper);
            dof+=1;
        }
    }
}

MatrixXd RobotKDL::getFKFrame(const VectorXd &jointPos) const
{
    KDL::JntArray q(jointPos.size());
    q.data=jointPos;
    KDL::Frame p_out;
    fk_solver->JntToCart(q,p_out);

    MatrixXd frame_(4,4);
    frame_.setIdentity();

    // translation
    frame_(0,3)=p_out.p[0];
    frame_(1,3)=p_out.p[1];
    frame_(2,3)=p_out.p[2];

    //rotation
    for(int i=0;i<3;++i)
    {
        for(int j=0;j<3;j++)
        {
            frame_(i,j)=p_out.M(i,j);
        }
    }

    return frame_;
}

MatrixXd RobotKDL::getFKFrame(const std::vector<double> &jointPos) const
{
    const VectorXd jointPosEigen = Eigen::Map<const VectorXd>(jointPos.data(),jointPos.size());
    return getFKFrame(jointPosEigen);
}

Eigen::Matrix<double, 6, Eigen::Dynamic> RobotKDL::getJacobian(const std::vector<double> &jointPos) const
{
    const VectorXd jointPosEigen = Eigen::Map<const VectorXd>(jointPos.data(),jointPos.size());
    return getJacobian(jointPosEigen);
};

Eigen::Matrix<double, 6, Eigen::Dynamic> RobotKDL::getJacobian(const VectorXd &jointPos) const
{
    KDL::JntArray q(jointPos.size());
    q.data=jointPos;
    jacobian_solver->JntToJac(q,*jacobian_);
    return jacobian_->data;
};
