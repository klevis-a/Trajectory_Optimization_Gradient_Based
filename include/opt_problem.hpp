#ifndef OPT_PROBLEM_HPP
#define OPT_PROBLEM_HPP
#ifndef SPARSE_GRADIENT
#define SPARSE_GRADIENT false
#endif
#include <iostream>

// Eigen library for vectors and matrices:
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>

#include <vector>

using namespace std;
namespace opt_problems
{
  class optProblem
  {
  public:
    // constructor
    optProblem(int dummy);// dummy since pagmo requires default values for variables
    optProblem(manipulator_kdl::robotKDL robot_kdl, vector<vector<double>> data_arr,vector<double> vel_limits, vector<double> timeBetween);

    // functions
    double objFunction(const vector<double> &x) const;
    vector<double> inEqConstraints(const vector<double> &x) const;
    vector<double> EqConstraints(const vector<double> &x) const;
    vector<double> gradient(const vector<double> &x) const;
    vector<double> objGradient(const vector<double> &x) const;
    vector<double> inEqGradient(const vector<double> &x) const;
    vector<double> eqGradient(const vector<double> &x) const;
    Eigen::VectorXd positionCost(const vector<double> &x, int i) const;
      Eigen::VectorXd positionCost(const vector<double> &x, int i, vector<double> &joint_pos) const;
    
    vector<vector<double>> bounds() const;
    #if SPARSE_GRADIENT==true
    vector<double> sparse_gradient(vector<double> x) const;
    std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> gradient_sparsity() const;
    #endif
    int m_dim,m_nec,m_nic;
    vector<Eigen::VectorXd> des_poses;
  private:
    manipulator_kdl::robotKDL robot_kdl_;
    // Collision checking: TODO
    //mutable collisionChecker c_checker;
    int timesteps_;
    vector<double> vel_lim;
    vector<double> timeBetweenS;
  };
}
#endif
