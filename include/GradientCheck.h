#ifndef M20IA_TRAJ_OPT_GRADIENTCHECK_H
#define M20IA_TRAJ_OPT_GRADIENTCHECK_H

#include <vector>
#include <random>

#include "Robot_mocap_opt_problem.h"

class GradientCheck
{
public:
    GradientCheck(opt_problems::Robot_mocap_opt_problem &p, double tolerance);
    bool checkGradientObj(std::vector<double> x0);
    bool checkGradientRandom(unsigned int numTrials=10);
    std::vector<double> createRandomX();
private:
    opt_problems::Robot_mocap_opt_problem &problem;
    std::mt19937 eng;
    std::vector<std::uniform_real_distribution<double>> dof_distr;
    double _tolerance;
    int _total_dof;
};

#endif //M20IA_TRAJ_OPT_GRADIENTCHECK_H
