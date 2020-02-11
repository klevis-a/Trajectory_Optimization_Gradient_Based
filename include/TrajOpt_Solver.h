//
// Created by klevis on 2/14/18.
//

#ifndef M20IA_TRAJ_OPT_TRAJOPT_SOLVER_H
#define M20IA_TRAJ_OPT_TRAJOPT_SOLVER_H

#include <string>
#include <Pagmo_traj_optimization.hpp>
#include <DataReader.h>
#include "InputParser.h"


class TrajOpt_Solver {
public:
    TrajOpt_Solver(const ConfigParser &configParser, const std::string &dataFile, const std::string &seedsFile, const Eigen::MatrixXd &toolframe, bool calcInitPos);
    TrajOpt_Solver(const ConfigParser &configParser, const std::string &dataFile, const std::string &seedsFile, const Eigen::MatrixXd &toolframe);
    void solve();
    void printResults(const std::string &jointsFile, const std::string &offsetFile) const;
private:
    pagmo::algorithm _algo;
    pagmo::population _pop;
    pagmo::population _pop_result;
    unsigned int _dof;
    unsigned int _timesteps;
};


#endif //M20IA_TRAJ_OPT_TRAJOPT_SOLVER_H
