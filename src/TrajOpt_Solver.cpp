//
// Created by klevis on 2/14/18.
//

#include <pagmo_plugins_nonfree/snopt7.hpp>

#include "TrajOpt_Solver.h"
#include "CsvWriter.h"

using std::vector;
using std::string;

TrajOpt_Solver::TrajOpt_Solver(const ConfigParser &configParser, const string &dataFile, const string &seedsFile, const Eigen::MatrixXd &toolframe, bool calcInitPos) {
    // initialize kdl class
    RobotKDL kdl(configParser.urdfFile(),configParser.baseName(),configParser.eeName());
    _dof=kdl.dof;

    //read the data files specified in the XML config
    DataReader dataReader(dataFile, seedsFile, kdl, toolframe);
    _timesteps=dataReader.timesteps();

    //create the optimization problem
    Robot_mocap_opt_problem optProblem(kdl, dataReader.trajectoryData(), dataReader.dt(), Eigen::Map<const Eigen::VectorXd>(configParser.velLimits().data(), configParser.velLimits().size()), 
                                        configParser.posLB(), configParser.posUB(), configParser.rotLB(), configParser.rotUB());
    pagmo::problem prob{pagmo::Pagmo_traj_optimization{optProblem}};

    //initial guess
    vector<double> x_in = dataReader.initialJoints();
    vector<double> init_transform = dataReader.initPosGuess();
    if(calcInitPos) {
        x_in.insert(x_in.end(), init_transform.begin(), init_transform.end());
    }
    else
    {
        x_in.insert(x_in.end(), configParser.initPosition().begin(), configParser.initPosition().end());
    }

    //set the tolerances
    vector<double> tol(optProblem.m_nec+optProblem.m_nic, configParser.tolerance());
    prob.set_c_tol(tol);

    //set the solving algorithm
    pagmo::snopt7 sn_v(false, configParser.snoptcLib());
    sn_v.set_verbosity(100);
    sn_v.set_numeric_option("Major feasibility tolerance", configParser.tolerance());
    sn_v.set_integer_option("Major iterations limit", configParser.majorIterLimit());
    sn_v.set_integer_option("Iterations limit", configParser.iterLimit());

    /*
    nlopt nlopt{configParser.algorithm()};
    nlopt.set_verbosity(10);
    nlopt.set_ftol_rel(configParser.stopValue());
    nlopt.set_stopval(configParser.stopValue());
    */

    _algo=pagmo::algorithm{sn_v};
    _algo.set_verbosity(100);
    _pop=pagmo::population{prob};

    //send the initial guess to the optimization problem
    vector<double> f_in=prob.fitness(x_in);
    _pop.push_back(x_in,f_in);
}

TrajOpt_Solver::TrajOpt_Solver(const ConfigParser &configParser, const string &dataFile, const string &seedsFile, const Eigen::MatrixXd &toolframe) :
    TrajOpt_Solver(configParser, dataFile, seedsFile, toolframe, configParser.calcInitPosition())
{}

void TrajOpt_Solver::solve() {
    //solve the problem
    _pop_result = _algo.evolve(_pop);
    std::cout<<"Final cost:"<<_pop_result.champion_f()[0]<<std::endl;
}

void TrajOpt_Solver::printResults(const string &jointsFile, const string &offsetFile) const {
    //store resulting joint angles
    vector<double> x_best=_pop_result.champion_x();
    vector<vector<double>> solved_joint_angles;

    for(size_t i=0; i<_timesteps; i++)
    {
        vector<double> current_joint_angles(_dof,0.0);
        for(size_t j=0; j<_dof; j++)
        {
            current_joint_angles[j]=x_best[i*_dof+j];
        }
        solved_joint_angles.push_back(current_joint_angles);
    }


    //store the resulting starting offset positions
    vector<vector<double>> offsetPosVec;
    double zrot = x_best[x_best.size()-4];
    double x_off = x_best[x_best.size()-3];
    double y_off = x_best[x_best.size()-2];
    double z_off = x_best[x_best.size()-1];
    vector<double> offsetPos{zrot, x_off, y_off, z_off};
    offsetPosVec.push_back(offsetPos);

    //write to CSV
    CsvWriter jointsWriter(solved_joint_angles);
    CsvWriter offsetWriter(offsetPosVec);

    jointsWriter.writeToFile(jointsFile);
    offsetWriter.writeToFile(offsetFile);
}
