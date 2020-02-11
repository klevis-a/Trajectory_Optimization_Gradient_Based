//
// Created by klevis on 12/7/17.
//

#ifndef M20IA_TRAJ_OPT_DATAREADER_H
#define M20IA_TRAJ_OPT_DATAREADER_H


#include "ConfigParser.h"
#include "RobotKDL.h"
#include <eigen3/Eigen/Dense>
#include <string>

class DataReader {
public:
    DataReader(const std::string &trajFile, const std::string &seedFile, const RobotKDL &kdl, const Eigen::MatrixXd &tf);
    const std::vector<double> &initialJoints() const;
    const std::vector<Eigen::MatrixXd> &trajectoryData() const;
    const Eigen::VectorXd &dt() const;
    const unsigned int timesteps() const;
    const std::vector<double> &initPosGuess() const;
private:
    std::vector<double> _initJoints;
    std::vector<Eigen::MatrixXd> _trajData;
    std::vector<Eigen::MatrixXd> _frameTransforms;
    Eigen::VectorXd _dt;
    unsigned int _numFrames;
    std::vector<double> _initPosGuess;

    void readInitialJoints();
    void readTrajectory();

    const std::string &_trajFile;
    const std::string &_seedFile;
    const RobotKDL &_kdl;
    const Eigen::MatrixXd &_tf;
};


#endif //M20IA_TRAJ_OPT_DATAREADER_H
