//
// Created by klevis on 12/7/17.
//

#include "DataReader.h"
#include "csv.h"
#include "GeometryUtils.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Map;

DataReader::DataReader(const std::string &trajectoryFile, const std::string &seedFile, const RobotKDL &kdl, const MatrixXd &tf):_trajFile(trajectoryFile), _seedFile(seedFile), _kdl(kdl), _tf(tf) {
    readTrajectory();
    readInitialJoints();
}

const vector<double> &DataReader::initialJoints() const {
    return _initJoints;
}

const vector<MatrixXd> &DataReader::trajectoryData() const {
    return _trajData;
}

const Eigen::VectorXd &DataReader::dt() const {
    return _dt;
}

void DataReader::readTrajectory() {
    MatrixXd tfInv = GeometryUtils::frameInverse(_tf);
    //read in the trajectory
    io::CSVReader<17> input(_trajFile);
    double tb,t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;

    size_t rowNum = 0;
    MatrixXd firstFrameInv;

    vector<double> dt;
    while(input.read_row(tb,t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33))
    {
        MatrixXd currentFrame(4,4);
        currentFrame << t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;
        _trajData.push_back(currentFrame*tfInv);
        dt.push_back(tb);
        if(rowNum>0)
        {
            MatrixXd currentTransform = firstFrameInv*currentFrame*tfInv;
            _frameTransforms.push_back(currentTransform);
        }
        else
        {
            firstFrameInv = GeometryUtils::frameInverse(currentFrame*tfInv);
        }
        rowNum++;
    }

    _dt = Map<VectorXd>(dt.data(),dt.size());
    _numFrames = rowNum;
}

void DataReader::readInitialJoints() {

    //read in the initial guess for the joint angles
    io::CSVReader<6> inputJ(_seedFile);
    double j1,j2,j3,j4,j5,j6;

    inputJ.read_row(j1,j2,j3,j4,j5,j6);
    vector<double> joints0 = {j1, j2, j3, j4, j5, j6};
    _initJoints = {j1, j2, j3, j4, j5, j6};

    //from here we have the first frame of the trajectory
    MatrixXd firstFrame = _kdl.getFKFrame(Map<VectorXd>(joints0.data(), joints0.size()));
    MatrixXd previousFrame = firstFrame;
    VectorXd currentTheta = Map<VectorXd>(joints0.data(), joints0.size());

    //but first let's create the initial guess for starting position and rotation about z
    _initPosGuess = vector<double>(4,0.0);
    //matrix difference between the two frames expressed in world coordinates
    Eigen::Matrix3d rotDiff = firstFrame.topLeftCorner<3,3>()*_trajData[0].topLeftCorner<3,3>().transpose();
    //define x-axis
    Vector3d xaxis;
    xaxis << 1,0,0;
    //rotate the x-axis using the rotation matrix
    Vector3d xaxisRot = rotDiff * xaxis;
    //project the rotated x-axis onto the xy plane
    xaxisRot[2]=0.0;
    //find the angle between the rotated x-axis and the initial x-axis;
    double rotAngle = acos(xaxisRot.dot(xaxis));
    //to find the sign of the angle simply look at the rotated vector
    double multiplier=1.0;

    if(xaxisRot[1]<0)
    {
        multiplier=-1;
    }

    rotAngle = rotAngle*multiplier;

    /* we could use euler angles but the problem is that Eigen minimizes the rotation about the first angle
     * in this case the z-rotation. But in our case this is undesirable because we want to maximize the
     * rotation about z R(z)*R(y)*R(x)
    Vector3d eulAngles = rotDiff.eulerAngles(2,1,0);
     */

    _initPosGuess[0]=rotAngle;
    //x y z translation
    VectorXd offsetTranslation = Eigen::AngleAxisd(_initPosGuess[0], Vector3d::UnitZ()).toRotationMatrix()*_trajData[0].topRightCorner<3,1>();
    _initPosGuess[1]=firstFrame(0,3)-offsetTranslation[0];
    _initPosGuess[2]=firstFrame(1,3)-offsetTranslation[1];
    _initPosGuess[3]=firstFrame(2,3)-offsetTranslation[2];


    //now construct the rest of the trajectory
    for(size_t i=0; i<_frameTransforms.size(); i++)
    {
        //compute the dx in the workspace from the previous frame
        MatrixXd desiredFrame = firstFrame*_frameTransforms[i];
        VectorXd dx = GeometryUtils::poseDiffExtrinsic(previousFrame, desiredFrame);

        //compute dtheta from that
        MatrixXd jacobian = _kdl.getJacobian(currentTheta);
        MatrixXd jacobianPinv = GeometryUtils::pseudoinverse(jacobian);
        VectorXd dtheta =  jacobianPinv*dx;

        //compute the new theta
        currentTheta = currentTheta + dtheta;

        //compute forward kinematics
        previousFrame = _kdl.getFKFrame(currentTheta);

        //add achieved theta to joints
        vector<double> jointsSTL = vector<double>(currentTheta.data(), currentTheta.data()+currentTheta.size());
        _initJoints.insert(_initJoints.end(), jointsSTL.begin(), jointsSTL.end());
    }
}

const unsigned int DataReader::timesteps() const {
    return _numFrames;
}

const vector<double> &DataReader::initPosGuess() const {
    return _initPosGuess;
}

