//
// Created by klevis on 10/31/17.
//

#include <angles/angles.h>
#include <csv.h>
#include <trajectory_optimization.hpp>
#include <fstream>
#include <GradientCheck.hpp>

using namespace pagmo;
using namespace std;
using namespace Eigen;

void printVector(vector<double> &vec)
{
    cout << Map<VectorXd>(vec.data(), vec.size()) << endl;
}

int main(int argc, char** argv)
{
	int endIndex = stoi(argv[1]);
	double timeConstant = stod(argv[2]);
	unsigned int pop_size = static_cast<unsigned int>(stoi(argv[3]));
	double stopValue = stod(argv[4]);
	string joints_file = argv[5];
	string traj_file = argv[6];
    unsigned int checkGradient = static_cast<unsigned int>(stoi(argv[7]));
	
    // urdf file location:
    string urdf_file = "/home/klevis/Desktop/URDF/m20iag.urdf";
    vector<string> ee_names={"tool0"};
    vector<string> base_names={"base"};
    vector<double> g_vec={0.0,0.0,-9.8};

    // initialize kdl class
    manipulator_kdl::robotKDL m20ia(urdf_file,base_names,ee_names,g_vec);
    m20ia.urdfParser();

    //set the velocity limits
    vector<double> vel_limits(6);
    vel_limits[0]=3.403392;
    vel_limits[1]=3.054326;
    vel_limits[2]=3.141593;
    vel_limits[3]=6.283185;
    vel_limits[4]=6.283185;
    vel_limits[5]=9.599311;

    //read in the trajectory
    io::CSVReader<7> input(traj_file);
    double tb,px,py,pz,yaw,pitch,roll;
    vector<vector<double>> data_arr;
    vector<double> timeBetween;

    while(input.read_row(tb,px,py,pz,yaw,pitch,roll))
    {
        vector<double> currentRow = {px,py,pz,yaw,pitch,roll};
        data_arr.push_back(currentRow);
        timeBetween.push_back(tb*timeConstant);
    }

    //read in the initial guess for the joint angles
    io::CSVReader<7> inputJ(joints_file);
    double j1,j2,j3,j4,j5,j6;
    int timeIndex;
    vector<vector<double>> joints0;
    while(inputJ.read_row(timeIndex,j1,j2,j3,j4,j5,j6)) {
        vector<double> currentRow = {j1, j2, j3, j4, j5, j6};
        joints0.push_back(currentRow);
    }

    //truncate the number of steps in case we want to only consider part of the trajectory
    vector<vector<double>> data_arr_t(data_arr.begin(), data_arr.begin()+endIndex);
    vector<double> timeBetween_t(timeBetween.begin(), timeBetween.begin()+endIndex);

    //create the optimization problem
    opt_problems::optProblem optProblem(m20ia, data_arr_t, vel_limits, timeBetween_t);
    problem prob{trajectory_optimization{optProblem}};

    //set the tolerances
    vector<double> tol(m20ia.dof * (data_arr_t.size()), 1e-3);
    prob.set_c_tol(tol);

    //start the optimization
    nlopt nlopt{"slsqp"};
    nlopt.set_verbosity(10);
    nlopt.set_xtol_rel(0);
    nlopt.set_stopval(stopValue);
    algorithm algo{nlopt};
    algo.set_verbosity(10);
    population pop{prob};
    if(pop_size>0) {
        pop=population{prob, pop_size};
    }

    //create the first guess vector
    assert(joints0.size()==1 || joints0.size()==data_arr_t.size());

    vector<double> x_in(m20ia.dof*data_arr_t.size(),0.0);

    if(joints0.size()==1)
    {
        for (int i = 0; i < data_arr_t.size(); i++) {
            for (int j = 0; j < m20ia.dof; j++) {
                x_in[i * m20ia.dof + j] = joints0[0][j];
            }
        }
    }
    else {
        for (int i = 0; i < joints0.size(); i++) {
            for (int j = 0; j < m20ia.dof; j++) {
                x_in[i * m20ia.dof + j] = joints0[i][j];
            }
        }
    }

    //check the gradient and return if instructed to do so
    if(checkGradient) {
        GradientCheck gc(optProblem);
        return gc.checkGradientObj(x_in, 1E-5);
    }

    //send the initial guess to the optimization problem
    vector<double> f_in=prob.fitness(x_in);
    pop.push_back(x_in,f_in);
    cout << "Initial Fitness: " << endl;
    printVector(f_in);

    //solve the problem
    population pop_res = algo.evolve(pop);
    cerr<<"Final cost:"<<pop_res.champion_f()[0]<<endl;

    //print the resulting joint angles
    vector<double> x_best=pop_res.champion_x();
    vector<vector<double>> solved_joint_angles;

    vector<double> current_joint_angles;
    current_joint_angles.resize(6,0.0);

    for(int i=0; i<x_best.size(); i++)
    {
        current_joint_angles[i%m20ia.dof] = x_best[i];
        if(i%m20ia.dof == 5) {
            solved_joint_angles.push_back(current_joint_angles);
        }
    }

    cout << "Joints: " << endl;
    for(int i=0; i<solved_joint_angles.size(); i++)
    {
        printVector(solved_joint_angles[i]);
    }

    cout << "Positions: " << endl;
    for(int i=0; i<solved_joint_angles.size(); i++)
    {
        vector<double> currentPose = m20ia.getFK(0,solved_joint_angles[i], true);
        cout << "Pose" << i+1 << " :" << endl;
        printVector(currentPose);
    }
}
