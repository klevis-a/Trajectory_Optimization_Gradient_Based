#include <Eigen/Dense>
#include <GradientCheck.hpp>

using namespace std;
using namespace Eigen;

GradientCheck::GradientCheck(opt_problems::Robot_mocap_opt_problem &p, double tolerance) : problem(p)
{
    _tolerance = tolerance;

    vector<double> lowBounds = problem.bounds()[0];
    vector<double> upBounds = problem.bounds()[1];

    //the dof is determined by the bounds
    _total_dof = lowBounds.size();

    random_device rd;
    eng = mt19937(rd());

    //create a random number generator that satisfies the bounds criteria
    //for each degree of freedom
    for(size_t i=0; i!= _total_dof; i++)
    {
        dof_distr.push_back(uniform_real_distribution<double>(lowBounds[i], upBounds[i]));
    }
}

bool GradientCheck::checkGradientObj(vector<double> x0)
{
    //compute the gradient using the central finite difference method
	double dx=0.0001;
	vector<double> gradient(x0.size());
	
	for(int i=0; i<x0.size(); i++)
	{
		vector<double> px=x0;
		px[i]+=dx;
		vector<double> mx=x0;
		mx[i]-=dx;
		//double fxpdx = problem.objFunction(px);
		//double fxmdx = problem.objFunction(mx);
        double fxpdx = problem.EqConstraints(px)[0];
        double fxmdx = problem.EqConstraints(mx)[0];
		gradient[i] = (fxpdx-fxmdx)/(2*dx);
	}

    //compute the analytical gradient
	//vector<double> gradientA = problem.objGradient(x0);
    vector<double> gradientA = problem.eqGradient(x0);

    //convert to Eigen and compute the difference
	VectorXd gradientE = Map<VectorXd>(gradient.data(), gradient.size());
	VectorXd gradientAE = Map<VectorXd>(gradientA.data(), gradientA.size());
	VectorXd gradientDiff = gradientAE - gradientE;

    //output
    cout << "X_vec" << endl << Map<VectorXd>(x0.data(), x0.size()).transpose() << endl;
	cout << "Analytical Gradient: " << endl << gradientAE.transpose() << endl;
	cout << "Finite Difference Gradient: " << endl << gradientE.transpose() << endl;
	cout << "Difference: " << endl << gradientDiff.transpose() << endl;

    //check that all differences are under the desired tolerance
    return (gradientDiff.cwiseAbs().array() < _tolerance).all();
}

bool GradientCheck::checkGradientRandom(unsigned int trials)
{
    //this method simply creates a random x_in vector and calls the
    //checkGradient method for a specified number of trials.
    //note that this method will stop on the trial that we encounter a failure
    //due to compiler optimization
    bool success=true;

    for(size_t i=0; i!=trials; i++)
    {
        vector<double> x_in = createRandomX();
        success = success && checkGradientObj(x_in);
    }

    return success;
}

vector<double> GradientCheck::createRandomX()
{
    //create a random x_in vector based off the created random number generators
    vector<double> xvec(_total_dof);

    for(size_t i=0; i<_total_dof; i++)
    {
        xvec[i] = dof_distr[i](eng);
    }

    return xvec;
}