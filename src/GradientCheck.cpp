#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <GradientCheck.hpp>

GradientCheck::GradientCheck(opt_problems::optProblem &p) : problem(p)
{
}

bool GradientCheck::checkGradientObj(vector<double> x0, double tolerance)
{
	double dx=0.01;
	vector<double> gradient(x0.size());
	
	for(int i=0; i<x0.size(); i++)
	{
		vector<double> px=x0;
		px[i]+=dx;
		vector<double> mx=x0;
		mx[i]-=dx;
		double fxpdx = problem.objFunction(px);
		double fxmdx = problem.objFunction(mx);
		
		gradient[i] = (fxpdx-fxmdx)/(2*dx);
	}
	
	vector<double> gradientA = problem.objGradient(x0);
	
	Eigen::VectorXd gradientE = Map<Eigen::VectorXd>(gradient.data(), gradient.size());
	Eigen::VectorXd gradientAE = Map<Eigen::VectorXd>(gradientA.data(), gradientA.size());
	Eigen::VectorXd gradientDiff = gradientAE - gradientE;
	
	cout << "Analytical Gradient: " << endl << gradientAE.transpose() << endl;
	cout << "Finite Difference Gradient: " << endl << gradientE.transpose() << endl;
	cout << "Difference: " << endl << gradientDiff.transpose() << endl;

    return (gradientDiff.array() < tolerance).all();
}
