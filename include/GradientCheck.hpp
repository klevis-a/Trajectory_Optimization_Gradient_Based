#include <vector>
#include <opt_problem.hpp>

class GradientCheck
{
	public:
		GradientCheck(opt_problems::optProblem &p);
		bool checkGradientObj(std::vector<double> x0, double tolerance);
	private:
		opt_problems::optProblem &problem;
};
