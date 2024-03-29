#ifndef M20IA_TRAJ_OPT_PAGMO_TRAJ_OPTIMIZATION_HPP
#define M20IA_TRAJ_OPT_PAGMO_TRAJ_OPTIMIZATION_HPP

#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pagmo/pagmo.hpp>
#include <pagmo/exceptions.hpp>
#include <pagmo/io.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/types.hpp>

#include "Robot_mocap_opt_problem.h"

namespace pagmo {
    struct Pagmo_traj_optimization {
        Pagmo_traj_optimization(Robot_mocap_opt_problem robot_opt_problem = Robot_mocap_opt_problem(1), vector_double::size_type dim = 2, vector_double::size_type nic = 0,
                                vector_double::size_type nec = 1) : opt_problem(robot_opt_problem), m_dim(dim),m_nic(nic), m_nec(nec) {
            //std::cerr<<"Getting dimensions"<<std::endl;
            m_dim = opt_problem.m_dim;
            m_nec = opt_problem.m_nec;
            m_nic = opt_problem.m_nic;
            //std::cerr<<"Got dimensions"<<std::endl;
        };


        // Fitness computation
        // This is the objective function+constraints giving the total cost
        vector_double fitness(const vector_double &x) const {
            std::vector<double> fit_val;//(1+m_nec+m_nic,0.0);
            double retval = 0.0;
            retval = opt_problem.objFunction(x);
            std::cout << "Cost value: " << retval << std::endl;
            fit_val.push_back(retval);
            std::vector<double> eq_consts = opt_problem.EqConstraints(x);
            fit_val.insert(fit_val.end(), eq_consts.begin(), eq_consts.end());
            std::vector<double> ineq_consts = opt_problem.inEqConstraints(x);
            fit_val.insert(fit_val.end(), ineq_consts.begin(), ineq_consts.end());
            return fit_val;// return {obj_fn,equality_constraints,inequality_constraints}

        }

        // Bounds(variable bounds)
        // This function encodes the constraints
        std::pair<vector_double, vector_double> get_bounds() const {
            std::vector<std::vector<double>> bounds = opt_problem.bounds();
            vector_double lb = bounds[0];
            vector_double ub = bounds[1];

            return {lb, ub};
        }

        // returns the number of inequality constraints
        vector_double::size_type get_nic() const {
            return m_nic;
        }

        // returns the number of equality constraints
        vector_double::size_type get_nec() const {
            return m_nec;
        }


        // Problem name
        std::string get_name() const {
            return "Trajectory Optimization Problem";
        }

        vector_double gradient(const vector_double &x) const {
	    //return pagmo::estimate_gradient(fitness , x);
            #if SPARSE_GRADIENT == true
            return opt_problem.sparse_gradient(x);
            #else
            return opt_problem.gradient(x);
            #endif
        }

        #if SPARSE_GRADIENT == true
        sparsity_pattern gradient_sparsity() const
        {
          return opt_problem.gradient_sparsity();
        }
        #endif

        template<typename Archive>
        void serialize(Archive &ar) {
            ar(m_dim);
        }

        // problem dimension
        vector_double::size_type m_dim;
        vector_double::size_type m_nec;
        vector_double::size_type m_nic;

        Robot_mocap_opt_problem opt_problem;
    };
}

#endif //M20IA_TRAJ_OPT_PAGMO_TRAJ_OPTIMIZATION_HPP
