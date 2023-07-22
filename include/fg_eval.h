
#ifndef FGEVAL_H
#define FGEVAL_H

#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <string>
#include <map>

using CppAD::AD;

class FG_eval 
{
    public:
        Eigen::VectorXd coeffs;
        double _dt, _ref_cte, _ref_etheta, _ref_vel; 
        double  _w_cte, _w_etheta, _w_vel, _w_angvel, _w_accel, _w_angvel_d, _w_accel_d;
        int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start;
        bool _dbg_info;

        FG_eval(Eigen::VectorXd coeffs);

        AD<double> cost_cte, cost_etheta, cost_vel, cost_angvel, cost_accel, cost_angvel_d, cost_accel_d;

        void LoadParams(const std::map<std::string, double> &params);
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
        void operator()(ADvector& fg, const ADvector& vars);
};



#endif /* FGEVAL_H */
