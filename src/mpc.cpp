
#include "ros/ros.h"
#include "mpc.h"
#include "fg_eval.h"
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <string>
#include <stdexcept>

using CppAD::AD;

MPC::MPC()
{
    // 设置默认值
    _mpc_steps    = 20;
    _max_angvel   = 3.0; // Maximal angvel radian (~30 deg)
    _max_throttle = 1.0; // Maximal throttle accel
    _bound_value  = 1.0e3; // Bound value for other variables

    _x_start      = 0;
    _y_start      = _x_start + _mpc_steps;
    _theta_start  = _y_start + _mpc_steps;
    _v_start      = _theta_start + _mpc_steps;
    _cte_start    = _v_start + _mpc_steps;
    _etheta_start = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _a_start      = _angvel_start + _mpc_steps - 1;

}

void MPC::LoadParams(const std::map<std::string, double> &params)
{
    _params = params;
    //Init parameters for MPC object
    _mpc_steps    = _params.find("STEPS")  != _params.end() ? _params.at("STEPS")  : _mpc_steps;
    _max_angvel   = _params.find("ANGVEL") != _params.end() ? _params.at("ANGVEL") : _max_angvel;
    _max_throttle = _params.find("MAXTHR") != _params.end() ? _params.at("MAXTHR") : _max_throttle;
    _bound_value  = _params.find("BOUND")  != _params.end() ? _params.at("BOUND")  : _bound_value;
    
    _x_start      = 0;
    _y_start      = _x_start + _mpc_steps;
    _theta_start  = _y_start + _mpc_steps;
    _v_start      = _theta_start + _mpc_steps;
    _cte_start    = _v_start + _mpc_steps;
    _etheta_start = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _a_start      = _angvel_start + _mpc_steps - 1;
     
    ROS_INFO("\n!! MPC Obj parameters updated !! ");
}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double v = state[3];
    const double cte = state[4];
    const double etheta = state[5];


    // 设置模型变量
    size_t n_vars = _mpc_steps * 6 + (_mpc_steps - 1) * 2;
    
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_x_start] = x;
    vars[_y_start] = y;
    vars[_theta_start] = theta;
    vars[_v_start] = v;
    vars[_cte_start] = cte;
    vars[_etheta_start] = etheta;

    // 设置变量的上下限
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // 将所有非执行机构的上限和下限设置为最大负值和正值。
    for (int i = 0; i < _angvel_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }
    // 角度的上限和下限分别设置为-25度和25度（以弧度为单位的值）。
    for (int i = _angvel_start; i < _a_start; i++) 
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }
    // 加速/减速上限和下限
    for (int i = _a_start; i < n_vars; i++)  
    {
        vars_lowerbound[i] = -_max_throttle;
        vars_upperbound[i] = _max_throttle;
    }

    // 约束的下限和上限
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    constraints_lowerbound[_v_start] = v;
    constraints_lowerbound[_cte_start] = cte;
    constraints_lowerbound[_etheta_start] = etheta;
    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    constraints_upperbound[_v_start] = v;
    constraints_upperbound[_cte_start] = cte;
    constraints_upperbound[_etheta_start] = etheta;

    // 计算目标和约束的对象
    FG_eval fg_eval(coeffs);
    fg_eval.LoadParams(_params);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    // 注意：将稀疏设置为true允许解算器利用稀疏例程，这使计算速度更快。如果你可以取消注释其中的1个，
    // 看看它是否有影响，但如果你取消注释这两个，计算时间应该会增加几个数量级。
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    // 注意：当前解算器的最大时间限制为0.5秒。根据需要更改此项。
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    if(!ok)
    {
        throw std::runtime_error("An error occurred: Failed to Solve MPC.");
    }

    this->mpc_x = {};
    this->mpc_y = {};
    this->mpc_theta = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
        this->mpc_theta.push_back(solution.x[_theta_start + i]);
    }
    
    vector<double> result;
    result.push_back(solution.x[_angvel_start]);
    result.push_back(solution.x[_a_start]);
    return result;
}
