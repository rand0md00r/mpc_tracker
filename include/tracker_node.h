
#ifndef MPCNODE_H
#define MPCNODE_H

#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "mpc.h"

struct robot_state {
    const double x;
    const double y;
    const double theta;
    const double v;
    const double w;
    const double a;

    robot_state() : x(0.0), y(0.0), theta(0.0), v(0.0), w(0.0), a(0.0) {}
    robot_state(const double x_, const double y_, const double theta_, const double v_, const double w_, const double a_) : 
    x(x_), y(y_), theta(theta_), v(v_), w(w_), a(a_) {}
};

class MPCNode
{
    public:
        MPCNode(ros::NodeHandle& nh);
        int get_thread_numbers();
        
    private:
        ros::NodeHandle _nh;

        MPC _mpc;

        ros::Subscriber _sub_odom, _sub_path, _sub_goal, _sub_amcl, _sub_base_vel;
        ros::Publisher _pub_odompath, _pub_twist, _pub_mpctraj;
        ros::Publisher _pub_ackermann;
        ros::Timer _timer1;

        // control loop
        int _thread_numbers, _contr_freq;
        bool _pub_twist_flag, _debug_info, _goal_received, _goal_reached, _path_computed, _time_spent, _move_forward;
        double _max_speed, _waypointsDist, _pathLength, _goalRadius;

        geometry_msgs::Point _goal_pos;
        nav_msgs::Odometry _odom;
        nav_msgs::Path _odom_path, _mpc_traj; 
        geometry_msgs::Twist _twist_msg;

        bool _velOdom_flag, _poseOdom_flag, _odom_flag;

        string _globalPath_topic, _goal_topic, _odom_topic, _base_odom_topic;
        string _map_frame, _odom_frame, _car_frame;

        
        map<string, double> _mpc_params;
        double _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel, 
               _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value;

        double _dt, _w, _throttle, _speed;

        pair<double, double> _forwardLookingPoints;
        
        


        void setSubPub();
        void setParameters();
        void loadParamsForSolver();
        
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        
        void checkReachedGoal(const double px, const double py);
        void veloCB(const nav_msgs::Odometry::ConstPtr& baseOdomMsg);
        void modifyResults();
        void pubMPCtrajctory();
        tuple<Eigen::VectorXd, Eigen::VectorXd> calculateActionState(robot_state cur_st);
        double polyeval(Eigen::VectorXd coeffs, double x);
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

        void controlLoopCB(const ros::TimerEvent&);

}; // end of class

#endif /* MPCNODE_H */
