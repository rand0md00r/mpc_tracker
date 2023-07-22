
#include <iostream>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <tracker_node.h>
#include "mpc.h"
#include <Eigen/Core>
#include <Eigen/QR>
#include <chrono>

using namespace std;
using namespace Eigen;

MPCNode::MPCNode(ros::NodeHandle& nh) : _nh(nh)
{
    // set Params for control loop and MPC solver from param_server
    setParameters();

    // load params for mpc class
    loadParamsForSolver();

    // set topic, frame, sub, pub, timer
    setSubPub();

    ROS_INFO("Path tracker loaded. Waiting for global path ~");
}

void MPCNode::setParameters()
{
    // Parameters for control loop
    _nh.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
    _nh.param("pub_twist_cmd",  _pub_twist_flag, true);
    _nh.param("debug_info",     _debug_info,     true);
    _nh.param("time_spent",     _time_spent,     true);
    _nh.param("max_speed",      _max_speed,      1.0); // unit: m/s
    _nh.param("waypoints_dist", _waypointsDist, -1.0); // unit: m
    _nh.param("path_length",    _pathLength,     5.0); // unit: m
    _nh.param("goal_radius",    _goalRadius,     0.5); // unit: m
    _nh.param("controller_freq",_contr_freq,     10);
    _dt = double(1.0/_contr_freq);


    // Parameter for MPC solver
    _nh.param("mpc_steps",      _mpc_steps, 20.0);
    _nh.param("mpc_ref_cte",    _ref_cte, 0.0);
    _nh.param("mpc_ref_vel",    _ref_vel, 1.0);
    _nh.param("mpc_ref_etheta", _ref_etheta, 0.0);

    _nh.param("mpc_max_angvel",     _max_angvel,    0.5);
    _nh.param("mpc_max_throttle",   _max_throttle,  1.0);
    _nh.param("mpc_bound_value",    _bound_value,   1.0e3);

    _nh.param("mpc_w_cte",      _w_cte,         5000.0);
    _nh.param("mpc_w_etheta",   _w_etheta,      5000.0);
    _nh.param("mpc_w_vel",      _w_vel,         100000.0);
    _nh.param("mpc_w_angvel",   _w_angvel,      100.0);
    _nh.param("mpc_w_angvel_d", _w_angvel_d,    10.0);
    _nh.param("mpc_w_accel",    _w_accel,       0.0);
    _nh.param("mpc_w_accel_d",  _w_accel_d,     0.0);


    // Init variables
    _twist_msg  = geometry_msgs::Twist();
    _mpc_traj   = nav_msgs::Path();

    _goal_received = false;
    _goal_reached  = false;
    _path_computed = false;
    _velOdom_flag  = false;
    _poseOdom_flag = false;
    _odom_flag     = false;
    _move_forward  = true;

    _throttle   = 0.0;
    _w          = 0.0;
    _speed      = 0.0;

}

void MPCNode::loadParamsForSolver()
{
    //Init parameters for MPC object
    _mpc_params["DT"]         = _dt;
    _mpc_params["STEPS"]      = _mpc_steps;
    _mpc_params["REF_CTE"]    = _ref_cte;
    _mpc_params["REF_ETHETA"] = _ref_etheta;
    _mpc_params["REF_V"]      = _ref_vel;
    _mpc_params["W_CTE"]      = _w_cte;
    _mpc_params["W_EPSI"]     = _w_etheta;
    _mpc_params["W_V"]        = _w_vel;
    _mpc_params["W_ANGVEL"]   = _w_angvel;
    _mpc_params["W_A"]        = _w_accel;
    _mpc_params["W_DANGVEL"]  = _w_angvel_d;
    _mpc_params["W_DA"]       = _w_accel_d;
    _mpc_params["ANGVEL"]     = _max_angvel;
    _mpc_params["MAXTHR"]     = _max_throttle;
    _mpc_params["BOUND"]      = _bound_value;
    _mpc_params["DEBUG"]      = _debug_info;
    _mpc.LoadParams(_mpc_params);

    ROS_INFO("----------------------------------------");
    ROS_INFO("MPC Tracker Parameters have been loaded.");
    ROS_INFO("----------------------------------------");
}

void MPCNode::setSubPub()
{
    //Parameter for topics
    _nh.param<std::string>("global_path_topic", _globalPath_topic,  "/global_path"  );
    _nh.param<std::string>("goal_topic",        _goal_topic,        "/move_base_simple/goal" );
    _nh.param<std::string>("odom_topic",        _odom_topic,        "/Odometry" );
    _nh.param<std::string>("base_odom_topic",   _base_odom_topic,   "/bunker_odom" );

    //Parameter for Frame name
    _nh.param<std::string>("odom_frame",    _odom_frame,    "odom");        // Global Frame
    _nh.param<std::string>("car_frame",     _car_frame,     "base_link" );  // Local Frame

    //Subscribers
    _sub_odom     = _nh.subscribe(_odom_topic, 1,           &MPCNode::odomCB, this);
    _sub_path     = _nh.subscribe( _globalPath_topic, 1,    &MPCNode::pathCB, this);
    _sub_goal     = _nh.subscribe( _goal_topic, 1,          &MPCNode::goalCB, this);
    _sub_base_vel = _nh.subscribe(_base_odom_topic, 1,      &MPCNode::veloCB, this);

    //Publishers
    _pub_odompath = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1);
    _pub_mpctraj  = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);
    _pub_twist    = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    //Timer for Control loop
    _timer1 = _nh.createTimer(ros::Duration((1.0)/_contr_freq), &MPCNode::controlLoopCB, this);
}

// Public: return _thread_numbers
int MPCNode::get_thread_numbers()
{
    return _thread_numbers;
}

// Evaluate a polynomial.
double MPCNode::polyeval(VectorXd coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

VectorXd MPCNode::polyfit(VectorXd xvals, VectorXd yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// CallBack: Update odometry
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    // fast_lio2's /Odometry have no vel & angvel
    if(odomMsg->twist.twist.linear.x == 0.0 || odomMsg->twist.twist.angular.z == 0.0)
    {
        nav_msgs::Odometry tempOdom = *odomMsg;
        _odom.child_frame_id = tempOdom.child_frame_id;
        _odom.header = tempOdom.header;
        _odom.pose = tempOdom.pose;
        _poseOdom_flag = true;
    }
    // base_odom in gazebo do have vel & angvel
    else
    {
        _odom = *odomMsg;
        _poseOdom_flag = true;
        _velOdom_flag = true;
    }

}

void MPCNode::veloCB(const nav_msgs::Odometry::ConstPtr& baseOdomMsg) 
{
    // use buker_odom's vel & angvel, when running on the bunker
    nav_msgs::Odometry tempOdom = *baseOdomMsg;
    _odom.twist.twist.linear.x = tempOdom.twist.twist.linear.x;
    _odom.twist.twist.angular.z = tempOdom.twist.twist.angular.z;
    _velOdom_flag = true;
}

// CallBack: Update path waypoints (conversion to odom frame)
void MPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    if(_goal_received && !_goal_reached && !pathMsg->poses.empty())
    {
        int downSampling = 0;
        nav_msgs::Path odom_path = nav_msgs::Path();
        try
        {
            double total_length = 0.0;
            int sampling = downSampling;

            //find waypoints distance
            if(_waypointsDist <=0.0)
            {
                double dx = pathMsg->poses[1].pose.position.x - pathMsg->poses[0].pose.position.x;
                double dy = pathMsg->poses[1].pose.position.y - pathMsg->poses[0].pose.position.y;
                _waypointsDist = sqrt(dx*dx + dy*dy);
                downSampling = int(_pathLength/10.0/_waypointsDist);
            }            

            // Cut and downsampling the path
            for(int i =0; i< pathMsg->poses.size(); i++)
            {
                if(total_length > _pathLength)
                    break;

                if(sampling == downSampling)
                {
                    odom_path.poses.push_back(pathMsg->poses[i]);  
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist; 
                sampling = sampling + 1;  
            }

            if(odom_path.poses.size() >= 6 )
            {
                _odom_path = odom_path; // Path waypoints in odom frame
                _path_computed = true;
                // publish odom path
                odom_path.header.frame_id = _odom_frame;
                odom_path.header.stamp = ros::Time::now();
                _pub_odompath.publish(odom_path);
            }
            else
            {
                ROS_INFO("Insufficient number of path points. Which is: %zu", odom_path.poses.size());
                _waypointsDist = -1;
            }
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    
}

// CallBack: Update goal status
void MPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    _goal_pos = goalMsg->pose.position;
    _goal_received = true;
    _goal_reached = false;
    ROS_INFO("Goal Received :goalCB!");
}


void MPCNode::checkReachedGoal(const double px, const double py) {
    if(_goal_received)
    {
        double car2goal_x = _goal_pos.x - px;
        double car2goal_y = _goal_pos.y - py;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        if(dist2goal < _goalRadius)
        {
            _goal_received = false;
            _goal_reached = true;
            _path_computed = false;
            ROS_INFO("Goal Reached !");
        }
    }
}

tuple<VectorXd, VectorXd> MPCNode::calculateActionState(robot_state cur_st)
{
    nav_msgs::Path odom_path = _odom_path;

    // Waypoints related parameters
    const int N = odom_path.poses.size();
    const double costheta = cos(cur_st.theta);
    const double sintheta = sin(cur_st.theta);

    // Convert to the vehicle coordinate system
    VectorXd x_veh(N);
    VectorXd y_veh(N);
    for(int i = 0; i < N; i++)
    {
        const double dx = odom_path.poses[i].pose.position.x - cur_st.x;
        const double dy = odom_path.poses[i].pose.position.y - cur_st.y;
        x_veh[i] = dx * costheta + dy * sintheta;
        y_veh[i] = dy * costheta - dx * sintheta;
    }
    
    if(!N)
    {
        if(x_veh[1] > 0)
        {
            _move_forward = true;
        }
        else
        {
            _move_forward = false;
        }
        _forwardLookingPoints.first = x_veh[N / 2];
        _forwardLookingPoints.second = y_veh[N / 2];
    }

    auto coeffs = polyfit(x_veh, y_veh, 3); 

    const double cte  = polyeval(coeffs, 0.0);
    const double etheta = atan(coeffs[1]);

    // Update state for action time
    VectorXd state(6);
    const double px_act = cur_st.v * _dt;
    const double py_act = 0;
    const double theta_act = cur_st.w * _dt;
    const double v_act = cur_st.v + cur_st.a * _dt;
    const double cte_act = cte + cur_st.v * sin(etheta) * _dt;
    double etheta_act = etheta - theta_act;

    state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;

    tuple<VectorXd, VectorXd> actState(state, coeffs);

    return actState;
}

void MPCNode::modifyResults()
{
    if(_speed >= _max_speed) 
    {
        _speed = _max_speed;
    }

    // 自旋
    if(_forwardLookingPoints.first < 0.0 ) 
    {
        _speed = 0;
        if(_forwardLookingPoints.second > 0) _w = 0.5;
        if(_forwardLookingPoints.second < 0) _w = -0.5;
    }

    // if(!_move_forward)
    // {
    //     if(_forwardLookingPoints.second > 0)
    //     {
    //         _w = - abs(_w);
    //     }
    //     else
    //     {
    //         _w = abs(_w);
    //     }
        
    // }
}

void MPCNode::pubMPCtrajctory()
{
    // Display the MPC predicted trajectory
    _mpc_traj = nav_msgs::Path();
    _mpc_traj.header.frame_id = _car_frame;
    _mpc_traj.header.stamp = ros::Time::now();

    for(int i=0; i<_mpc.mpc_x.size(); i++)
    {
        geometry_msgs::PoseStamped tempPose;
        tempPose.header = _mpc_traj.header;
        tempPose.pose.position.x = _mpc.mpc_x[i];
        tempPose.pose.position.y = _mpc.mpc_y[i];
        tempPose.pose.orientation.w = 1.0;
        _mpc_traj.poses.push_back(tempPose); 
    }

    _pub_mpctraj.publish(_mpc_traj);
}


void MPCNode::controlLoopCB(const ros::TimerEvent&)
{
    // Received Goal & Goal not reached & Received Odom & Received Path
    _odom_flag = _poseOdom_flag & _velOdom_flag;
    if(_goal_received && !_goal_reached && _path_computed && _odom_flag)
    {    
        nav_msgs::Odometry odom = _odom; 
        tf::Pose pose;
        tf::poseMsgToTF(odom.pose.pose, pose);

        // Update Current State
        robot_state cur_st(odom.pose.pose.position.x, 
                                        odom.pose.pose.position.y, 
                                        tf::getYaw(pose.getRotation()),
                                        odom.twist.twist.linear.x,
                                        odom.twist.twist.angular.z,
                                        _throttle);
        checkReachedGoal(cur_st.x, cur_st.y);

        // Update Action State
        tuple<VectorXd, VectorXd> actState = calculateActionState(cur_st);
        VectorXd state = get<0>(actState);
        VectorXd coeffs = get<1>(actState);

        // Solve MPC Problem
        auto start_time = std::chrono::high_resolution_clock::now();
        vector<double> mpc_results = _mpc.Solve(state, coeffs);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration_ms = (end_time - start_time) / 1000000;

        // MPC result
        _w          = mpc_results[0];           // radian/sec, angular velocity
        _throttle   = mpc_results[1];           // acceleration
        _speed      = cur_st.v + _throttle*_dt; // speed

        modifyResults();
        pubMPCtrajctory();

        // Print Debug Infomation
        if(_time_spent)
        {
            cout << "--- Solving Success: Time Spent: " << duration_ms.count() << endl;
        }
    }
    else
    {
        _throttle = 0.0;
        _speed    = 0.0;
        _w        = 0.0;

        if(_goal_reached && _goal_received)
        {
            ROS_INFO("Goal Reached: control loop !");
        }
        if(!_odom_flag)
        {
            ROS_INFO("Warning: NO Odom Massage Received ! ");
        }
        if(!_path_computed && _goal_received)
        {
            ROS_INFO("Warning: Failed to Compute Path  ! ");
        }
    }

    // Publish Twist
    if(_pub_twist_flag)
    {
        _twist_msg.linear.x  = _speed; 
        _twist_msg.angular.z = _w;
        _pub_twist.publish(_twist_msg);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "MPC_TRACKER_NODE");
    ros::NodeHandle nh("~");

    MPCNode mpc_node(nh);

    ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
    spinner.start();
    
    ros::waitForShutdown();
    return 0;
}

