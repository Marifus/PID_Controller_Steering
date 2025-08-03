#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <autoware_msgs/VehicleCmd.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <deque>

namespace controller {


    class Controller
    {
        bool input_log, output_log;

        ros::NodeHandle& nh_;
        ros::Subscriber path_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber docking_signal_sub;
        ros::Publisher control_pub;
        ros::Publisher mark_pub;
        //ros::Publisher path_pub;

        int wp_index, ctrl_index;
        int marker_id;
        double Kp, Ki, Kd, Kp_max, Ki_max, Kd_max, Kp_min, Ki_min, Kd_min;
        double current_heading, current_velocity;
        double wheelbase;
        double high_ctrl_max, low_ctrl_max, ctrl_max;
        double max_velocity, high_velocity, low_velocity, velocity;
        std::string odom_topic, path_topic, cmd_topic, marker_topic, docking_signal_topic;
        double herhangi_bir_sey;
        bool docking, path_received;
        int filter_length;
        double weight_current;
        double curvature_coefficient, curvature_lookahead, max_curvature;
        int curvature_index;

        std::deque<double> prev_inputs_deq;
        
        double steering_prev_error;
        double steering_i_error;
        double steering_d_error;

        double velocity_prev_error;
        double velocity_i_error;
        double velocity_d_error;

        double prev_time;
        double current_time;

        nav_msgs::Odometry vehicle_odom;
        nav_msgs::Path path;

        ros::Timer timer;

        bool ReadParameters();
        //void PathCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void PathCallback(const nav_msgs::Path::ConstPtr& msg);
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void DockingCallback(const std_msgs::Bool& msg);
        void TimerCallback(const ros::TimerEvent&);
        void ControlOutput();
        double PID(double error, double t_Kp, double t_Ki, double t_Kd, double dt, double& prev_error, double& i_error, double& d_error);
        double UpdateSteeringError(const geometry_msgs::Pose& target_point_pose, const geometry_msgs::Pose& current_point_pose);
        geometry_msgs::PoseStamped ChooseWaypoint(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path, int ctrl_idx, double ctrl_max);
        int ClosestWaypointIndex(const geometry_msgs::Pose& current_pose, const nav_msgs::Path& t_path);
        geometry_msgs::Pose LocalTransform(const geometry_msgs::Pose& origin_pose, const geometry_msgs::Pose& target_point_pose);
        double GetYaw(const geometry_msgs::Quaternion& q);
        double CalculatePathCurv(const geometry_msgs::Pose& current_point_pose,const nav_msgs::Path& t_path, int curv_idx, double t_curv_lookahead, double t_max_curv);
        double CalculateVelocityCmd(double velocity, double curvature, double k_curvature);
        double LowPassFilter(double input, std::deque<double> &prev_inputs, int f_length, double w_current);
        void UpdatePIDCoefficients(double& Kp, double& Ki, double& Kd, double& ctrl_max);

        public:

            Controller(ros::NodeHandle& nh);
            
    };
}