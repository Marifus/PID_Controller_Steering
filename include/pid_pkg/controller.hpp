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

namespace controller {


    class Controller
    {
        bool input_log, output_log;

        ros::NodeHandle& nh_;
        ros::Subscriber path_sub;
        ros::Subscriber odom_sub;
        ros::Publisher control_pub;
        //ros::Publisher path_pub;
        ros::Publisher mark_pub;

        int wp_index, ctrl_index;
        int marker_id;
        double Kp, Ki, Kd, current_heading;
        double wheelbase, ctrl_max;
        double velocity;
        std::string odom_topic, path_topic, cmd_topic, marker_topic;
        
        double d_error;
        double i_error;
        double prev_error;

        double prev_time;
        double current_time;

        nav_msgs::Odometry vehicle_odom;
        nav_msgs::Path path;

        bool ReadParameters();
        //void PathCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void PathCallback(const nav_msgs::Path::ConstPtr& msg);
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void ControlOutput();
        double PID(double error, double t_Kp, double t_Ki, double t_Kd, double dt);
        double UpdateError(const geometry_msgs::Pose& target_point_pose, const geometry_msgs::Pose& current_point_pose);
        geometry_msgs::PoseStamped ChooseWaypoint(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path, int ctrl_idx, double ctrl_max);
        int ClosestWaypointIndex(const geometry_msgs::Pose& current_pose, const nav_msgs::Path& t_path);
        geometry_msgs::Pose LocalTransform(const geometry_msgs::Pose& origin_pose, const geometry_msgs::Pose& target_point_pose);
        double GetYaw(const geometry_msgs::Quaternion& q);

        public:

            Controller(ros::NodeHandle& nh);
            
    };
}