#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <autoware_msgs/VehicleCmd.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <tf/transform_listener.h>

namespace controller {


    class Controller
    {
        ros::NodeHandle& nh_;
        ros::Subscriber path_sub;
        ros::Subscriber odom_sub;
        ros::Publisher control_pub;
        ros::Publisher path_pub;
        ros::Publisher mark_pub;

        int wp_index, ctrl_index;
        int marker_id = 0;
        double Kp, Ki, Kd, current_heading;
        double axle_length, ctrl_max;
        double velocity = 10;
        
        double d_error;
        double i_error = 0;
        double prev_error = 0;

        nav_msgs::Odometry vehicle_odom;
        nav_msgs::Path path;

        bool ReadParameters();
        double PID(double error, double t_Kp, double t_Ki, double t_Kd);
        //void PathCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void PathCallback(const nav_msgs::Path::ConstPtr& msg);
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        double UpdateError(const geometry_msgs::Pose& target_point_pose, const geometry_msgs::Pose& current_point_pose);
        geometry_msgs::PoseStamped ChooseWaypoint(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path, int ctrl_idx, double ctrl_max);
        int ClosestWaypointIndex(const geometry_msgs::Pose& current_pose, const nav_msgs::Path& t_path);
        void ControlOutput();
        void LocalTransform(const geometry_msgs::Pose& target_point_pose, const geometry_msgs::Pose& current_point_pose, double transformed_vector[3]);

        public:

            Controller(ros::NodeHandle& nh);
            
    };
}