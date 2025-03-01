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
        double Ko, Ki, Kt, current_heading;
        double velocity = 10;
        
        double d_error;
        double i_error = 0;
        double prev_error = 0;

        geometry_msgs::PoseStamped target_point;
        geometry_msgs::PoseWithCovariance current_point;
        nav_msgs::Odometry vehicle_odom;
        nav_msgs::Path path;

        bool ReadParameters();
        double PID(double error, double t_Ko, double t_Ki, double t_Kt);
        void PathCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        double UpdateError(geometry_msgs::Pose& target_point_pose, geometry_msgs::Pose& current_point_pose);
        void ChooseWaypoint();
        void ControlOutput();
        void LocalTransform(geometry_msgs::Pose& target_point_pose, geometry_msgs::Pose& current_point_pose, double transformed_vector[3]);

        public:

            Controller(ros::NodeHandle& nh);
            
    };
}