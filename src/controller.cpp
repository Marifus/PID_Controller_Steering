#include "pid_pkg/controller.hpp"

namespace controller
{

    Controller::Controller(ros::NodeHandle& nh) : nh_(nh)
    {

        if (!ReadParameters())
        {
            ROS_ERROR("Parametreler Okunamadi.");
            ros::requestShutdown();
        }

        path_sub = nh_.subscribe("/odom", 10, &Controller::PathCallback, this);
        odom_sub = nh_.subscribe("/odom_sim", 10, &Controller::OdomCallback, this);
        control_pub = nh_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 10);
        path_pub = nh_.advertise<nav_msgs::Path>("/path", 10);
        mark_pub = nh_.advertise<visualization_msgs::Marker>("/waypoint", 10);

    }


    bool Controller::ReadParameters()
    {
        if (!nh_.getParam("pid_katsayilari/Ko", Ko)) return false;
        if (!nh_.getParam("pid_katsayilari/Ki", Ki)) return false;
        if (!nh_.getParam("pid_katsayilari/Kt", Kt)) return false;
        if (!nh_.getParam("ctrl_index", ctrl_index)) return false;

        ROS_INFO("PID Katsayilari: [%f, %f, %f]", Ko, Ki, Kt);
        ROS_INFO("Control Index: [%d]", ctrl_index);

        return true;
    }


    double Controller::PID(double error, double t_Ko, double t_Ki, double t_Kt)
    {
        ROS_INFO("PID Icindeki Hata: [%f]", error);
        i_error += error;
        d_error = error - prev_error;
        prev_error = error;

        return (t_Ko * error) + (t_Ki * i_error) + (t_Kt * d_error);
    }


    void Controller::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
/* 
        ROS_INFO("Simdiki Konum: [%f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y);
        current_heading = tf::getYaw(msg->pose.pose.orientation);
        ROS_INFO("Simdiki Yaw: [%f]", current_heading);
        current_point.pose = msg->pose.pose;
 */

        vehicle_odom = *msg;
        current_point.pose = vehicle_odom.pose.pose;
        current_heading = tf::getYaw(vehicle_odom.pose.pose.orientation);

        ROS_INFO("Simdiki Konum: [%f, %f]", current_point.pose.position.x, current_point.pose.position.y);
        ROS_INFO("Simdiki Yaw: [%f]", current_heading);

        ROS_INFO("Vehicle Frame id: [%s]", vehicle_odom.header.frame_id.c_str());

        ChooseWaypoint();
        ControlOutput();
    }


    void Controller::ControlOutput()
    {
        double steering_angle = PID(UpdateError(target_point.pose, current_point.pose), Ko, Ki, Kt);

        double steering_angle_degree = steering_angle * (180 / M_PI);

        if (steering_angle_degree > 100) {

            steering_angle = 100 * (M_PI / 180);

        }

        if (steering_angle_degree < -100) {

            steering_angle = -100 * (M_PI / 180);

        }

        ROS_INFO("Donus Acisi: [%f]", steering_angle);

        autoware_msgs::VehicleCmd control_msg;
        control_msg.twist_cmd.twist.angular.z = steering_angle;
        control_msg.twist_cmd.twist.linear.x = velocity;
        control_pub.publish(control_msg);
    }


    void Controller::PathCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose_stamped;

        pose_stamped.pose = msg->pose.pose;
        pose_stamped.header = msg->header;

        path.poses.push_back(pose_stamped);
        path.header = msg->header;
        path_pub.publish(path);
    }


    void Controller::ChooseWaypoint()
    {
        wp_index = ctrl_index;
        geometry_msgs::PoseStamped waypoint = path.poses[wp_index];
        double min_distance2 = std::pow((waypoint.pose.position.x - current_point.pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point.pose.position.y), 2);

        for (int i = 1; i < path.poses.size(); ++i)
        {
            geometry_msgs::PoseStamped& waypoint_ = path.poses[i];
            double distance2 = std::pow((waypoint_.pose.position.x - current_point.pose.position.x), 2) + std::pow((waypoint_.pose.position.y - current_point.pose.position.y), 2);

            if (distance2 < min_distance2)
            {
                wp_index = i + ctrl_index;
                min_distance2 = distance2;
            }
        }

        waypoint = path.poses[wp_index];
        target_point = waypoint;

        ROS_INFO("Secilen Yol Noktasi: [%f, %f]", target_point.pose.position.x, target_point.pose.position.y);
        ROS_INFO("Yol Noktasi Index: [%d]", wp_index);

        visualization_msgs::Marker marker;
        marker.header = target_point.header;
        marker.id = marker_id;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = target_point.pose;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        
        mark_pub.publish(marker);
        marker_id++;
    }


    double Controller::UpdateError(geometry_msgs::Pose& target_point_pose, geometry_msgs::Pose& current_point_pose)
    {

        tf::Vector3 target_vec(target_point_pose.position.x, target_point_pose.position.y, 0.0);
        tf::Vector3 current_vec(current_point_pose.position.x, current_point_pose.position.y, 0.0);

        tf::Transform transform;
        tf::Quaternion q;
        q.setRPY(0, 0, tf::getYaw(current_point_pose.orientation));
        transform.setRotation(q);
        transform.setOrigin(current_vec);

        tf::Vector3 error_vec = transform.inverse() * target_vec;

        double error = atan2(error_vec.y(), error_vec.x());

/* 
        tf::Vector3 target_vec(target_point_pose.position.x, target_point_pose.position.y, 0.0);
        tf::Vector3 current_vec(current_point_pose.position.x, current_point_pose.position.y, 0.0);

        tf::Vector3 error_vec = target_vec - current_vec;

        double goal_heading = atan2(error_vec.y(), error_vec.x());
        double current_heading_ = tf::getYaw(current_point_pose.orientation);

        double error = goal_heading - current_heading_;
 */

        if (std::isnan(error)) return 0;

        if (error > M_PI) 
        {
            error -= 2 * M_PI;
            return error;
        }

        else if (error < -M_PI)
        {
            error += 2 * M_PI;
            return error;
        }

        return error;
    }

}