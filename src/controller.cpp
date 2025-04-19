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
        if (!nh_.getParam("pid_katsayilari/Kp", Kp)) return false;
        if (!nh_.getParam("pid_katsayilari/Ki", Ki)) return false;
        if (!nh_.getParam("pid_katsayilari/Kd", Kd)) return false;
        if (!nh_.getParam("ctrl_index", ctrl_index)) return false;
        if (!nh_.getParam("axle_length", axle_length)) return false;

        ROS_INFO("PID Katsayilari: [%f, %f, %f]", Kp, Ki, Kd);
        ROS_INFO("Control Index: [%d]", ctrl_index);
        ROS_INFO("Sase Uzunlugu: [%f]", axle_length);

        return true;
    }


    double Controller::PID(double error, double t_Kp, double t_Ki, double t_Kd)
    {
        i_error += error;
        d_error = error - prev_error;
        prev_error = error;

        return (t_Kp * error) + (t_Ki * i_error) + (t_Kd * d_error);
    }


    void Controller::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {

        vehicle_odom = *msg;
        current_heading = tf::getYaw(vehicle_odom.pose.pose.orientation);
        vehicle_odom.pose.pose.position.x += cos(current_heading)*axle_length/2;
        vehicle_odom.pose.pose.position.y += sin(current_heading)*axle_length/2;

        ROS_INFO("Simdiki Konum: [%f, %f]", vehicle_odom.pose.pose.position.x, vehicle_odom.pose.pose.position.y);
        ROS_INFO("Simdiki Yaw: [%f]", current_heading);

        if (path.poses.size() != 0)
        {
        ChooseWaypoint();
        ControlOutput();
        }
    }


    void Controller::ControlOutput()
    {
        double steering_angle = PID(UpdateError(target_point.pose, vehicle_odom.pose.pose), Kp, Ki, Kd);

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
        double min_distance2 = std::pow((waypoint.pose.position.x - vehicle_odom.pose.pose.position.x), 2) + std::pow((waypoint.pose.position.y - vehicle_odom.pose.pose.position.y), 2);

        for (int i = 1; i < path.poses.size(); ++i)
        {
            geometry_msgs::PoseStamped& waypoint_ = path.poses[i];
            double distance2 = std::pow((waypoint_.pose.position.x - vehicle_odom.pose.pose.position.x), 2) + std::pow((waypoint_.pose.position.y - vehicle_odom.pose.pose.position.y), 2);

            if (distance2 < min_distance2)
            {
                wp_index = i + ctrl_index;
                min_distance2 = distance2;
            }
        }

        waypoint = path.poses[wp_index];
        target_point = waypoint;

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

        double transformed_vec[3] = {0, 0, 0};
        LocalTransform(target_point_pose, current_point_pose, transformed_vec);

        double error = atan2(transformed_vec[1], transformed_vec[0]);

        if (std::isnan(error)) return 0;

        if (error > M_PI) 
        {
            error -= 2 * M_PI;
        }

        else if (error < -M_PI)
        {
            error += 2 * M_PI;
        }

        return error;
    }


    void Controller::LocalTransform(geometry_msgs::Pose& target_point_pose, geometry_msgs::Pose& current_point_pose, double transformed_vector[3])
    {

//Bileşke Matrisli Kod:
        
        double tx = current_point_pose.position.x;
        double ty = current_point_pose.position.y;

        double target_vec[3] = {target_point_pose.position.x, target_point_pose.position.y, 1};

        double current_heading_ = tf::getYaw(current_point_pose.orientation);
        double TransformationMatrix[3][3] = {
            {cos(-current_heading_), -sin(-current_heading_), cos(-current_heading_) * -tx - sin(-current_heading_) * -ty},
            {sin(-current_heading_), cos(-current_heading_), sin(-current_heading_) * -tx + cos(-current_heading_) * -ty},
            {0.0, 0.0, 1.0}
        };

        for (int i=0; i<3; i++) {
        
            transformed_vector[i] = 0;

            for (int j=0; j<3; j++) {

                transformed_vector[i] += TransformationMatrix[i][j] * target_vec[j];

           }
        }

//Ayrık Matrisli Kod:
/* 
        double tx = -current_point_pose.position.x;
        double ty = -current_point_pose.position.y;
        double current_heading_ = tf::getYaw(current_point_pose.orientation);
        double target_vector[3] = {target_point_pose.position.x, target_point_pose.position.y, 1};

        double translation_matrix[3][3] = {
            {1, 0, tx},
            {0, 1, ty},
            {0, 0, 1}
        };

        double rotation_matrix[3][3] = {
            {cos(current_heading_), sin(current_heading_), 0},
            {-sin(current_heading_), cos(current_heading_), 0},
            {0.0, 0.0, 1.0}
        };

        double translated_vector[3];

        for (int i=0; i<3; i++) {
            translated_vector[i] = 0;
            for (int j=0; j<3; j++) {

                translated_vector[i] += translation_matrix[i][j] * target_vector[j];

            }
        }

        for (int i=0; i<3; i++) {
            transformed_vector[i] = 0;
            for (int j=0; j<3; j++) {

                transformed_vector[i] += rotation_matrix[i][j] * translated_vector[j];
    
            }
        }
 */


//Matrissiz Kod:
/* 
        double translated_x = target_point_pose.position.x - current_point_pose.position.x;
        double translated_y = target_point_pose.position.y - current_point_pose.position.y;
        double current_heading_ = tf::getYaw(current_point_pose.orientation);

        transformed_vector[0] = translated_x * cos(current_heading_) + translated_y * sin(current_heading_);
        transformed_vector[1] = translated_x * -sin(current_heading_) + translated_y * cos(current_heading_);
        transformed_vector[2] = 1;
 */


//tf Kütüphaneli Kod:
/* 
        tf::Vector3 target_vec(target_point_pose.position.x, target_point_pose.position.y, 0.0);
        tf::Vector3 current_vec(current_point_pose.position.x, current_point_pose.position.y, 0.0);

        tf::Transform transform;
        tf::Quaternion q;
        q.setRPY(0, 0, tf::getYaw(current_point_pose.orientation));
        transform.setRotation(q);
        transform.setOrigin(current_vec);

        tf::Vector3 error_vec = transform.inverse() * target_vec;

        transformed_vector[0] = error_vec.x();
        transformed_vector[1] = error_vec.y();
        transformed_vector[2] = 1;
 */

    }

}