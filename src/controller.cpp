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

        path_sub = nh_.subscribe("/shortest_path", 10, &Controller::PathCallback, this);
        odom_sub = nh_.subscribe("/odom", 10, &Controller::OdomCallback, this);
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
        if (!nh_.getParam("ctrl_max", ctrl_max)) return false;
        if (!nh_.getParam("axle_length", axle_length)) return false;

        ROS_INFO("PID Katsayilari: [%f, %f, %f]", Kp, Ki, Kd);
        ROS_INFO("Control Index: [%d]   Control Max: [%f]", ctrl_index, ctrl_max);
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
        vehicle_odom.pose.pose.position.x += cos(current_heading)*axle_length*0;
        vehicle_odom.pose.pose.position.y += sin(current_heading)*axle_length*0;

        ROS_INFO("Simdiki Konum: [%f, %f]", vehicle_odom.pose.pose.position.x, vehicle_odom.pose.pose.position.y);
        ROS_INFO("Simdiki Yaw: [%f]", current_heading);

        if (path.poses.size() != 0)
        {
            ControlOutput();
        }

        else
        {
            autoware_msgs::VehicleCmd control_msg;
            control_msg.twist_cmd.twist.angular.z = 0;
            control_msg.twist_cmd.twist.linear.x = 0;
            control_pub.publish(control_msg);
        }
    }


    void Controller::ControlOutput()
    {
        geometry_msgs::PoseStamped target_point = ChooseWaypoint(vehicle_odom.pose.pose, path, ctrl_index, ctrl_max);
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


/*     void Controller::PathCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose_stamped;

        pose_stamped.pose = msg->pose.pose;
        pose_stamped.header = msg->header;

        path.poses.push_back(pose_stamped);
        path.header = msg->header;
        path_pub.publish(path);
    } */


    void Controller::PathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        path = *msg;

        for (int i = 0; i < path.poses.size(); ++i)
        {
            if (i+1<path.poses.size())
            {
                while (path.poses[i].pose.position.x == path.poses[i+1].pose.position.x && path.poses[i].pose.position.y == path.poses[i+1].pose.position.y)
                {
                    path.poses.erase(path.poses.begin()+i);
                    if (i+1==path.poses.size()) break;
                }
            }
        }
    }


    geometry_msgs::PoseStamped Controller::ChooseWaypoint(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path, int ctrl_idx, double ctrl_max)
    {
/*         int closest_idx = ClosestWaypointIndex(current_point_pose, t_path); */
        int closest_idx = 0;
        int target_idx = closest_idx + ctrl_idx;

        while (!(target_idx<path.poses.size()))
        {
            target_idx--;
        }

        geometry_msgs::PoseStamped target_point = t_path.poses[target_idx];

        while (std::sqrt(std::pow(target_point.pose.position.x-current_point_pose.position.x, 2) + std::pow(target_point.pose.position.y-current_point_pose.position.y, 2)) > ctrl_max)
        {
            if (target_idx>0)
            {
                target_idx--;
                target_point = t_path.poses[target_idx];
            }

            else
            {
                target_idx = 0;
                target_point = t_path.poses[target_idx];
                break;
            }
        }

        visualization_msgs::Marker marker;
        marker.header = t_path.header;
        marker.id = marker_id;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = target_point.pose;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        
        mark_pub.publish(marker);
/*         marker_id++; */

        return target_point;
    }


    int Controller::ClosestWaypointIndex(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path)
    {
        int closest_point_index;
        double min_distance2;

        for (int i = 0; i < t_path.poses.size(); ++i)
        {
            geometry_msgs::PoseStamped waypoint = t_path.poses[i];

            if (i == 0)
            {
                closest_point_index = i;
                min_distance2 = std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2);
            }

            else 
            {
                double distance2 = std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2);

                if (distance2 < min_distance2)
                {
                    closest_point_index = i;
                    min_distance2 = distance2;
                }
            }
        }

        return closest_point_index;
    }


    double Controller::UpdateError(const geometry_msgs::Pose& target_point_pose, const geometry_msgs::Pose& current_point_pose)
    {
        double transformed_vec[3] = {0, 0, 0};
        LocalTransform(target_point_pose, current_point_pose, transformed_vec);

        double error = std::atan2(transformed_vec[1], transformed_vec[0]);

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


    void Controller::LocalTransform(const geometry_msgs::Pose& target_point_pose, const geometry_msgs::Pose& current_point_pose, double transformed_vector[3])
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