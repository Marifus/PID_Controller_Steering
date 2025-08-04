#include "pid_pkg/controller.hpp"

namespace controller
{

    Controller::Controller(ros::NodeHandle& nh) 
        : nh_(nh), marker_id(0), steering_d_error(0), steering_i_error(0), steering_prev_error(0),velocity_d_error(0), velocity_i_error(0), 
        velocity_prev_error(0), prev_time(0), docking(false), path_received(false)
    {
        if (!ReadParameters())
        {
            ROS_ERROR("Parametreler Okunamadi.");
            ros::requestShutdown();
        }

        path_sub = nh_.subscribe(path_topic, 10, &Controller::PathCallback, this);
        odom_sub = nh_.subscribe(odom_topic, 10, &Controller::OdomCallback, this);
        control_pub = nh_.advertise<autoware_msgs::VehicleCmd>(cmd_topic, 10);
        docking_signal_sub = nh_.subscribe(docking_signal_topic, 10, &Controller::DockingCallback, this);
        mark_pub = nh_.advertise<visualization_msgs::Marker>(marker_topic, 10);

        timer = nh_.createTimer(ros::Duration(herhangi_bir_sey), &Controller::TimerCallback, this);
        //path_pub = nh_.advertise<nav_msgs::Path>("/path", 10);
    }


    bool Controller::ReadParameters()
    {
        if (!nh_.getParam("pid_coefficients/Kp_max", Kp_max)) return false;
        if (!nh_.getParam("pid_coefficients/Ki_max", Ki_max)) return false;
        if (!nh_.getParam("pid_coefficients/Kd_max", Kd_max)) return false;
        if (!nh_.getParam("pid_coefficients/Kp_min", Kp_min)) return false;
        if (!nh_.getParam("pid_coefficients/Ki_min", Ki_min)) return false;
        if (!nh_.getParam("pid_coefficients/Kd_min", Kd_min)) return false;

        if (!nh_.getParam("ctrl/ctrl_index", ctrl_index)) return false;
        if (!nh_.getParam("ctrl/high_ctrl_max", high_ctrl_max)) return false;
        if (!nh_.getParam("ctrl/low_ctrl_max", low_ctrl_max)) return false;

        if (!nh_.getParam("velocity_controller/velocity/max_velocity", max_velocity)) return false;
        if (!nh_.getParam("velocity_controller/velocity/low_velocity", low_velocity)) return false;
        if (!nh_.getParam("velocity_controller/velocity/high_velocity", high_velocity)) return false;
        if (!nh_.getParam("velocity_controller/curvature/curvature_coefficient", curvature_coefficient)) return false;
        if (!nh_.getParam("velocity_controller/curvature/curvature_index", curvature_index)) return false;
        if (!nh_.getParam("velocity_controller/curvature/curvature_lookahead", curvature_lookahead)) return false;
        if (!nh_.getParam("velocity_controller/curvature/max_curvature", max_curvature)) return false;

        if (!nh_.getParam("wheelbase", wheelbase)) return false;
        if (!nh_.getParam("herhangi_bir_sey", herhangi_bir_sey)) return false;

        if (!nh_.getParam("logs/input_log", input_log)) return false;
        if (!nh_.getParam("logs/output_log", output_log)) return false;

        if (!nh_.getParam("subscribe_topics/odom_topic", odom_topic)) return false;
        if (!nh_.getParam("subscribe_topics/path_topic", path_topic)) return false;
        if (!nh_.getParam("subscribe_topics/docking_signal_topic", docking_signal_topic)) return false;
        if (!nh_.getParam("publish_topics/cmd_topic", cmd_topic)) return false;
        if (!nh_.getParam("publish_topics/marker_topic", marker_topic)) return false;

        if (!nh_.getParam("low_pass_filter/filter_length", filter_length)) return false;
        if (!nh_.getParam("low_pass_filter/weight_current", weight_current)) return false;

        ROS_INFO("Max PID Katsayilari: [%f, %f, %f]", Kp_max, Ki_max, Kd_max);
        ROS_INFO("Min PID Katsayilari: [%f, %f, %f]", Kp_min, Ki_min, Kd_min);
        ROS_INFO("Control Index: [%d]   Control Max: [%f, %f]", ctrl_index, low_ctrl_max, high_ctrl_max);
        ROS_INFO("Dingil Mesafesi: [%f]", wheelbase);
        ROS_INFO("Maksimum Arac Hizi: [%f]", max_velocity);
        ROS_INFO("Girdi/Cikti Gosterme: [%d, %d]", input_log, output_log);

        return true;
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
        path_received = true;

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


    void Controller::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        vehicle_odom = *msg;
        current_heading = GetYaw(vehicle_odom.pose.pose.orientation);
        current_velocity = std::hypot(vehicle_odom.twist.twist.linear.x, vehicle_odom.twist.twist.linear.y);
        vehicle_odom.pose.pose.position.x += std::cos(current_heading)*wheelbase*0;
        vehicle_odom.pose.pose.position.y += std::sin(current_heading)*wheelbase*0;

        if(input_log)
        {
            ROS_INFO("Mevcut Konum: [%f, %f]", vehicle_odom.pose.pose.position.x, vehicle_odom.pose.pose.position.y);
            ROS_INFO("Mevcut Yaw: [%f]", current_heading);
            ROS_INFO("Mevcut Hiz [%f]", current_velocity);
        }

        ROS_INFO("Mevcut Hiz [%f]", current_velocity);

        if (path.poses.size() == 0 || docking)
        {            
            autoware_msgs::VehicleCmd control_msg;
            control_msg.twist_cmd.twist.linear.x = 0;
            control_pub.publish(control_msg);
        } else ControlOutput();
    }


    void Controller::DockingCallback(const std_msgs::Bool& msg)
    {
        docking = msg.data;
    }


    void Controller::TimerCallback(const ros::TimerEvent&)
    {
        if(!path_received) path.poses.clear();
        else path_received = false;
    }


    void Controller::ControlOutput()
    {
        UpdatePIDCoefficients(Kp, Ki, Kd, ctrl_max);
        geometry_msgs::PoseStamped target_point = ChooseWaypoint(vehicle_odom.pose.pose, path, ctrl_index, ctrl_max);

        current_time = ros::Time::now().toSec();
        double dt = current_time - prev_time;

        double steering_angle = PID(UpdateSteeringError(target_point.pose, vehicle_odom.pose.pose), Kp, Ki, Kd, dt, velocity_prev_error, velocity_i_error, velocity_d_error);

        prev_time = current_time;

        double steering_angle_degree = steering_angle * (180 / M_PI);

        if (steering_angle_degree > 100) {

            steering_angle = 100 * (M_PI / 180);

        }

        else if (steering_angle_degree < -100) {

            steering_angle = -100 * (M_PI / 180);

        }

        if(output_log)
        {
            ROS_INFO("Donus Acisi: [%f]", steering_angle);
        }


        double velocity_cmd = LowPassFilter(CalculateVelocityCmd(max_velocity, CalculatePathCurv(vehicle_odom.pose.pose, path, curvature_index, curvature_lookahead, max_curvature), curvature_coefficient), prev_inputs_deq, filter_length, weight_current);

        autoware_msgs::VehicleCmd control_msg;
        control_msg.twist_cmd.twist.angular.z = steering_angle;
        control_msg.twist_cmd.twist.linear.x = velocity_cmd;
        control_pub.publish(control_msg);
    }


    double Controller::PID(double error, double t_Kp, double t_Ki, double t_Kd, double dt, double& prev_error, double& i_error, double& d_error)
    {
        i_error += error * dt;
        if (dt != 0) d_error = (error - prev_error) / dt;
        else d_error = 0;
        prev_error = error;

        return (t_Kp * error) + (t_Ki * i_error) + (t_Kd * d_error);
    }


    geometry_msgs::PoseStamped Controller::ChooseWaypoint(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path, int ctrl_idx, double ctrl_max)
    {
/*         int closest_idx = ClosestWaypointIndex(current_point_pose, t_path); */
        int closest_idx = 0;
        int target_idx = closest_idx + ctrl_idx;

        if (!(target_idx<path.poses.size())) 
        {
            target_idx = path.poses.size()-1;
        }

        geometry_msgs::PoseStamped target_point = t_path.poses[target_idx];

        while (std::hypot(target_point.pose.position.x-current_point_pose.position.x, target_point.pose.position.y-current_point_pose.position.y) > ctrl_max)
        {
            if (target_idx>0)
            {
                target_point = t_path.poses[--target_idx];
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


    double Controller::UpdateSteeringError(const geometry_msgs::Pose& target_point_pose, const geometry_msgs::Pose& current_point_pose)
    {
        geometry_msgs::Pose transformed_pose = LocalTransform(current_point_pose, target_point_pose);

        double error = std::atan2(transformed_pose.position.y, transformed_pose.position.x);

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


    geometry_msgs::Pose Controller::LocalTransform(const geometry_msgs::Pose& origin_pose, const geometry_msgs::Pose& target_point_pose)
    {   
        tf2::Transform origin_tf, target_point_tf, g2l_transform, transformed_point_tf;

        tf2::fromMsg(origin_pose, origin_tf);
        g2l_transform = origin_tf.inverse();

        tf2::fromMsg(target_point_pose, target_point_tf);

        transformed_point_tf = g2l_transform * target_point_tf;
        
        geometry_msgs::Pose transformed_pose;
        tf2::toMsg(transformed_point_tf, transformed_pose);

        return transformed_pose;
    }


    double Controller::GetYaw(const geometry_msgs::Quaternion& q)
    {
        return atan2(2.0 * (q.w * q.z + q.x * q.y) , 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }


    double Controller::CalculatePathCurv(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path, int curv_idx, double curv_lookahead, double max_curv)
    {
        while (std::hypot(current_point_pose.position.x - t_path.poses[curv_idx].pose.position.x, 
            current_point_pose.position.y - t_path.poses[curv_idx].pose.position.y) > curv_lookahead)
        {
            if (curv_idx > 0) {
                curv_idx--;
            } else break;
        }

        geometry_msgs::Pose transformed_pose = LocalTransform(current_point_pose, t_path.poses[curv_idx].pose);
        double dx = transformed_pose.position.x;
        double dy = transformed_pose.position.y;

/* 
        double dx = vehicle_odom.pose.pose.position.x - t_path.poses[curv_idx].pose.position.x;
        double dy = vehicle_odom.pose.pose.position.y - t_path.poses[curv_idx].pose.position.y;
 */

        double radius = std::abs((std::pow(dx, 2) + std::pow(dy, 2)) / (2*dy));

/*         
        if (curv_idx < 2) curv_idx = 2;
        int mid_idx = curv_idx / 2;

        double p1[2] {t_path.poses[0].pose.position.x, t_path.poses[0].pose.position.y};
        double p2[2] {t_path.poses[mid_idx].pose.position.x, t_path.poses[mid_idx].pose.position.y};
        double p3[2] {t_path.poses[curv_idx].pose.position.x, t_path.poses[curv_idx].pose.position.y};
        
        double a = std::hypot(p2[0] - p3[0], p2[1] - p3[1]);
        double b = std::hypot(p1[0] - p3[0], p1[1] - p3[1]);
        double c = std::hypot(p1[0] - p2[0], p1[1] - p2[1]);

        double s = (a + b + c) / 2.0;
        double area = std::sqrt(s * (s-a) * (s-b) * (s-c));

        if (area < 1e-6) {
            return 0.0;
        }

        double radius = (a * b * c) / (4.0 * area);
 */

        return ((1.0 / radius) < (max_curv) ? (1.0 / radius) : max_curv);
    }


    double Controller::CalculateVelocityCmd(double velocity, double curvature, double k_curvature)
    {
        if (output_log) {
            ROS_INFO("Path Curvature: [%f]", curvature);   
        }
        return (velocity/(curvature*k_curvature+1));
    }


    double Controller::LowPassFilter(double input, std::deque<double> &prev_inputs, int f_length, double w_current)
    {
        prev_inputs.push_back(input);
        if (prev_inputs.size() > f_length) prev_inputs.pop_front();
        if (prev_inputs.size() < 1) return input;

        double sum {0.0};

        for (int i=0; i<(prev_inputs.size()-1); i++) sum+= prev_inputs[i];

        double w_prev = (1.0 - w_current) / (prev_inputs.size()-1);


        return ((sum * w_prev) + (input * w_current));
    }


    void Controller::UpdatePIDCoefficients(double& t_Kp, double& t_Ki, double& t_Kd, double& t_ctrl_max)
    {
        double low_mid_velocity = low_velocity + (high_velocity-low_velocity)*0.25;
        double high_mid_velocity = high_velocity - (high_velocity-low_velocity)*0.25;

        if (current_velocity<low_mid_velocity) {
            t_Kp = Kp_max;
            t_Ki = Ki_max;
            t_Kd = Kd_max;
            t_ctrl_max = low_ctrl_max;
        } 
        
        else if (current_velocity>high_mid_velocity) {
            t_Kp = Kp_min;
            t_Ki = Ki_min;
            t_Kd = Kd_min;
            t_ctrl_max = high_ctrl_max;
        }
        
        else
        {
            double fark = high_mid_velocity - low_mid_velocity;

            t_Kp = (Kp_max+Kp_min)/2 + (Kp_max-Kp_min)/2*cos(M_PI/(fark)*(current_velocity-low_mid_velocity)/fark);
            t_Ki = (Ki_max+Ki_min)/2 + (Ki_max-Ki_min)/2*cos(M_PI/(fark)*(current_velocity-low_mid_velocity)/fark);
            t_Kd = (Kd_max+Kd_min)/2 + (Kd_max-Kd_min)/2*cos(M_PI/(fark)*(current_velocity-low_mid_velocity)/fark);
            t_ctrl_max = (low_ctrl_max+high_ctrl_max)/2 - (high_ctrl_max-low_ctrl_max)/2*cos(M_PI/(fark)*(current_velocity-low_mid_velocity)/fark);
        }

        if (output_log)
        {
            ROS_INFO("Guncel PID Katsayilari: [%f, %f, %f]", t_Kp, t_Ki, t_Kd);
            ROS_INFO("Guncel Ctrl Max: [%f]", ctrl_max);
        }

            ROS_INFO("Guncel PID Katsayilari: [%f, %f, %f]", t_Kp, t_Ki, t_Kd);
            ROS_INFO("Guncel Ctrl Max: [%f]", ctrl_max);
    }

}