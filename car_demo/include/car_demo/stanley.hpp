#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tf/transform_datatypes.h>

class Stanley {
    public:
    float                           throttle, brake, x_current = 2.96542, y_current = -11.9999;
    double                          yaw=0;
        Stanley(ros::NodeHandle nh) {
            pose                    = nh.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 1, &Stanley::poseCallback, this);
        }
        void velocityCalc() {
            float vel = max_throttle - sqrt(v_current*v_current);
            // if (vel>max_throttle) {
            //     throttle            = 0.0;
            //     brake               = sqrt(vel*vel) * kd;
            // }
            // else {                
                brake               = 0.0;
                throttle            = vel * kd;
            // }
        }
        float steerAngle(float required_yaw, float y_next, float x_next) {
            float headingerror = heading_error(required_yaw);
            float crosserror = crosstrack_error(x_next, y_next);
            std::cout << "heading error: " << headingerror << ", crosstrack error: " << crosserror << std::endl;
            return crosserror + headingerror;
        }
    private:
        float                       max_throttle=3;
        float                       steeringAngle;
        float                       v_current, K=0.0925;
        double                      roll, pitch, kd=2;
        ros::Subscriber             pose;
        void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
            x_current               = msg->pose.pose.position.x;
            y_current               = msg->pose.pose.position.y;
            float vx_current        = msg->twist.twist.linear.x;
            float vy_current        = msg->twist.twist.linear.y;
            float quat_x            = msg->pose.pose.orientation.x;
            float quat_y            = msg->pose.pose.orientation.y;
            float quat_z            = msg->pose.pose.orientation.z;
            float quat_w            = msg->pose.pose.orientation.w;
            tf::Quaternion          quaternion(quat_x, quat_y, quat_z, quat_w);
            tf::Matrix3x3           matrix(quaternion);
            matrix.getRPY           (roll, pitch, yaw);
            v_current               = (vy_current/vx_current)*(sqrt((vx_current*vx_current)/(vy_current*vy_current)))*sqrt(vx_current*vx_current + vy_current*vy_current);
        }
        //
        float heading_error(float required_yaw) {
            return (required_yaw - yaw);
        }
        float crosstrack_error(float x_next, float y_next) {
            if (v_current == 0) {
                return 0;
            }
            else {
                float et = sqrt((y_current - y_next)*(y_current - y_next) + (x_current - x_next)*(x_current - x_next));
                std::cout << "vel: " << v_current <<std::endl;
                // if (std::isnan(gamma)) {
                //     gamma = 0;
                // }
                if (v_current>0) {
                    return atan2((K*et),((v_current)));
                }
                else {
                    return atan2((-1*K*et),(sqrt(v_current*v_current)));
                }
            }
        }
};