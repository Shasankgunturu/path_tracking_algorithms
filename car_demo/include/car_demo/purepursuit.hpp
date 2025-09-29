#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tf/transform_datatypes.h>

class Pure {
    public:
    float                           throttle, brake, x_current = 2.96542, y_current = -11.9999, ld=3.5;
        Pure(ros::NodeHandle nh) {
            pose                    = nh.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 1, &Pure::poseCallback, this);
        }

        float steerAngle(double x_next, double y_next) {
            float alpha             = calcAlpha(y_next, x_next);
            float steeringAngle     = atan2((2*L*sin(alpha)), ld);
            std::cout << "steering angle: " << steeringAngle << std::endl;
            return steeringAngle;
        }

        void velocityCalc() {
            float vel = max_throttle - v_current;
            if (vel>max_throttle) {
                throttle            = 0.0;
                brake               = sqrt(vel*vel) * kd;
            }
            else {                
                brake               = 0.0;
                throttle            = (vel) * kd;
            }
        }
    private:
        float                       max_throttle=3;
        float                       steeringAngle;
        float                       v_current;
        float                       kd=2, L=3;
        double                      roll, pitch, yaw=0;
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
            v_current               = sqrt(vx_current*vx_current + vy_current*vy_current);
        }
        float calcAlpha(float y_next, float x_next) {
            float alpha             = -atan2((y_current - y_next), (x_current - x_next)) + std::round(yaw * 10.0) / 10.0;
            return alpha;
        }
};