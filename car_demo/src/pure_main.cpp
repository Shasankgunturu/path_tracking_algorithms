#include <ros/ros.h>
#include <car_demo/purepursuit.hpp>
#include <car_demo/tracking_utils.hpp>
#include <prius_msgs/Control.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "purepursuit");
    ros::NodeHandle nh;
    ros::Publisher controller = nh.advertise<prius_msgs::Control>("/prius", 1);
    std::string inputpath = "/home/shasankgunturu/tracking_ws/src/car_demo/waypoints/waypoints2.txt";
    Waypoints way(inputpath);
    Pure p(nh);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        prius_msgs::Control c;
        std::vector<double> next_pose;
        next_pose      = way.nextWaypoint(p.x_current, p.y_current, p.ld);
        p.velocityCalc();
        c.brake     = p.brake;
        c.throttle  = p.throttle;
        std::cout << "current point: " << p.x_current << ", " << p.y_current << std::endl;
        std::cout << "next point: " << next_pose[0] << ", " << next_pose[1] << std::endl;
        c.steer     = p.steerAngle(next_pose[0], next_pose[1]);
        controller.publish(c);
        ros::spinOnce();
        loop_rate.sleep();
        }
}