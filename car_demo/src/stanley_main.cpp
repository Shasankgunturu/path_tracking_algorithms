#include <ros/ros.h>
#include <car_demo/stanley.hpp>
#include <car_demo/tracking_utils.hpp>
#include <prius_msgs/Control.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "stanley");
    ros::NodeHandle nh;
    ros::Publisher controller = nh.advertise<prius_msgs::Control>("/prius", 1);
    std::string inputpath = "/home/shasankgunturu/tracking_ws/src/car_demo/waypoints/waypoints1.txt";
    Waypoints way(inputpath);
    Stanley s(nh);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        prius_msgs::Control c;
        std::vector<double> next_pose;
        next_pose      = way.nextWaypoint(s.x_current, s.y_current, 2);
        s.velocityCalc();
        c.brake        = s.brake;
        c.throttle     = s.throttle;
        c.steer        = s.steerAngle(next_pose[2],next_pose[1],next_pose[0]);
        std::cout << "current point: " << s.x_current << ", " << s.y_current << ", " << s.yaw << std::endl;
        std::cout << "next point: " << next_pose[0] << ", " << next_pose[1] << ", " << next_pose[2] << std::endl;
        std::cout << "movement: " << c.throttle << ", " << c.brake << std::endl;
        std::cout << "steer: " << c.steer << std::endl;
        controller.publish(c);
        ros::spinOnce();
        loop_rate.sleep();
    }
}