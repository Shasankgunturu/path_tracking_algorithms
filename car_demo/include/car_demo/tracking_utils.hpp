#include <ros/ros.h>
#include <prius_msgs/Control.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <sstream>

class Waypoints {
    public:
        std::vector<double> x_coordinates;
        std::vector<double> y_coordinates;
        std::vector<double> yaw_values;
        Waypoints(std::string inputpath) {
            std::ifstream inputFile(inputpath);
            if (!inputFile.is_open()) {
                std::cerr << "Error opening file!" << std::endl;
            }
            std::string line;
            while (std::getline(inputFile, line)) {
                std::stringstream ss(line);
                std::string x_str, y_str;
                std::getline(ss, x_str, ',');
                std::getline(ss, y_str);
                double x = stod(x_str);
                double y = stod(y_str);
                x_coordinates.push_back(x);
                y_coordinates.push_back(y);
            }
            inputFile.close();
            for (int i=1; i<x_coordinates.size(); i++) {
                yaw_values.push_back(atan2((y_coordinates[i] - y_coordinates[i-1]), (x_coordinates[i] - x_coordinates[i-1])));
            }
        }
        std::vector<double> nextWaypoint(double x_current, double y_current, float ld) {
            std::vector<double> next_pose;
            // std::cout << "next distance: " << sqrt(pow((x_current - x_coordinates[0]), 2) + pow((y_current - y_coordinates[0]), 2)) << std::endl;
            while (sqrt(pow((x_current - x_coordinates[0]), 2) + pow((y_current - y_coordinates[0]), 2)) < ld) {
                x_coordinates.erase(x_coordinates.begin());
                y_coordinates.erase(y_coordinates.begin());
                yaw_values.erase(yaw_values.begin());
            }
            float dist_min = 999;
            int min_index = 0;
            for (int i=0; i<x_coordinates.size(); i++) {
                float dist = sqrt(pow((x_current - x_coordinates[i]), 2) + pow((y_current - y_coordinates[i]), 2));
                if (dist<dist_min) {
                    dist_min = dist;
                    min_index = i;
                }
            }
            next_pose.push_back(x_coordinates[min_index]);
            next_pose.push_back(y_coordinates[min_index]);
            next_pose.push_back(yaw_values[min_index]);
            // std::cout << "remaining waypoints: " << x_coordinates.size() << std::endl;
            return next_pose;
        }

        // std::vector<double> nextWaypointStanley(double x_current, double y_current, float ld) {
        //     std::vector<double> next_pose, y_diff, distance_diff;
        //     // std::cout << "next distance: " << sqrt(pow((x_current - x_coordinates[0]), 2) + pow((y_current - y_coordinates[0]), 2)) << std::endl;
        //     while (sqrt(pow((x_current - x_coordinates[0]), 2) + pow((y_current - y_coordinates[0]), 2)) < ld) {
        //         x_coordinates.erase(x_coordinates.begin());
        //         y_coordinates.erase(y_coordinates.begin());
        //     }
        //     next_pose.push_back(x_coordinates[0]);
        //     next_pose.push_back(y_coordinates[0]);
        //     // std::cout << "remaining waypoints: " << x_coordinates.size() << std::endl;
        //     return next_pose;
        // }
};