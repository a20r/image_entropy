
#include <iostream>
#include <stdlib.h>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "topo_maps/Paths.h"
#include "evg-thin/fileio.hpp"
#include "topo_maps/topo_send_image.hpp"

#define QUEUE_SIZE 1

using namespace std;

string fixed_frame;
double resolution;

// converts a stupid evg occupancy grid to a ros occupancy grid in nav_msgs
nav_msgs::OccupancyGrid convert_to_ros(EvgThin::OccupancyGrid grid) {
    nav_msgs::OccupancyGrid ret_grid;
    ret_grid.header.frame_id = fixed_frame;
    ret_grid.info.width = grid.size();
    ret_grid.info.height = grid[0].size();
    ret_grid.info.resolution = resolution;
    for (int i = 0; i < grid[0].size();  i++) {
        for (int j = 0; j < grid.size(); j++) {
            if (grid[j][i] == EvgThin::Free) {
                ret_grid.data.push_back(0);
            } else if (grid[j][i] == EvgThin::Occupied) {
                ret_grid.data.push_back(100);
            } else {
                ret_grid.data.push_back(-1);
            }
        }
    }

    return ret_grid;
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "topo_maps_send_image");
    ros::NodeHandle n;

    // initializes the parameters for the test
    string image_file, occupancy_topic;
    int unknown_min;
    int unknown_max;
    int current_x, current_y, goal_x, goal_y;
    ros::param::get("~image_file", image_file);
    ros::param::get("~unknown_min", unknown_min);
    ros::param::get("~unknown_max", unknown_max);
    ros::param::get("~fixed_frame", fixed_frame);
    ros::param::get("~resolution", resolution);
    ros::param::get("~occupancy_topic", occupancy_topic);

    // get the grid for the test occupancy map
    EvgThin::fileio IO;
    EvgThin::OccupancyGrid grid = IO.read_file(image_file.c_str(), unknown_min,
        unknown_max);
    nav_msgs::OccupancyGrid og = convert_to_ros(grid);


    // creates the publishers used to interact with the ros node
    ros::Publisher grid_pub = n.advertise<nav_msgs::OccupancyGrid>(
        occupancy_topic, QUEUE_SIZE);

    // regulates the speed of the publishers
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        grid_pub.publish(og);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
