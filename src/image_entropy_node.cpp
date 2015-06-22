
#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "opencv2/opencv.hpp"
#include "image_entropy_node.hpp"
#include "tf/tf.h"
#include <cmath>
#include <ctime>

using namespace cv;
using namespace std;

int pc_counter = 0;
int occs [width][height][num_events];
int update_threshold;
double entropy_grid[width][height] = {1};
double learning_rate, h_angle, v_angle;
string pose_topic, pc_topic;
VideoCapture cap;
ros::Subscriber pose_sub;
ros::Publisher pc_pub;
Mat frame, e_surf;

inline void initialize_occs(int occs[width][height][num_events]) {
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            reset_occs(occs, i, j);
        }
    }
}

inline double calculate_entropy(int occs[width][height][num_events],
        int x, int y) {
    double total = sum_row(occs, x, y);
    double entropy = 0;
    double prob;
    for (int i = 0; i < num_events; i++) {
        if (occs[x][y][i] > 0) {
            prob = occs[x][y][i] / total;
            entropy -= prob * log2(prob);
        }
    }
    return entropy;
}

inline int sum_row(int occs[width][height][num_events], int x, int y) {
    int sum = 0;
    for (int i = 0; i < num_events; i++) {
        sum += occs[x][y][i];
    }
    return sum;
}

inline void reset_occs(int occs[width][height][num_events], int x, int y) {
    for (int i = 0; i < num_events; i++) {
        occs[x][y][i] = 0;
    }
}

inline geometry_msgs::Point32 transform_pixel(int px, int py,
        geometry_msgs::PoseStamped pose) {

    geometry_msgs::Point32 pos;
    double x = pose.pose.position.z * tan(px * h_angle / width - h_angle / 2);
    double y = pose.pose.position.z * tan((height - py) * v_angle / height
            - v_angle / 2);
    double beta = -2 * acos(pose.pose.orientation.w);
    pos.x = x * cos(beta) - y * sin(beta) + pose.pose.position.x;
    pos.y = x * sin(beta) + y * cos(beta) + pose.pose.position.y;
    pos.z = 0;
    return pos;
}

void pose_callback(geometry_msgs::PoseStamped pose) {
    cap >> frame;
    cvtColor(frame, frame, CV_BGR2GRAY);
    resize(frame, frame, Size(width, height));
    sensor_msgs::PointCloud pc;
    pc.header.seq = pc_counter++;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "map";

    sensor_msgs::ChannelFloat32 cf;
    cf.name = "intensity";
    pc.channels.push_back(cf);

    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            Scalar intensity = frame.at<uchar>(j, i);
            int event = intensity.val[0] / interval;
            occs[i][j][event]++;

            if (sum_row(occs, i, j) >= update_threshold) {
                double oe = (1 - learning_rate) * entropy_grid[i][j];
                double ne = learning_rate * calculate_entropy(occs, i, j);
                entropy_grid[i][j] = oe + ne;
                reset_occs(occs, i, j);
            }
            geometry_msgs::Point32 pos = transform_pixel(i, j, pose);
            pc.points.push_back(pos);
            pc.channels[0].values.push_back(entropy_grid[i][j]);
        }
    }
    pc_pub.publish(pc);
}

inline double radians(double degree) {
    return degree * (M_PI / 180);
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "image_entropy_node");
    ros::NodeHandle n;
    cap = VideoCapture(0);

    if(!cap.isOpened()) {
        return -1;
    }

    cap >> e_surf;
    resize(e_surf, e_surf, Size(width, height));
    initialize_occs(occs);
    ros::param::get("~pose_topic", pose_topic);
    ros::param::get("~point_cloud_topic", pc_topic);
    ros::param::get("~learning_rate", learning_rate);
    ros::param::get("~update_threshold", update_threshold);
    ros::param::get("~horizontal_angle", h_angle);
    ros::param::get("~vertical_angle", v_angle);
    h_angle = radians(h_angle);
    v_angle = radians(v_angle);
    pose_sub = n.subscribe(pose_topic, queue_size, pose_callback);
    pc_pub = n.advertise<sensor_msgs::PointCloud>(pc_topic, queue_size);
    ros::spin();
    return 0;
}
