
#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "opencv2/opencv.hpp"
#include "image_entropy_node.hpp"
#include "tf/tf.h"
#include <cmath>
#include <sstream>
#include <ctime>
#include <unordered_map>

using namespace cv;
using namespace std;

vector<string> pc_topics;
vector<ros::Subscriber> sub_vec;
string ec_topic;
double learning_rate;
ros::Publisher ec_pub;
sensor_msgs::PointCloud entropy_cloud;
int pc_counter = 0;
map<string, int> point_map;

void pc_callback(sensor_msgs::PointCloud pc) {
    for (int i = 0; i < pc.points.size(); i++) {
        stringstream buffer;
        buffer << pc.points[i];
        string p_string = buffer.str();
        if (point_map.count(p_string) > 0) {
            int j = point_map[p_string];
            entropy_cloud.points[j] = pc.points[i];
            entropy_cloud.channels[0].values[j] = pc.channels[0].values[i];
        } else {
            entropy_cloud.points.push_back(pc.points[i]);
            entropy_cloud.channels[0].values.push_back(
                    pc.channels[0].values[i]);
            point_map[p_string] = entropy_cloud.points.size();
        }
    }

    entropy_cloud.header.seq = pc_counter++;
    entropy_cloud.header.stamp = ros::Time::now();
    ec_pub.publish(entropy_cloud);
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "entropy_fuser_node");
    ros::NodeHandle n;

    ros::param::get("~point_cloud_topics", pc_topics);
    ros::param::get("~entropy_cloud_topic", ec_topic);
    ros::param::get("~learning_rate", learning_rate);

    ec_pub = n.advertise<sensor_msgs::PointCloud>(ec_topic, queue_size);
    for (int i = 0; i < pc_topics.size(); i++) {
        ros::Subscriber sub = n.subscribe(pc_topics[i],
                queue_size, pc_callback);
        sub_vec.push_back(sub);
    }

    sensor_msgs::ChannelFloat32 cf;
    cf.name = "intensity";
    entropy_cloud.header.seq = 0;
    entropy_cloud.header.stamp = ros::Time::now();
    entropy_cloud.header.frame_id = "map";
    entropy_cloud.channels.push_back(cf);
    ros::spin();
    return 0;
}
