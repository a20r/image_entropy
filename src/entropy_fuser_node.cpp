
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

vector<string> pc_topics;
string ec_topic;
double learning_rate;
ros::Publisher ec_pub;
vector<ros::Subscriber> pc_subs;

void pc_callback(sensor_msgs::PointCloud pc) {

}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "entropy_fuser_node");
    ros::NodeHandle n;

    ros::param::get("~point_cloud_topics", pc_topics);
    ros::param::get("~entropy_cloud_topic", ec_topic);
    ros::param::get("~learning_rate", learning_rate);
    ec_pub = n.advertise<sensor_msgs::PointCloud>(ec_topic, queue_size);
    for (int i = 0; i < pc_topics.size(); i++) {
        n.subscribe(pc_topics[i], queue_size, pc_callback);
    }
    ros::spin();
    return 0;
}
