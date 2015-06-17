
#include "ros/ros.h"
#include <ros/console.h>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

const int width = 640;
const int height = 480;
const int num_events = 255 / 10 + 1;
int occs [width][height][num_events];

void initialize_occs(int occs[width][height][num_events]) {
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            for (int k = 0; k < num_events; k++) {
                occs[i][j][k] = 0;
            }
        }
    }
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "image_entropy_node");
    ros::NodeHandle n;

    initialize_occs(occs);
    VideoCapture cap(0);

    if(!cap.isOpened()) {
        return -1;
    }

    Mat edges;
    Mat frame;
    namedWindow("Entropy", 1);
    ROS_DEBUG("%d", num_events);
    while (waitKey(1) < 0) {
        cap >> frame;
        cvtColor(frame, frame, CV_BGR2GRAY);
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                Scalar intensity = frame.at<uchar>(j, i);
                int event = intensity.val[0] / 10;
                occs[i][j][event]++;
            }
        }
        imshow("Entropy", frame);
    }
    return 0;

}
