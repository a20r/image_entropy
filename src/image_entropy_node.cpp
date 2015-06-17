
#include "ros/ros.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

const int width = 640;
const int height = 480;
const int num_events = 255 / 10 + 1;
int occs [width][height][num_events];

int main(int argc, char *argv[]) {

    VideoCapture cap(0);
    if(!cap.isOpened()) {
        return -1;
    }

    Mat edges;
    Mat frame;
    namedWindow("edges", 1);
    while (waitKey(1) < 0) {
        cap >> frame;
        cvtColor(frame, edges, CV_BGR2GRAY);
        imshow("edges", edges);
    }
    return 0;

}
