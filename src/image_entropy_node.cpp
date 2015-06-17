
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
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                int event = frame.at<int>(j, i) / 10;
                occs[i][j][event]++;
            }
        }

        imshow("edges", edges);
    }
    return 0;

}
