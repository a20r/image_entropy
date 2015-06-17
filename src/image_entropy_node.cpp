
#include "ros/ros.h"
#include "ros/console.h"
#include "opencv2/opencv.hpp"
#include "image_entropy_node.hpp"
#include "tf/tf.h"
#include <cmath>

using namespace cv;
using namespace std;

int occs [width][height][num_events];
double entropy_grid[width][height] = {1};

void initialize_occs(int occs[width][height][num_events]) {
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            for (int k = 0; k < num_events; k++) {
                occs[i][j][k] = 0;
            }
        }
    }
}

double calculate_entropy(int occs[width][height][num_events], int x, int y) {
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

int sum_row(int occs[width][height][num_events], int x, int y) {

    int sum = 0;
    for (int i = 0; i < num_events; i++) {
        sum += occs[x][y][i];
    }

    return sum;
}

void reset_occs(int occs[width][height][num_events], int x, int y) {
    for (int i = 0; i < num_events; i++) {
        occs[x][y][i] = 0;
    }
}

void develop_entropy_image(double eg[width][height], Mat *dst) {
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            int h = 128 - eg[i][j] * 33;
            if (h < 0) {
                h = 0;
            }
            dst->at<Vec3b>(j, i) = Vec3b(h, 255, 255);
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

    Mat frame;
    Mat e_surf;
    cap >> e_surf;
    namedWindow("Entropy", 1);

    while (waitKey(1) < 0) {
        cap >> frame;
        cvtColor(frame, frame, CV_BGR2GRAY);
        resize(frame, frame, Size(width, height));
        resize(e_surf, e_surf, Size(width, height));
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
            }
        }

        cvtColor(e_surf, e_surf, CV_BGR2HSV);
        develop_entropy_image(entropy_grid, &e_surf);
        cvtColor(e_surf, e_surf, CV_HSV2BGR);
        resize(e_surf, e_surf, Size(640, 480));
        imshow("Entropy", e_surf);
    }
    return 0;

}
