// SLIC superpixels

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"

#include <ctype.h>
#include <iostream>

#define WIN_NAME "Superpixels"

using namespace cv;
using namespace cv::ximgproc;

static void help() {
    std::cout << std::endl <<
    "SLIC Superpixels" << std::endl <<
    "Algorithms: 0 = SLIC, 1 = SLICO, 2 = MSLIC" << std::endl <<
    "Usage: ./superpixels <input-image> [0]" << std::endl;
}

void init_window(int &algorithm, int &region_size, int &ruler, int &min_element_size,
                 int &num_iterations) {

    namedWindow(WIN_NAME, 0);
    createTrackbar("Algorithm", WIN_NAME, &algorithm, 2, 0);
    createTrackbar("Region size", WIN_NAME, &region_size, 200, 0);
    createTrackbar("Ruler", WIN_NAME, &ruler, 100, 0);
    createTrackbar("Connectivity", WIN_NAME, &min_element_size, 100, 0);
    createTrackbar("Iterations", WIN_NAME, &num_iterations, 12, 0);
}

int main(int argc, char** argv) {
    int algorithm, region_size, ruler, min_element_size, num_iterations;
    Mat img, mask;

    if (argc < 2) {
        help();
        return -1;
    }

    // read image and convert to colour
    img = imread(argv[1], 0);
    if (!img.data) {
        std::cerr << "Failed to read input image." << std::endl;
        return -1;
    }
    cvtColor(img, img, COLOR_BayerGR2BGR);

    algorithm = 0;
    region_size = 50;
    ruler = 30.0;
    min_element_size = 50;
    num_iterations = 3;

    if (argc > 2) {
        algorithm = std::stoi(argv[2]);
    }

    init_window(algorithm, region_size, ruler, min_element_size, num_iterations);

    char display_mode = 's';
    char prev_display_mode = display_mode;
    while (display_mode != 'q') {
        Mat frame;
        img.copyTo(frame);
        // cvtColor(frame, frame, COLOR_BGR2HSV);

        Ptr<SuperpixelSLIC> slic = createSuperpixelSLIC(frame, algorithm+SLIC, region_size, (float)ruler);
        slic->iterate(num_iterations);

        if (min_element_size > 0)
            slic->enforceLabelConnectivity(min_element_size);

        std::cout << slic->getNumberOfSuperpixels() << " superpixels generated." << std::endl;

        // generate mask (just superpixels contours, no image pixels)
        slic->getLabelContourMask(mask, true);
        frame.setTo(Scalar(0, 0, 255), mask);

        switch (prev_display_mode) {
            case 's':
                // superpixel contours
                imshow(WIN_NAME, frame);
                prev_display_mode='s';
                break;
            case 'm':
                // mask
                imshow(WIN_NAME, mask);
                prev_display_mode='m';
                break;
        }
        display_mode = (char)waitKey(0);
    }

    return 0;
}