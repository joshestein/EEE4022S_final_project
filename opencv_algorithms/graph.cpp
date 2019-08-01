/*
 * Algorithm:
 * Pedro F Felzenszwalb and Daniel P Huttenlocher. 
 * Efficient graph-based image segmentation.
 */

#include "opencv2/ximgproc/segmentation.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace cv::ximgproc::segmentation;

Scalar hsv_to_rgb(Scalar &&c);
Scalar color_mapping(uint &segment_id);
void help() {
    std::cout << std::endl <<
    "Please run with the following parameters:" << std::endl <<
    ".gs input_image <sigma-val=0.5> <k-val=300> <min-size=100>" << std::endl;
}

Scalar hsv_to_rgb(Scalar &&c) {
    Mat in(1, 1, CV_32FC3); // 32 bit float with 3 channels

    float *p = in.ptr<float>(0);

    p[0] = (float)c[0] * 360.0f;
    p[1] = (float)c[1];
    p[2] = (float)c[2];

    cvtColor(in, in, COLOR_HSV2RGB);

    p[0] *= 255;
    p[1] *= 255;
    p[2] *= 255;

    return Scalar(p[0], p[1], p[2]);

    // Scalar t;
    // // Vec3f = Vector<float, 3>
    // Vec3f p2 = in.at<Vec3f>(0, 0);
    // t[0] = (int)(p2[0] * 255);
    // t[1] = (int)(p2[1] * 255);
    // t[2] = (int)(p2[2] * 255);
    
    // return t;
}

Scalar color_mapping(uint &segment_id) {
    double base = (double)(segment_id) * 0.618033988749895 + 0.24443434;
    return hsv_to_rgb(Scalar(fmod(base, 1.2), 0.95, 0.80));
}

int main(int argc, char** argv) {
    if (argc < 2) {
        help();
        return -1;
    }

    Mat img = imread(argv[1], 0);
    if (!img.data) {
        std::cerr << "Failed to read input image.";
        help();
        return -3;
    }
    cvtColor(img, img, COLOR_BayerGR2BGR);

    // resize image
    int height = 500;
    int width = img.cols*height/img.rows;
    resize(img, img, Size(width, height));

    Ptr<GraphSegmentation> gs = createGraphSegmentation();

    // set sigma - defaults to 0.5
    if (argc > 2) {
        gs->setSigma(std::stod(argv[2]));
    }
    // set K - defaults to 300
    if (argc > 3) {
        gs->setK(std::stof(argv[3]));
    }
    // set min size
    if (argc > 4) {
        gs->setMinSize(std::stoi(argv[4]));
    }

    gs->processImage(img, img);

    double min, max;
    minMaxLoc(img, &min, &max);

    int num_segs = (int)max + 1;
    std::cout << num_segs << " segs" << std::endl;

    Mat out_img = Mat::zeros(img.rows, img.cols, CV_8UC3);
    uint *p;
    uchar *p2;

    for (int i = 0; i < img.rows; i++) {
        p = img.ptr<uint>(i);
        p2 = out_img.ptr<uchar>(i);

        for (int j = 0; j < img.cols; j++) {
            Scalar color = color_mapping(p[j]);
            p2[j*3] = (uchar)color[0];
            p2[j*3+1] = (uchar)color[1];
            p2[j*3+2] = (uchar)color[2];
        }
    }
    imshow("output", out_img);
    waitKey(0);
    return 0;
}