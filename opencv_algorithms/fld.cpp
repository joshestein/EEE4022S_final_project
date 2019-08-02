#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>

#define WIN_NAME "FLD"

using namespace cv;
using namespace cv::ximgproc;

static void help() {
    std::cout << std::endl << 
    "Fast Line Detector" << std::endl <<
    "Usage: ./fld <input-image>" << std::endl;
}

void init_window(int &length_thresh, int &aperture_size) {
    namedWindow(WIN_NAME, 0);
    createTrackbar("Length threshold", WIN_NAME, &length_thresh, 50, 0);
    createTrackbar("Aperture size", WIN_NAME, &aperture_size, 6, 0);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        help();
        return -1;
    }

    Mat img = imread(argv[1], 0);
    if (!img.data) {
        std::cerr << "Failed to read image.";
        return -1;
    }

    int length_thresh = 10;
    float dist_thresh = 1.41421356f;
    int canny_thresh_1 = 50;
    int canny_thresh_2 = 50;
    int aperture_size = 3;
    bool merge = false;

    init_window(length_thresh, aperture_size);

    char display_mode = 'n';
    while (display_mode != 'q') {
        Mat clone_img = img.clone();

        // 3 < aperture size < 7 (upper limit restricted by trackbar) 
        if (aperture_size < 3) { 
            aperture_size = 3;
        }
        // aperture size must always be odd
        if ((aperture_size % 2) != 1) {
            aperture_size +=1;
        }
        Ptr<FastLineDetector> fld = createFastLineDetector(length_thresh, dist_thresh, canny_thresh_1, canny_thresh_2, aperture_size, merge);

        // detect and draw lines
        std::vector<Vec4f> lines_fld;
        fld->detect(clone_img, lines_fld);
        fld->drawSegments(clone_img, lines_fld);

        imshow("FLD", clone_img);
        display_mode = (char)waitKey(0);
    }

    return 0;
}