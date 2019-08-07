// See opencv_contrib/modules/ximgproc/samples/selectivesearchsegmentation_demo.cpp

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc/segmentation.hpp"
#include <iostream>
#include <ctime>

using namespace cv;
using namespace cv::ximgproc::segmentation;

void help() {
    std::cout << std::endl <<
    "Selective search: " << std::endl <<
    "The final true[1]/false[0] value represents whether the input image is Bayer encoded." << std::endl <<
    "./selective_search input_image (s|[f]|q) true[1]/false[0]" << std::endl;
}

void draw_rects(Mat &img, const int &num_rects, std::vector<Rect> &rects);

int main(int argc, char** argv) {
    Mat img;
    if (argc < 2) {
        help();
    }

    if (std::stoi(argv[3]) == 1) {
        // Bayer
        img = imread(argv[1], 0);
        cvtColor(img, img, COLOR_BayerGR2BGR);
    } else {
        img = imread(argv[1]);
    }
    
    if (!img.data) {
        std::cerr << "Failed to load input image" << std::endl;
        return -3;
    }

    // resize image
    int height = 500;
    int width = img.cols*height/img.rows;
    resize(img, img, Size(width, height));


    Ptr<SelectiveSearchSegmentation> ss = createSelectiveSearchSegmentation();
    ss->setBaseImage(img);

    /*
     * Sets quality - defaults to fast
     * (C)olour, (T)exture, (S)ize, (F)ill
     * Single: HSV - C+T+S+F
     * Fast:   HSV - C+T+S+F, 
     *         Lab - T+S+F
     * Quality: HSV - C+T+S+F,
     *          Lab - T+S+F,
     *          rgI (RGB + intensity) - F,
     *          Intensity (Grayscale)   - S 
     * See original paper for more details.
     */
    if (argv[2][0] == 's') {
        ss->switchToSingleStrategy();
    } else if (argv[2][0] == 'q') {
        ss->switchToSelectiveSearchQuality();
    } else {
        ss->switchToSelectiveSearchFast();
    }

    std::vector<Rect> rects;
    ss->process(rects);
    int num_rects = 10;

    std::cout << "Num regions: " << rects.size() << std::endl;
    std::cout << "Enter 'm'/'l' for more/less rects, 'q' to quit.\n" ;

    char c = (char)waitKey();
    while (c != 'q') {
        // keep original img for proper updating rects
        Mat clone_img = img.clone();

        draw_rects(clone_img, num_rects, rects);

        c = (char)waitKey();
        if (c == 'm') {
            num_rects += 10;
        }
        if (c == 'l' && num_rects > 10) {
            num_rects -= 10;
        }
        
    }
    return 0;
}

void draw_rects(Mat &img, const int &num_rects, std::vector<Rect> &rects) {
    int rects_drawn = 0;
    for (auto it = rects.begin(); it != rects.end(); ++it) {
        if (rects_drawn++ < num_rects) {
            rectangle(img, *it, Scalar(0, 0, 255));
        }
    }
    imshow("Selective Search", img);
}
