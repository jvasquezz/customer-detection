#ifndef DEPENDENCIES_HPP
#define DEPENDENCIES_HPP

#include "/Users/drifter/Dropbox/Feloh/FelohDependencies/FelohDependencies.h"
#include <math.h>
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ctype.h>
#include <stdio.h>
#include <iostream>

/** Function Headers (PROTOTYPES) */
void encapsulate_objects( Mat *areaOI, Mat *BFRAME, int METHOD, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE );
void mergeOverlappingBoxes(std::vector<cv::Rect> *inputBoxes, cv::Mat &image, std::vector<cv::Rect> *outputBoxes, int METHOD);

enum {GAUSSIAN, BLUR, MEDIAN};
enum {OBJECT_ITEM, OBJECT_CUSTOMER};

extern int sigma;
extern int smoothType;
extern int ksize;

extern VideoCapture cap;

extern Mat diff;
extern int thresh;
extern bool verbose;

extern Rect MOLD_CUSTOMERLINE_WIDE;
extern Rect MOLD_CUSTOMERLINE;
extern Rect MOLD_CONVEYOR_BELT;

extern Mat frame, baseframe, line_print, belt_print, customer_line;
extern Mat smoothed, laplace, result;

String VIDEOPATH = "/Users/drifter/Desktop/capstone/ver.mp4";
const String BASEFRAME_DIR = "/Users/drifter/Dropbox/Feloh/Customer Detection/baseframe.png";

/**  Scalar COLORS */
Scalar yellow = Scalar(0,255,255);
Scalar green = Scalar(0,255,0);
Scalar red = Scalar(0,0,255);
Scalar blue = Scalar(255,0,0);
Scalar purple(184, 114, 216);
Scalar ade004(173,224,4);
Scalar lightGREEN(102,255,102);
Scalar lightORANGE(153,204,255);
Scalar lightBLUE(255,204,153);
Scalar WHITE(255,255,255);

#endif
