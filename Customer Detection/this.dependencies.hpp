#ifndef DEPENDENCIES_HPP
#define DEPENDENCIES_HPP

#include "/Users/drifter/Dropbox/Feloh/FelohDependencies/FelohDependencies.h"
#include "/Users/drifter/Documents/opencv-3.0.0/modules/features2d/include/opencv2/features2d.hpp"
#include "/Users/drifter/Dropbox/Feloh/documents-export-2015-12-01/Software/V1/jacob-ppl-carts/include/opencv2/nonfree/features2d.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <iostream>
#include <ctype.h>
#include <math.h>
#include <stdio.h>

enum {GAUSSIAN, BLUR, MEDIAN, BILATERAL_FILTER};
enum {OBJECT_ITEM, OBJECT_CUSTOMER};

/** Function Headers (PROTOTYPES) */
class Customer;
void customerList_add( Customer ttcustomer);
unsigned int customerList_add(deque<Customer> ttcustomers);
void linkCustomers(deque<Customer>* current_detected, deque<Customer>* anchor_customer);
deque<Customer> encapsulateObjects(Mat* instanceROI, Mat* baseIMG, int targetObject/*OBJECT_CUSTOMER*/, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE/*MEDIAN*/);
int mergeOverlappingBoxes(std::vector<cv::Rect> *inputBoxes, cv::Mat &image, std::vector<cv::Rect> *outputBoxes, int METHOD);

extern deque<Customer> track_customer;

extern int sigma;
extern int smoothType;
extern int ksize;

extern VideoCapture cap;
extern String capstone_dir;

extern Mat diff;
extern int thresh;
extern bool verbose;

extern Rect MOLD_CUSTOMERLINE_WIDE;
extern Rect MOLD_CUSTOMERLINE;
extern Rect MOLD_CONVEYOR_BELT;

extern Mat frame, baseframe, line_print, belt_print, customer_line;
extern Mat smoothed, laplace, result;

const String VIDEOPATH = "/Users/drifter/Desktop/capstone/ver.mp4";
const String BASEFRAME_DIR = "/Users/drifter/Dropbox/Feloh/Customer Detection/baseframe.png";


/**  Scalar COLORS */
Scalar paint_yellow = Scalar(0,255,255);
Scalar paint_green = Scalar(0,255,0);
Scalar paint_red = Scalar(0,0,255);
Scalar paint_blue = Scalar(255,0,0);
Scalar paint_purple(184, 114, 216);
Scalar paint_ade004(173,224,4);
Scalar paint_lightGREEN(102,255,102);
Scalar paint_lightORANGE(153,204,255);
Scalar paint_lightBLUE(255,204,153);
Scalar WHITE(255,255,255);
extern Scalar stain[10];

#endif
