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
#include <array>
#include "inscribe.cpp"

/** namespaces */
using namespace::inscribe;

/*!
 * @typedef BLUR_METHOD
 * @brief A list of blur method types
 * @constant GAUSSIAN GaussianBlur method used
 * @constant BLUR normal blur() method used
 * @constant MEDIAN uses medianBlur method, recommended
 * @constant BILATERAL_FILTER heavy computational filter
 */
typedef enum {GAUSSIAN, BLUR, MEDIAN, BILATERAL_FILTER} SMOOTHTYPE;
typedef enum {OBJECT_ITEM, OBJECT_CUSTOMER} OBJ_RECOGNIZE;

/** Function Headers (PROTOTYPES) */
class Customer;
void customerList_add( Customer ttcustomer);
unsigned int customerList_add(deque<Customer> ttcustomers);
void linkCustomers(deque<Customer>* current_detected, deque<Customer>* anchor_customer);
deque<Customer> encapsulateObjects(Mat* instanceROI, Mat* baseIMG, int targetObject/*OBJECT_CUSTOMER*/, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE/*MEDIAN*/);
int mergeOverlappingBoxes(std::vector<cv::Rect> *inputBoxes, cv::Mat &image, std::vector<cv::Rect> *outputBoxes, int METHOD);
void CustomerOpticalFlow(int noObjects_TDOF);


extern deque<Customer> track_customer;


//Scalar Scalar2(int,int,int);
//void putLabel(Mat img, char* labelText, Point startPoint, double letters, Scalar ground_color);
//void putLabel(Mat img, char* labelText, Point startPoint, int letters);

extern int sigma;
extern int smoothType;
extern int ksize;

extern VideoCapture cap;
extern String capstone_dir;

extern Mat diff;
extern int thresh;
extern bool verbose;

extern bool verbose;
extern bool verbose2;
extern bool verbose_linkingCustomers;
extern bool flag;
extern bool OPTFLOW_ON;
extern const int INSTANT_DISPLACEMENT_TOLERANCE;
extern const int CART_DETECTED_AT_START;
extern const int OBJ_CREATION_LINE;
extern const int OBJ_DELETION_LINE;
extern bool SHOW_EDGES;
extern bool SHOW_DIFF;

extern Rect MOLD_CUSTOMERLINE_WIDE;
extern Rect MOLD_CUSTOMERLINE;
extern Rect MOLD_CONVEYOR_BELT;

extern Mat frame, baseframe, line_print, belt_print, customer_line;
extern Mat smoothed, laplace, result;

const String VIDEOPATH = "/Users/drifter/Desktop/capstone/ver.mp4";
const String BASEFRAME_DIR = "/Users/drifter/Dropbox/Feloh/Customer Detection/baseframe.png";


/**  Scalar COLORS @constructor Scalar(B,G,R) */
Scalar paint_sea(204,102,0);
Scalar paint_salmon(114,128,250);
Scalar paint_maroon(0,0,128);
Scalar paint_dark_red = Scalar(34,34,178);
Scalar paint_royal_orange(65,105,255);
Scalar paint_royal_blue(255,105,65);
Scalar paint_indigo(130,0,75);
Scalar paint_yellow = Scalar(0,255,255);
Scalar paint_green = Scalar(0,255,0);
Scalar paint_red = Scalar(0,0,255);
Scalar paint_blue = Scalar(255,0,0);
Scalar paint_pink(184, 114, 216);
Scalar paint_purple(216, 114, 184);
Scalar paint_ade004(173,224,4);
Scalar paint_lightGREEN(102,255,102);
Scalar paint_lightORANGE(153,204,255);
Scalar paint_lightBLUE(255,204,153);
Scalar WHITE(255,255,255);
extern Scalar stain[10];

#endif
