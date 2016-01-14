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
#include <sys/time.h>
#include <unistd.h>
//#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include "inscribe.cpp"
#include "archive.cpp"
#include "Displays.cpp"
#include "Customer.h"

/** namespaces */
using namespace::inscribe;
using namespace::std;
using namespace::cv;


/**
 * @typedef Smooth_tier
 * @brief A list of blur method types
 * @constant GAUSSIAN GaussianBlur method used
 * @constant BLUR normal blur() method used
 * @constant MEDIAN uses medianBlur method, recommended
 * @constant BILATERAL_FILTER heavy computational filter
 */
typedef enum {GAUSSIAN, BLUR, MEDIAN, BILATERAL_FILTER} Smooth_tier;
typedef enum {OBJECT_ITEM, OBJECT_CUSTOMER} Pick_object;
typedef enum {DARK, BRIGHT} Background;
extern Smooth_tier smoothTier;


/** Function Headers (PROTOTYPES) */
class Customer;

/**
 Instantiates newly detected objects
 @function customerList_add
 @param ttcustomer an instance of Customer
 @see overloaded customerList_add(deque<Customer> customers)
 */
template <typename T>
void customerList_add(T ttcustomers);

void linkCustomers(deque<Customer>* current_detected, deque<Customer>* anchor_customer);
deque<Customer> encapsulateObjects(Mat* instanceROI, Mat* baseIMG, Pick_object targetObject/*OBJECT_CUSTOMER*/, int KSIZE, int SIGMA, int THRESH, Smooth_tier SMOOTHTYPE/*MEDIAN*/);

/**
 Merges overlapping boxes in order to show only one box per object detected
 @function mergeOverlappingBoxes
 @param inputBoxes is an array of all the boxes found in frame
 @param image is pointer to matrix of area of interest (ROI)
 @param outputBoxes will hold the new set of boxes to be printed on image
 @param MOCI the method which indicates what object is being detected. i.e. if OBJ_CUSTOMER we set minimun area of rectangle higher then if OBJ_ITEM
 @see overloaded customerList_add(deque<Customer> customers)
 @return number of outPut boxes, likely to be decreased compared to inputBoxes
 */
int mergeOverlappingBoxes(std::vector<cv::Rect> *inputBoxes, cv::Mat &image, std::vector<cv::Rect> *outputBoxes, int METHOD, vector<Point2f>center);

/**
 @discussion checks given ROI from frame if there is an object or item. Uses basic substraction.
 @function isObjectPresent
 @param arearoi the current frame to be evaluated
 @param baseroi the base frame region of interest to compare arearoi to
 @param passes a title to show window of the changes done to the image
 @param illumination of the background
 @see isObjectPresent overloaded
 @return true if there is an object false otherwise
 */
bool isObjectPresent(Mat* arearoi, Mat* baseroi, char* header, Background illumination, bool drawrect);

/**
 @discussion overloaded of isObjectPresent
 @function isObjectPresent
 @param arearoi the current frame to be evaluated
 @param baseroi the base frame region of interest to compare arearoi to
 @param lighting of the background
 @see isObjectPresent overloaded
 @return true if there is an object false otherwise
 */
bool isObjectPresent(Mat* arearoi, Mat* baseroi, Background lighting);

/**
 @discussion counts the number of swipes the cashier performs
 @function countSwipes
 @param foundObj if there is an object in current frame
 @param disp matrix where we display count
 @see isObjectPresent gets foundObj value
 */
void countSwipes(bool foundObj, Mat* disp);

/**
 Draws line as the object is moving
 @function CustomerOpticalFlow
 @param noObjects_TDOF number of objects to be tracked in mask. TDOF(ToDisplayOpticalFlow)
 @see where is called, in function encapsulateObjects
 @see OPTFLOW_ON switch at top
 */
void CustomerOpticalFlow(int noObjects_TDOF);

void castBars();


extern deque<Customer> track_customer;


//Scalar Scalar2(int,int,int);
//void putLabel(Mat img, char* labelText, Point startPoint, double letters, Scalar ground_color);
//void putLabel(Mat img, char* labelText, Point startPoint, int letters);

extern int sigma;
//extern int smoothType;
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
extern int OPTFLOW_ON;
extern int INSTANT_DISPLACEMENT_TOLERANCE;
extern const int CART_DETECTED_AT_START;
extern const int OBJ_CREATION_LINE;
extern const int OBJ_DELETION_LINE;
extern int SHOW_EDGES;
extern int SHOW_DIFF;

extern Rect ROI_CUSTOMERLINE_NARROW;
extern Rect ROI_CUSTOMERLINE_WIDE;
extern Rect ROI_CONVEYOR_BELT;
extern Rect ROI_CUSTOMERLINE;

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
