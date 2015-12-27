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

Scalar color = Scalar(0,255,255);
Scalar yellow = Scalar(0,255,255);
Scalar green = Scalar(0,255,0);
Scalar red = Scalar(0,0,255);
Scalar blue = Scalar(255,0,0);
Scalar purple(184, 114, 216);
Scalar ade004(173,224,4);
Scalar lightGREEN(102,255,102);
Scalar lightORANGE(153,204,255);
Scalar lightBLUE(255,204,153);

extern Mat diff;

extern bool verbose;

/* Global variables */
Mat src, erosion_dst, dilation_dst;
Mat cframe, dst;
Mat img;
Mat templ;
Mat result;


//extern Size size;

extern VideoCapture cap;

String VIDEOPATH = "/Users/drifter/Desktop/capstone/ver.mp4";
const String BASEFRAME_DIR = "/Users/drifter/Dropbox/Feloh/Customer Detection/baseframe.png";

Scalar WHITE(255,255,255);

//int erosion_elem = 0;
//int erosion_size = 4;
//int dilation_elem = 0;
//int dilation_size = 12;
//int const max_elem = 2;
//int const max_kernel_size = 21;
//RNG rng(12345);

int erosion_type = MORPH_RECT;
int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;
RNG rng(12345);

/* Morphology EX global variables */
int morph_elem = 0;
int morph_size = 0;
int morph_operator = 0;
int const max_operator = 4;
//int const max_elem = 2;
//int const max_kernel_size = 21;

enum {GAUSSIAN, BLUR, MEDIAN};
enum {OBJECT_ITEM, OBJECT_CUSTOMER};

Mat aframe, baseframe;
Mat cameraFeed;

String morph = "Morphology Transformations";

Mat e_element =
getStructuringElement( erosion_type,
                      Size( 2*erosion_size+1, 2*erosion_size+1 ),
                      Point( erosion_size, erosion_size ) );
int dilation_type = MORPH_RECT;
Mat d_element =
getStructuringElement( dilation_type,
                      Size( 2*dilation_size +1, 2*dilation_size+1),
                      Point( dilation_size, dilation_size) );

/** Function Headers (prototypes) */
void Erosion( int, void* );
void Dilation( int, void* );
void Morphology_Operations( int, void* );
void on_trackbar( int, void* );
void CannyThreshold(int, void*);
void MatchingMethod( int, void* );
void mergeOverlappingBoxes(std::vector<cv::Rect> *inputBoxes, cv::Mat &image, std::vector<cv::Rect> *outputBoxes, int METHOD);
void encapsulate_objects( Mat *areaOI, Mat *BFRAME, int METHOD, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE );

//void mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes);

//vector<vector<Point> > contours;
//vector<Vec4i> hierarchy;

/* For slider */
const int alpha_slider_max = 100;
int alpha_slider;
double alpha;
double beta;
Mat src1;
Mat src2;
Mat dstslider;


/* Canny vars */
Mat srcCanny, src_gray;
Mat dstCanny, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratioCanny = 3;
int kernel_size = 3;
char window_name[20] = "Edge Map";




int match_method;
int max_Trackbar = 5;
char image_window[30] = "Source Image";
char result_window[30] = "Result window";

#endif