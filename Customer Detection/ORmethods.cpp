/////**  @function MatchingMethod */
////void MatchingMethod( int, void* )
////{
////    /// Source image to display
////    Mat img_display;
////    img.copyTo( img_display );
////    Mat tmp2 = imread("/Users/drifter/Desktop/conveyorBelt.png");
////    Size size(img.cols, img.rows);
////    resize(tmp2, tmp2, size);
////    addWeighted(img, .5, tmp2, .5, 0.0, tmp2);
////    
////    /// Create the result matrix
////    int result_cols =  img.cols - templ.cols + 1;
////    int result_rows = img.rows - templ.rows + 1;
////    
////    result.create( result_rows, result_cols, CV_32FC1 );
////    
////    /// Do the Matching and Normalize
////    matchTemplate( img, templ, result, match_method );
////    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
////    
////    /// Localizing the best match with minMaxLoc
////    double minVal; double maxVal; Point minLoc; Point maxLoc;
////    Point matchLoc;
////    
////    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
////    
////    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
////    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
////    { matchLoc = minLoc; }
////    else
////    { matchLoc = maxLoc; }
////    
////    /// Show me what you got
////    rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
////    rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
////    
////    imshow( image_window, img_display );
////    imshow( result_window, result );
////    
////}
//
//
//
////
////  ORmethods.cpp
////  Customer Detection
////
////  Created by Rurouni on 12/24/15.
////  Copyright © 2015 Rurouni. All rights reserved.
////
//
//#include <stdio.h>
//
////#include <iostream>
////#include <fstream>
////
////#include "opencv2/core.hpp"
////#include "opencv2/core/utility.hpp"
////#include "opencv2/highgui.hpp"
////#include "/Users/drifter/Documents/opencv-3.0.0/modules/cudaoptflow/include/opencv2/cudaoptflow.hpp"
//////#include "opencv2/cudaoptflow.hpp"
////#include "/Users/drifter/Documents/opencv-3.0.0/modules/cudaarithm/include/opencv2/cudaarithm.hpp"
//////#include "opencv2/cudaarithm.hpp"
////
////using namespace std;
////using namespace cv;
////using namespace cv::cuda;
////
////inline bool isFlowCorrect(Point2f u)
////{
////    return !cvIsNaN(u.x) && !cvIsNaN(u.y) && fabs(u.x) < 1e9 && fabs(u.y) < 1e9;
////}
////
////static Vec3b computeColor(float fx, float fy)
////{
////    static bool first = true;
////
////    // relative lengths of color transitions:
////    // these are chosen based on perceptual similarity
////    // (e.g. one can distinguish more shades between red and yellow
////    //  than between yellow and green)
////    const int RY = 15;
////    const int YG = 6;
////    const int GC = 4;
////    const int CB = 11;
////    const int BM = 13;
////    const int MR = 6;
////    const int NCOLS = RY + YG + GC + CB + BM + MR;
////    static Vec3i colorWheel[NCOLS];
////
////    if (first)
////    {
////        int k = 0;
////
////        for (int i = 0; i < RY; ++i, ++k)
////            colorWheel[k] = Vec3i(255, 255 * i / RY, 0);
////
////        for (int i = 0; i < YG; ++i, ++k)
////            colorWheel[k] = Vec3i(255 - 255 * i / YG, 255, 0);
////
////        for (int i = 0; i < GC; ++i, ++k)
////            colorWheel[k] = Vec3i(0, 255, 255 * i / GC);
////
////        for (int i = 0; i < CB; ++i, ++k)
////            colorWheel[k] = Vec3i(0, 255 - 255 * i / CB, 255);
////
////        for (int i = 0; i < BM; ++i, ++k)
////            colorWheel[k] = Vec3i(255 * i / BM, 0, 255);
////
////        for (int i = 0; i < MR; ++i, ++k)
////            colorWheel[k] = Vec3i(255, 0, 255 - 255 * i / MR);
////
////        first = false;
////    }
////
////    const float rad = sqrt(fx * fx + fy * fy);
////    const float a = atan2(-fy, -fx) / (float) CV_PI;
////
////    const float fk = (a + 1.0f) / 2.0f * (NCOLS - 1);
////    const int k0 = static_cast<int>(fk);
////    const int k1 = (k0 + 1) % NCOLS;
////    const float f = fk - k0;
////
////    Vec3b pix;
////
////    for (int b = 0; b < 3; b++)
////    {
////        const float col0 = colorWheel[k0][b] / 255.0f;
////        const float col1 = colorWheel[k1][b] / 255.0f;
////
////        float col = (1 - f) * col0 + f * col1;
////
////        if (rad <= 1)
////            col = 1 - rad * (1 - col); // increase saturation with radius
////        else
////            col *= .75; // out of range
////
////        pix[2 - b] = static_cast<uchar>(255.0 * col);
////    }
////
////    return pix;
////}
////
////static void drawOpticalFlow(const Mat_<float>& flowx, const Mat_<float>& flowy, Mat& dst, float maxmotion = -1)
////{
////    dst.create(flowx.size(), CV_8UC3);
////    dst.setTo(Scalar::all(0));
////
////    // determine motion range:
////    float maxrad = maxmotion;
////
////    if (maxmotion <= 0)
////    {
////        maxrad = 1;
////        for (int y = 0; y < flowx.rows; ++y)
////        {
////            for (int x = 0; x < flowx.cols; ++x)
////            {
////                Point2f u(flowx(y, x), flowy(y, x));
////
////                if (!isFlowCorrect(u))
////                    continue;
////
////                maxrad = max(maxrad, sqrt(u.x * u.x + u.y * u.y));
////            }
////        }
////    }
////
////    for (int y = 0; y < flowx.rows; ++y)
////    {
////        for (int x = 0; x < flowx.cols; ++x)
////        {
////            Point2f u(flowx(y, x), flowy(y, x));
////
////            if (isFlowCorrect(u))
////                dst.at<Vec3b>(y, x) = computeColor(u.x / maxrad, u.y / maxrad);
////        }
////    }
////}
////
////static void showFlow(const char* name, const GpuMat& d_flow)
////{
////    GpuMat planes[2];
////    cuda::split(d_flow, planes);
////
////    Mat flowx(planes[0]);
////    Mat flowy(planes[1]);
////
////    Mat out;
////    drawOpticalFlow(flowx, flowy, out, 10);
////
////    imshow(name, out);
////}
////
////int main(int argc, const char* argv[])
////{
////    string filename1, filename2;
////    if (argc < 3)
////    {
////        cerr << "Usage : " << argv[0] << " <frame0> <frame1>" << endl;
////        filename1 = "../data/basketball1.png";
////        filename2 = "../data/basketball2.png";
////    }
////    else
////    {
////        filename1 = argv[1];
////        filename2 = argv[2];
////    }
////
////    Mat frame0 = imread(filename1, IMREAD_GRAYSCALE);
////    Mat frame1 = imread(filename2, IMREAD_GRAYSCALE);
////
////    if (frame0.empty())
////    {
////        cerr << "Can't open image ["  << filename1 << "]" << endl;
////        return -1;
////    }
////    if (frame1.empty())
////    {
////        cerr << "Can't open image ["  << filename2 << "]" << endl;
////        return -1;
////    }
////
////    if (frame1.size() != frame0.size())
////    {
////        cerr << "Images should be of equal sizes" << endl;
////        return -1;
////    }
////
////    GpuMat d_frame0(frame0);
////    GpuMat d_frame1(frame1);
////
////    GpuMat d_flow(frame0.size(), CV_32FC2);
////
////    Ptr<cuda::BroxOpticalFlow> brox = cuda::BroxOpticalFlow::create(0.197f, 50.0f, 0.8f, 10, 77, 10);
////    Ptr<cuda::DensePyrLKOpticalFlow> lk = cuda::DensePyrLKOpticalFlow::create(Size(7, 7));
////    Ptr<cuda::FarnebackOpticalFlow> farn = cuda::FarnebackOpticalFlow::create();
////    Ptr<cuda::OpticalFlowDual_TVL1> tvl1 = cuda::OpticalFlowDual_TVL1::create();
////
////    {
////        GpuMat d_frame0f;
////        GpuMat d_frame1f;
////
////        d_frame0.convertTo(d_frame0f, CV_32F, 1.0 / 255.0);
////        d_frame1.convertTo(d_frame1f, CV_32F, 1.0 / 255.0);
////
////        const int64 start = getTickCount();
////
////        brox->calc(d_frame0f, d_frame1f, d_flow);
////
////        const double timeSec = (getTickCount() - start) / getTickFrequency();
////        cout << "Brox : " << timeSec << " sec" << endl;
////
////        showFlow("Brox", d_flow);
////    }
////
////    {
////        const int64 start = getTickCount();
////
////        lk->calc(d_frame0, d_frame1, d_flow);
////
////        const double timeSec = (getTickCount() - start) / getTickFrequency();
////        cout << "LK : " << timeSec << " sec" << endl;
////
////        showFlow("LK", d_flow);
////    }
////
////    {
////        const int64 start = getTickCount();
////
////        farn->calc(d_frame0, d_frame1, d_flow);
////
////        const double timeSec = (getTickCount() - start) / getTickFrequency();
////        cout << "Farn : " << timeSec << " sec" << endl;
////
////        showFlow("Farn", d_flow);
////    }
////
////    {
////        const int64 start = getTickCount();
////
////        tvl1->calc(d_frame0, d_frame1, d_flow);
////
////        const double timeSec = (getTickCount() - start) / getTickFrequency();
////        cout << "TVL1 : " << timeSec << " sec" << endl;
////
////        showFlow("TVL1", d_flow);
////    }
////
////    imshow("Frame 0", frame0);
////    imshow("Frame 1", frame1);
////    waitKey();
////
////    return 0;
////}
//
//
//
//
/////**
//// * @file bg_sub.cpp
//// * @brief Background subtraction tutorial sample code
//// * @author Domenico D. Bloisi
//// */
////
//////opencv
////#include "opencv2/imgcodecs.hpp"
////#include "opencv2/imgproc.hpp"
////#include "opencv2/videoio.hpp"
////#include "opencv2/highgui.hpp"
////#include "opencv2/video.hpp"
//////C
////#include <stdio.h>
//////C++
////#include <iostream>
////#include <sstream>
////
////using namespace cv;
////using namespace std;
////
////// Global variables
////Mat frame; //current frame
////Mat fgMaskMOG2; //fg mask fg mask generated by MOG2 method
////Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
////int keyboard; //input from keyboard
////
/////** Function Headers */
////void help();
////void processVideo(char* videoFilename);
////void processImages(char* firstFrameFilename);
////
////void help()
////{
////    cout
////    << "--------------------------------------------------------------------------" << endl
////    << "This program shows how to use background subtraction methods provided by "  << endl
////    << " OpenCV. You can process both videos (-vid) and images (-img)."             << endl
////    << endl
////    << "Usage:"                                                                     << endl
////    << "./bs {-vid <video filename>|-img <image filename>}"                         << endl
////    << "for example: ./bs -vid video.avi"                                           << endl
////    << "or: ./bs -img /data/images/1.png"                                           << endl
////    << "--------------------------------------------------------------------------" << endl
////    << endl;
////}
////
/////**
//// * @function main
//// */
////int main(int argc, char* argv[])
////{
////    //print help information
////    help();
////
////    argc = 3;
//////    strcpy(argv[1], "-vid");
//////    strcpy(argv[2], "/Users/drifter/Desktop/capstone/ver.mp4");
////
////    //check for the input parameter correctness
////    if(argc != 3) {
////        cerr <<"Incorret input list" << endl;
////        cerr <<"exiting..." << endl;
////        return EXIT_FAILURE;
////    }
////
////    //create GUI windows
////    namedWindow("Frame");
////    namedWindow("FG Mask MOG 2");
////
////    //create Background Subtractor objects
//////    pMOG2 = createBackgroundSubtractorMOG2(200,200, false);
////    pMOG2 = createBackgroundSubtractorMOG2(); //MOG2 approach
////
////    if(strcmp("-vid", "-vid") == 0) {
////        //input data coming from a video
////        char atLocation[100] = "/Users/drifter/Desktop/capstone/ver.mp4";
////        processVideo(atLocation);
////    }
////    else if(strcmp(argv[1], "-img") == 0) {
////        //input data coming from a sequence of images
////        processImages(argv[2]);
////    }
////    else {
////        //error in reading input parameters
////        cerr <<"Please, check the input parameters." << endl;
////        cerr <<"Exiting..." << endl;
////        return EXIT_FAILURE;
////    }
////    //destroy GUI windows
////    destroyAllWindows();
////    return EXIT_SUCCESS;
////}
////
/////**
//// * @function processVideo
//// */
////void processVideo(char* videoFilename) {
////    //create the capture object
////    VideoCapture capture(videoFilename);
////    if(!capture.isOpened()){
////        //error in opening the video input
////        cerr << "Unable to open video file: " << videoFilename << endl;
////        exit(EXIT_FAILURE);
////    }
////    //read input data. ESC or 'q' for quitting
////    while( (char)keyboard != 'q' && (char)keyboard != 27 ){
////        //read the current frame
////        if(!capture.read(frame)) {
////            cerr << "Unable to read next frame." << endl;
////            cerr << "Exiting..." << endl;
////            exit(EXIT_FAILURE);
////        }
////        //update the background model
////        pMOG2->apply(frame, fgMaskMOG2);
////        //get the frame number and write it on the current frame
////        stringstream ss;
//////        rectangle(frame, cv::Point(10, 2), cv::Point(100,20),
//////                  cv::Scalar(255,255,255), -1);
////        ss << capture.get(CAP_PROP_POS_FRAMES);
////        string frameNumberString = ss.str();
////        putText(frame, frameNumberString.c_str(), cv::Point(frame.cols-100, frame.rows-15),
////                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,255,255));
////        //show the current frame and the fg masks
////        imshow("Frame", frame);
////        imshow("FG Mask MOG 2", fgMaskMOG2);
////        //get the input from the keyboard
////        keyboard = waitKey( 30 );
////    }
////    //delete capture object
////    capture.release();
////}
////
/////**
//// * @function processImages
//// */
////void processImages(char* fistFrameFilename) {
////    //read the first file of the sequence
////    frame = imread(fistFrameFilename);
////    if(frame.empty()){
////        //error in opening the first image
////        cerr << "Unable to open first image frame: " << fistFrameFilename << endl;
////        exit(EXIT_FAILURE);
////    }
////    //current image filename
////    string fn(fistFrameFilename);
////    //read input data. ESC or 'q' for quitting
////    while( (char)keyboard != 'q' && (char)keyboard != 27 ){
////        //update the background model
////        pMOG2->apply(frame, fgMaskMOG2);
////        //get the frame number and write it on the current frame
////        size_t index = fn.find_last_of("/");
////        if(index == string::npos) {
////            index = fn.find_last_of("\\");
////        }
////        size_t index2 = fn.find_last_of(".");
////        string prefix = fn.substr(0,index+1);
////        string suffix = fn.substr(index2);
////        string frameNumberString = fn.substr(index+1, index2-index-1);
////        istringstream iss(frameNumberString);
////        int frameNumber = 0;
////        iss >> frameNumber;
////        rectangle(frame, cv::Point(10, 2), cv::Point(100,20),
////                  cv::Scalar(255,255,255), -1);
////        putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
////                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));
////        //show the current frame and the fg masks
////        imshow("Frame", frame);
////        imshow("FG Mask MOG 2", fgMaskMOG2);
////        //get the input from the keyboard
////        keyboard = waitKey( 30 );
////        //search for the next image in the sequence
////        ostringstream oss;
////        oss << (frameNumber + 1);
////        string nextFrameNumberString = oss.str();
////        string nextFrameFilename = prefix + nextFrameNumberString + suffix;
////        //read the next frame
////        frame = imread(nextFrameFilename);
////        if(frame.empty()){
////            //error in opening the next image in the sequence
////            cerr << "Unable to open image frame: " << nextFrameFilename << endl;
////            exit(EXIT_FAILURE);
////        }
////        //update the path of the current frame
////        fn.assign(nextFrameFilename);
////    }
////}
////
//
//
//
////
////#include "opencv2/videoio/videoio.hpp"
////#include "opencv2/highgui/highgui.hpp"
////#include "opencv2/imgproc/imgproc.hpp"
////
////#include <ctype.h>
////#include <stdio.h>
////#include <iostream>
////
////using namespace cv;
////using namespace std;
////
////static void help()
////{
////    cout <<
////    "\nThis program demonstrates Laplace point/edge detection using OpenCV function Laplacian()\n"
////    "It captures from the camera of your choice: 0, 1, ... default 0\n"
////    "Call:\n"
////    "./laplace [camera #, default 0]\n" << endl;
////}
////
////enum {GAUSSIAN, BLUR, MEDIAN};
////
////int sigma = 3;
////int smoothType = GAUSSIAN;
////int ksize = (sigma*5)|1;
//
////
////int main( int argc, char** argv )
////{
////    VideoCapture cap;
////    if(verbose)
////        help();
////
////    baseframe = imread(BASEFRAME_DIR);
////    cap.open("/Users/drifter/Desktop/capstone/ver.mp4");
//////    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
//////        cap.open(argc == 2 ? argv[1][0] - '0' : 0);
//////    else if( argc >= 2 )
//////    {
//////        cap.open(argv[1]);
//////        if( cap.isOpened() )
//////            cout << "Video " << argv[1] <<
//////            ": width=" << cap.get(CAP_PROP_FRAME_WIDTH) <<
//////            ", height=" << cap.get(CAP_PROP_FRAME_HEIGHT) <<
//////            ", nframes=" << cap.get(CAP_PROP_FRAME_COUNT) << endl;
//////        if( argc > 2 && isdigit(argv[2][0]) )
//////        {
//////            int pos;
//////            sscanf(argv[2], "%d", &pos);
//////            cout << "seeking to frame #" << pos << endl;
//////            cap.set(CAP_PROP_POS_FRAMES, pos);
//////        }
//////    }
////
////    if( !cap.isOpened() )
////    {
////        cout << "Could not initialize capturing...\n";
////        return -1;
////    }
////
////    namedWindow( "Laplacian", 0 );
////    createTrackbar( "Sigma", "Laplacian", &sigma, 15, 0 );
////
////    Mat smoothed, laplace, result;
////    vector<Vec3f> hcircles;
////
////    /* mold: A hollow form or matrix for shaping a single line from the video. */
////    /* constructor for Rect: Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height); */
////    Rect mold(0,baseframe.rows/4,baseframe.cols,baseframe.rows/2);
////    Rect tightMold(0,baseframe.rows/3.3,baseframe.cols,baseframe.rows/3.3);
////
////    for(;;)
////    {
////        Mat frame;
////
////        cap >> frame;
////        frame = frame(tightMold);
////        cvtColor(frame, frame, COLOR_BGR2GRAY);
////        Mat sketching(frame);
////
////        if( frame.empty() )
////            break;
//////        imshow("orig", frame);
////
////        int ksize = (sigma*5)|1;
//////        if(smoothType == GAUSSIAN)
//////            GaussianBlur(frame, smoothed, Size(ksize, ksize), sigma, sigma);
//////        else if(smoothType == BLUR)
//////            blur(frame, smoothed, Size(ksize, ksize));
//////        else
////            medianBlur(frame, smoothed, ksize);
////
////        Laplacian(smoothed, laplace, CV_16S, 5);
////        convertScaleAbs(laplace, result, (sigma+1)*0.25);
////
////        Mat laplace8u,circles;
////        convertScaleAbs(laplace, laplace8u, CV_16U, 5);
//////        HoughCircles(laplace8u, hcircles, CV_HOUGH_GRADIENT, 1, 20);
////
////        imshow("laplace8u", laplace8u);
////        for( size_t i = 0; i < hcircles.size(); i++ )
////        {
////            Point center(cvRound(hcircles[i][0]), cvRound(hcircles[i][1]));
////            int radius = cvRound(hcircles[i][2]);
////            // draw the circle center
////            circle( sketching, center, 3, Scalar(0,255,0), -1, 8, 0 );
////            // draw the circle outline
////            circle( sketching, center, radius, Scalar(0,0,255), 3, 8, 0 );
////        }
////
//////        imshow("Frame", frame);
//////        imshow("Laplacian", result);
//////        imshow("HoughCircles", sketching);
////
////        int c = waitKey(30);
////        if( c == ' ' )
////            smoothType = smoothType == GAUSSIAN ? BLUR : smoothType == BLUR ? MEDIAN : GAUSSIAN;
////        if( c == 'q' || c == 'Q' || (c & 255) == 27 )
////            break;
////    }
////
////    return 0;
////}
//
//
////#include "opencv2/video/tracking.hpp"
////#include "opencv2/highgui/highgui.hpp"
////
////#include <stdio.h>
////
////using namespace cv;
////
////static inline Point calcPoint(Point2f center, double R, double angle)
////{
////    return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
////}
////
////static void help()
////{
////    printf( "\nExample of c calls to OpenCV's Kalman filter.\n"
////           "   Tracking of rotating point.\n"
////           "   Rotation speed is constant.\n"
////           "   Both state and measurements vectors are 1D (a point angle),\n"
////           "   Measurement is the real point angle + gaussian noise.\n"
////           "   The real and the estimated points are connected with yellow line segment,\n"
////           "   the real and the measured points are connected with red line segment.\n"
////           "   (if Kalman filter works correctly,\n"
////           "    the yellow segment should be shorter than the red one).\n"
////           "\n"
////           "   Pressing any key (except ESC) will reset the tracking with a different speed.\n"
////           "   Pressing ESC will stop the program.\n"
////           );
////}
////
////int main(int, char**)
////{
////    help();
////    Mat img(500, 500, CV_8UC3);
////    KalmanFilter KF(2, 1, 0);
////    Mat state(2, 1, CV_32F); /* (phi, delta_phi) */
////    Mat processNoise(2, 1, CV_32F);
////    Mat measurement = Mat::zeros(1, 1, CV_32F);
////    char code = (char)-1;
////
////    for(;;)
////    {
////        randn( state, Scalar::all(0), Scalar::all(0.1) );
////        KF.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);
////
////        setIdentity(KF.measurementMatrix);
////        setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
////        setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
////        setIdentity(KF.errorCovPost, Scalar::all(1));
////
////        randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
////
////        for(;;)
////        {
////            Point2f center(img.cols*0.5f, img.rows*0.5f);
////            float R = img.cols/3.f;
////            double stateAngle = state.at<float>(0);
////            Point statePt = calcPoint(center, R, stateAngle);
////
////            Mat prediction = KF.predict();
////            double predictAngle = prediction.at<float>(0);
////            Point predictPt = calcPoint(center, R, predictAngle);
////
////            randn( measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));
////
////            // generate measurement
////            measurement += KF.measurementMatrix*state;
////
////            double measAngle = measurement.at<float>(0);
////            Point measPt = calcPoint(center, R, measAngle);
////
////            // plot points
////#define drawCross( center, color, d )                                        \
////line( img, Point( center.x - d, center.y - d ),                          \
////Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0); \
////line( img, Point( center.x + d, center.y - d ),                          \
////Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0 )
////
////            img = Scalar::all(0);
////            drawCross( statePt, Scalar(255,255,255), 3 );
////            drawCross( measPt, Scalar(0,0,255), 3 );
////            drawCross( predictPt, Scalar(0,255,0), 3 );
////            line( img, statePt, measPt, Scalar(0,0,255), 3, LINE_AA, 0 );
////            line( img, statePt, predictPt, Scalar(0,255,255), 3, LINE_AA, 0 );
////
////            if(theRNG().uniform(0,4) != 0)
////                KF.correct(measurement);
////
////            randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
////            state = KF.transitionMatrix*state + processNoise;
////
////            imshow( "Kalman", img );
////            code = (char)waitKey(100);
////
////            if( code > 0 )
////                break;
////        }
////        if( code == 27 || code == 'q' || code == 'Q' )
////            break;
////    }
////
////    return 0;
////}
//
//
//
//
//
//
//
//
//
//////
//////  main.cpp
//////  Customer Detection
//////
//////  Created by Rurouni on 12/17/15.
//////  Copyright © 2015 Rurouni. All rights reserved.
//////
////
//////Mat img;
//////Mat templ;
//////Mat result;
//////
////
////int main(int argc, const char * argv[]) {
////    baseframe = imread(BASEFRAME_DIR);
////
////    Mat Tmage = imread("/Users/drifter/Desktop/wt2.png");
////
////    capture.open(VIDEOPATH);
////
////    /* constructor for resize: resize(InputArray src, OutputArray dst, Size dsize) */
////    Size size(baseframe.cols,baseframe.rows);
////
////	/* mold: A hollow form or matrix for shaping a single line from the video. */
////    /* constructor for Rect: Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height); */
////    Rect mold(0,baseframe.rows/4,baseframe.cols,baseframe.rows/2);
////
////    while (1) {
////        //store image to matrix
////        capture >> aframe;
////        /* constructor for resize: resize(InputArray src, OutputArray dst, Size dsize) */
////        resize(aframe, aframe, size);
////
////        Mat bwaframe = aframe;
////        cvtColor(bwaframe, bwaframe, COLOR_BGR2GRAY);
////
////        /** bilateralFilter(InputArray src, OutputArray dst, int d, double sigmaColor, double sigmaSpace)*/
////        /** @brief bilateralFilter is computationally heavy, keep d=5 for real-time, sigmas can be same (ranges 10-200)*/
//////        bilateralFilter(aframe, dst, 5, 70, 70);
//////        imshow("linear filters2", dst);
////
////        /** @function boxFilter(InputArray src, OutputArray dst, int ddepth, Size ksize, Point anchor=Point(-1,-1), bool normalize=true, int borderType=BORDER_DEFAULT ); */
////        boxFilter(aframe, dst, -1, Size(9,9));
//////        imshow("box filtered", dst);
////
////        integral(aframe, dst);
//////        imshow("integrated", dst);
////
////        Mat fg;
////        erode(aframe,fg,Mat(),Point(-1,-1),2);
////
////        // Identify image pixels without objects
////        Mat bg;
////        dilate(aframe,bg,Mat(),Point(-1,-1),3);
////        threshold(bg,bg,1,128,THRESH_BINARY_INV);
////
////        Mat markers(aframe.size(),CV_8U,Scalar(0));
////        markers = fg + bg;
////        markers.convertTo(markers, CV_32S);
//////        watershed(bwaframe, markers);
////
//////        imshow("watershed", markers);
////
////        vector<Mat> dst_vec;
////        buildPyramid(aframe, dst_vec, 5);
////
//////        imshow("vector[1]", dst_vec[1]);
//////        imshow("vector[4]", dst_vec[4]);
////
////        pyrDown(aframe, aframe);
////        imshow("pyred down", aframe);
////
////
////        cvSmooth(aframe, dst);
////        approxPolyDP(aframe, dst, 10, true);
//
////        double alpha = 2.5; double beta; //double input;
////
////        beta = ( 1.0 - alpha );
////        addWeighted( aframe, alpha, baseframe, beta, 0.0, dst);
////
////        /* difference of two images over a Rect mold showing pixels with < 60 RGB values */
////        Mat cline = (baseframe - aframe)(mold) < 60;
////
////        /* grabCut(InputArray img, InputOutputArray mask, Rect rect, InputOutputArray bgdModel, InputOutputArray fgdModel, int iterCount, int mode=GC_EVAL ) */
////
////
////        /* floodFill(InputOutputArray image, Point seedPoint, Scalar newVal, Rect* rect=0, Scalar loDiff=Scalar(), Scalar upDiff=Scalar(), int flags=4 ) */
//////        uchar fillValue = 128;
//////        floodFill(cline, Point(200,0), WHITE, (Rect*)0, Scalar(), 8 | FLOODFILL_MASK_ONLY | (fillValue << 8) );
//////        floodFill(aframe, Point(0,0), Scalar(200), (Rect*)0, Scalar(), 8 | FLOODFILL_MASK_ONLY);
////
////        /* matchTemplate(InputArray image, InputArray templ, OutputArray result, int method) */
////        matchTemplate(aframe, Tmage, result, CV_TM_CCOEFF_NORMED);
////        normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
////
////        Mat addingI =  baseframe + aframe;
////        Mat contrast_brightness;
////
//////        Mat contrast_brightness = 2*aframe - 2*100;
////////        imshow("contrast_brightness", contrast_brightness);
//////
//////        contrast_brightness = 3*aframe - 2*100;
//////        imshow("contrast_brightness 3c", contrast_brightness);
////
//////        Mat image;
//////        GaussianBlur(aframe, image, Size(0,0), 1, 1.5);
//////        addWeighted(aframe, alpha, image, beta, 0.0, image);
////
//////        aframe = image;
////
//////        contrast_brightness = 4.2*aframe - 2*100;
//////        Mat contrasted_baseframe = 4.2*baseframe - 2*100;
//////
//////        imshow("contrast_brightness 4c", contrast_brightness);
//////        imshow("Contrasted baseframe 4c", contrasted_baseframe);
//////
//////
//////        Mat diffX =contrasted_baseframe - contrast_brightness;
////
////
//////        cvtColor( diffX, diffX, COLOR_BGR2GRAY );
////
//////        diffX = diffX > 240;
//////        imshow("diff contrasted vs base contrasted", diffX);
////
//////        imshow("aframe + baseframe", addingI);
//////        Mat subsI = baseframe = aframe;
//////        imshow("aframe - baseframe", subsI);
//////
////////        result = result  > 50;
////////        Mat m1 = result;
////////        Mat m2 = result;
////////
////////        imshow("result",result);
//////////        resize(m1, m1, size);
//////////        resize(result,result,size);
////////
////////        m1 = result - m1;
////////        m2 = m2 - result;
////////        imshow("m1", m1);
////////        imshow("m2", m2);
//////
////////        imshow("template match", result);
//////
////        img = aframe;
////        templ = Tmage;
////        /// Create Trackbar
////        char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
////        createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );
////
////        MatchingMethod( 0, 0 );
////
////        waitKey(1);
////        int x = 1;
////        if(x == 1)
////            continue;
//////
//////        Mat dist;
//////        cvtColor( aframe, dist, COLOR_BGR2GRAY );
//////        imshow("dist with 0", dist);
//////
//////
//////
//////        waitKey(1);
//////
////////        Mat I = dist;
////////        Mat padded;                            //expand input image to optimal size
////////        int m = getOptimalDFTSize( I.rows );
////////        int n = getOptimalDFTSize( I.cols ); // on the border add zero values
////////        copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));
////////
////////        Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
////////        Mat complexI;
////////        merge(planes, 2, complexI);         // Add to the expanded another plane with zeros
////////
////////        dft(complexI, complexI);            // this way the result may fit in the source matrix
//////
////////dft(dist, tmp);
////////        imshow("fourier", complexI);
//////
//////
////
////        alpha = 0.5; //double input;
////
////        beta = ( 1.0 - alpha );
////        addWeighted( aframe, alpha, baseframe, beta, 0.0, dst);
//////        imshow("originals blended", dst);
////        Rect customerline(0,dst.rows/4,dst.cols,dst.rows/2);
////        cline = dst(customerline);
////        imshow("Blended", cline);
////
////////        Canny(dist, dist, 0, 5);
//////        Mat image;
//////        GaussianBlur(dist, image, Size(0,0), 1, 1.5);
//////        addWeighted(dist, alpha, image, beta, 0.0, image);
//////
//////        imshow("sharpened", image);
//////        imshow("dist", dist);
//////        distanceTransform(dist, dist, CV_DIST_L2, 3);
//////
//////        imshow("non-normalized", dist);
//////        normalize(dist, dist, 0.0, 1.0, NORM_MINMAX);
//////        imshow("normalized", dist);
//////
//////        imshow("aframe", aframe);
//////        imshow("cline", cline);
////
//////        imshow("diff", dst );
////        char key = waitKey(1); // waits to display frame
////        if (key == 'q') {
////            break;
////        }
////    }
////}
//
////    baseframe = baseframe > 150;
////    cvtColor( aframe, aframe, COLOR_BGR2GRAY );
////    GaussianBlur(aframe, aframe, Size(7,7), 1.5, 1.5);
////    Canny(aframe, aframe, 0, 15, 3);
//
////    findContours(aframe,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE, Point(0,0));
//
///* constructor for Rect: Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height); */
////    Rect line(0,baseframe.rows/4,baseframe.cols,baseframe.rows/2);
////    Rect bline(0,baseframe.rows/4,baseframe.cols,baseframe.rows/2);
//
////    Mat afr = aframe(aline);
////    Mat bfr = baseframe(bline);
//
////    imshow("afr", afr);
////    imshow("bfr", bfr);
//
////    double alpha = 0.5; double beta; //double input;
////
////    beta = ( 1.0 - alpha );
////    addWeighted( aframe, alpha, baseframe, beta, 0.0, dst);
//////    imshow("originals blended", dst);
////    Rect customerline(0,dst.rows/4,dst.cols,dst.rows/2);
//////    Mat cline = dst(customerline);
//////    imshow("Blended", cline);
////
//////    aframe = aframe > 50;
//////    baseframe = baseframe > 50;
//////    cvtColor( aframe, aframe, COLOR_BGR2 GRAY );
//////    cvtColor( baseframe, baseframe, COLOR_BGR2GRAY );
////    srcCanny = aframe;
////    cvtColor( srcCanny, src_gray, COLOR_BGR2GRAY );
////    namedWindow( window_name, CV_WINDOW_AUTOSIZE );
////
////    createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
////
////    /// Show the image
////    CannyThreshold(0, 0);
////
//////    GaussianBlur(aframe, aframe, Size(7,7), 100.5, 1.5);
//////    GaussianBlur(baseframe, baseframe, Size(7,7), 1.5, 1.5);
//////    namedWindow("gaussian blur", CV_WINDOW_AUTOSIZE);
//////    int slider = 0;
//////    createTrackbar("gaussian blur %", "blurs", &slider, 100, blur_func);
////
//////    Canny(aframe, aframe, 1, 15, 3);
////
////    imshow("only canny", aframe);
////    waitKey(0);
////
//////    baseframe = baseframe < 01;
////    Mat diff = baseframe - aframe;
////    Mat diff2 = aframe - baseframe;
////    Mat diff3 = baseframe - dst;
////    Mat diff4 = dst - aframe;
////
//////    GaussianBlur(diff, diff, Size(7,7), 1.5, 1.5);
////
//////    diff3 = diff3 < 20;
////    diff = diff < 60;
////    diff2 = diff2 < 50;
////    diff3 = diff3 < 50;
////    diff4 = diff4 < 50;
////    imshow("Difference", diff);
////    imshow("Difference2", diff2);
////
////    Mat modline = diff3(customerline);
////    imshow("Difference3", diff3);
////    imshow("Difference4", diff4);
////    imshow("Mod diff3", modline);
////
////    namedWindow("originals blended", CV_WINDOW_AUTOSIZE );
////    alpha_slider = 0;
////    createTrackbar( "Blend %\n", "originals blended", &alpha_slider, alpha_slider_max, on_trackbar );
////
////    /// Show some stuff
////    on_trackbar( alpha_slider, 0 );
////
////    waitKey(0);
//////    aframe = aframe < 60;
//////     uncomment next line when ready with trackbarslide
////    /*for( size_t i = 0; i< contours.size(); i++ )
////     {
////      rng random generator of color from passed interval
////     Scalar color = Scalar( rng.uniform(00,00), rng.uniform(00,00), rng.uniform(00,00) );
////     drawContours( frame, contours, (int)i, color, 2, 8, hierarchy, 200, Point() );
////     } */
////
//////    cframe = cframe > 120;
////    //    aframe = imread("/Users/drifter/Dropbox/Feloh/topview.jpg", CV_LOAD_IMAGE_COLOR);
////    //    aframe = imread("/Users/drifter/Dropbox/Feloh/brachs.png", CV_LOAD_IMAGE_COLOR);
////
//////    src = aframe;
////    src = aframe;
////    //    erode( aframe, aframe, e_element );
////
////    /// Create windows
////    namedWindow( "Erosion", CV_WINDOW_AUTOSIZE );
////    namedWindow( "Dilation", CV_WINDOW_AUTOSIZE );
////    cvMoveWindow( "Dilation", src.cols, 0 );
////
////    /// Create Erosion Trackbar
////    createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion",
////                   &erosion_elem, max_elem,
////                   Erosion );
////
////    createTrackbar( "Kernel size:\n 2n +1", "Erosion",
////                   &erosion_size, max_kernel_size,
////                   Erosion );
////
////    /// Create Dilation Trackbar
////    createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation",
////                   &dilation_elem, max_elem,
////                   Dilation );
////
////    createTrackbar( "Kernel size:\n 2n +1", "Dilation",
////                   &dilation_size, max_kernel_size,
////                   Dilation );
////
////    /// Create window
////    namedWindow( morph, CV_WINDOW_AUTOSIZE );
////
////    /// Create Trackbar to select Morphology operation
////    createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", morph, &morph_operator, max_operator, Morphology_Operations );
////
////    /// Create Trackbar to select kernel type
////    createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", morph,
////                   &morph_elem, max_elem,
////                   Morphology_Operations );
////
////    /// Create Trackbar to choose kernel size
////    createTrackbar( "Kernel size:\n 2n +1", morph,
////                   &morph_size, max_kernel_size,
////                   Morphology_Operations );
////
////
////    /// Default start
////    Erosion( 0, 0 );
////    Dilation( 0, 0 );
////    Morphology_Operations( 0, 0 );
////
////    //    imshow("Top view",aframe);
////    waitKey(0);
////
////    std::cout << "out\n";
////    return 0;
////}
////
/////**
//// * @function MatchingMethod
//// * @brief Trackbar callback
//// */
////void MatchingMethod( int, void* )
////{
////    /// Source image to display
////    Mat img_display;
////    img.copyTo( img_display );
////
////    /// Create the result matrix
////    int result_cols =  img.cols - templ.cols + 1;
////    int result_rows = img.rows - templ.rows + 1;
////
////    result.create( result_rows, result_cols, CV_32FC1 );
////
////    /// Do the Matching and Normalize
////    matchTemplate( img, templ, result, match_method );
////    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
////
////    /// Localizing the best match with minMaxLoc
////    double minVal; double maxVal; Point minLoc; Point maxLoc;
////    Point matchLoc;
////
////    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
////
////    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
////    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
////    { matchLoc = minLoc; }
////    else
////    { matchLoc = maxLoc; }
////
////    /// Show me what you got
////    rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
////    rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
////
////    imshow( image_window, img_display );
////    imshow( result_window, result );
////
////}
////
/////**
//// * @function CannyThreshold
//// * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
//// */
////void CannyThreshold(int, void*)
////{
////    /// Reduce noise with a kernel 3x3
////    blur( src_gray, detected_edges, Size(3,3) );
////
////    /// Canny detector
////    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratioCanny, kernel_size );
////
////    /// Using Canny's output as a mask, we display our result
////    dstCanny = Scalar::all(0);
////
////    srcCanny.copyTo( dstCanny, detected_edges);
////    imshow( window_name, dstCanny );
////}
////
/////**
//// * @function on_trackbar
//// * @brief Callback for trackbar
//// */
////void on_trackbar( int, void* )
////{
////    alpha = (double) alpha_slider/alpha_slider_max ;
////    beta = ( 1.0 - alpha );
////    src1 = baseframe;
////    src2 = aframe;
////    addWeighted( src1, alpha, src2, beta, 0.0, dstslider);
////
////    imshow( "originals blended", dstslider );
////}
////
////
/////**  @function Erosion  */
////void Erosion( int, void* )
////{
////    int erosion_type;
////    if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
////    else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
////    else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
////
////    Mat element = getStructuringElement( erosion_type,
////                                        Size( 2*erosion_size + 1, 2*erosion_size+1 ),
////                                        Point( erosion_size, erosion_size ) );
////
////    /// Apply the erosion operation
////    erode( src, erosion_dst, element );
////    imshow( "Erosion", erosion_dst );
////}
////
/////** @function Dilation */
////void Dilation( int, void* )
////{
////    int dilation_type;
////    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
////    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
////    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
////
////    Mat element = getStructuringElement( dilation_type,
////                                        Size( 2*dilation_size + 1, 2*dilation_size+1 ),
////                                        Point( dilation_size, dilation_size ) );
////    /// Apply the dilation operation
////    dilate( src, dilation_dst, element );
////    imshow( "Dilation", dilation_dst );
////}
////
/////**
//// * @function Morphology_Operations
//// */
////void Morphology_Operations( int, void* )
////{
////    // Since MORPH_X : 2,3,4,5 and 6
////    int operation = morph_operator + 2;
////
////    /* Make your own kernel mask */
////    Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
////
////    /// Apply the specified morphology operation
////    morphologyEx( src, dst, operation, element );
////    imshow( morph, dst );
////}
////
////
//
//
/////**
//// * @function MatchingMethod
//// * @brief Trackbar callback
//// */
////void MatchingMethod( int, void* )
////{
////    /// Source image to display
////    Mat img_display;
////    img.copyTo( img_display );
////    Mat tmp2 = imread("/Users/drifter/Desktop/laplacian.png", 0);
////    Size size(img.cols, img.rows);
////    resize(tmp2, tmp2, size);
////    addWeighted(img, .5, tmp2, .5, 0.0, tmp2);
////
////    /// Create the result matrix
////    int result_cols =  img.cols - templ.cols + 1;
////    int result_rows = img.rows - templ.rows + 1;
////
////    result.create( result_rows, result_cols, CV_32FC1 );
////
////    /// Do the Matching and Normalize
////    matchTemplate( img, templ, result, match_method );
////    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
////
////    /// Localizing the best match with minMaxLoc
////    double minVal; double maxVal; Point minLoc; Point maxLoc;
////    Point matchLoc;
////
////    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
////
////    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
////    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
////    { matchLoc = minLoc; }
////    else
////    { matchLoc = maxLoc; }
////
////    /// Show me what you got
////    rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
////    rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
////
////    imshow( image_window, img_display );
////    imshow( result_window, result );
////
////}
