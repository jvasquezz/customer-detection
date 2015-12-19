#include "/Users/drifter/Dropbox/Feloh/FelohDependencies/FelohDependencies.h"
#include "this.dependencies.h"
//
//  main.cpp
//  Customer Detection
//
//  Created by Rurouni on 12/17/15.
//  Copyright © 2015 Rurouni. All rights reserved.
//

int main(int argc, const char * argv[]) {
    
    // insert code here...
//    Mat aframe, bframe;
//    aframe = imread("/Users/drifter/Dropbox/Feloh/inonsho.png", CV_LOAD_IMAGE_COLOR);
//    aframe = imread("/Users/drifter/Dropbox/Feloh/Customer Detection/frame.png", CV_LOAD_IMAGE_COLOR);
//    bframe = imread("/Users/drifter/Dropbox/Feloh/Customer Detection/1frame.png", CV_LOAD_IMAGE_COLOR);
    aframe = imread("/Users/drifter/Dropbox/Feloh/Customer Detection/frame.png");
    bframe = imread("/Users/drifter/Dropbox/Feloh/Customer Detection/baseframe.png");
    
//    bframe = bframe > 150;
//    cvtColor( aframe, aframe, COLOR_BGR2GRAY );
//    GaussianBlur(aframe, aframe, Size(7,7), 1.5, 1.5);
//    Canny(aframe, aframe, 0, 15, 3);
   
//    findContours(aframe,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE, Point(0,0));

    /* constructor for Rect: Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height); */
    Rect line(0,bframe.rows/4,bframe.cols,bframe.rows/2);
//    Rect bline(0,bframe.rows/4,bframe.cols,bframe.rows/2);
    
//    Mat afr = aframe(aline);
//    Mat bfr = bframe(bline);
    
//    imshow("afr", afr);
//    imshow("bfr", bfr);
    
    double alpha = 0.5; double beta; //double input;
    
    beta = ( 1.0 - alpha );
    addWeighted( aframe, alpha, bframe, beta, 0.0, dst);
//    imshow("originals blended", dst);
    Rect customerline(0,dst.rows/4,dst.cols,dst.rows/2);
    Mat cline = dst(customerline);
    imshow("Blended", cline);
    
//    aframe = aframe > 50;
//    bframe = bframe > 50;
//    cvtColor( aframe, aframe, COLOR_BGR2 GRAY );
//    cvtColor( bframe, bframe, COLOR_BGR2GRAY );
    srcCanny = aframe;
    cvtColor( srcCanny, src_gray, COLOR_BGR2GRAY );
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
    
    /// Show the image
    CannyThreshold(0, 0);

    
//    GaussianBlur(aframe, aframe, Size(7,7), 100.5, 1.5);
//    GaussianBlur(bframe, bframe, Size(7,7), 1.5, 1.5);
//    namedWindow("gaussian blur", CV_WINDOW_AUTOSIZE);
//    int slider = 0;
//    createTrackbar("gaussian blur %", "blurs", &slider, 100, blur_func);

//    Canny(aframe, aframe, 1, 15, 3);
    
    imshow("only canny", aframe);
    waitKey(0);
    
//    bframe = bframe < 01;
    Mat diff = bframe - aframe;
    Mat diff2 = aframe - bframe;
    Mat diff3 = bframe - dst;
    Mat diff4 = dst - aframe;
    
//    GaussianBlur(diff, diff, Size(7,7), 1.5, 1.5);

//    diff3 = diff3 < 20;
    diff = diff < 60;
    diff2 = diff2 < 50;
    diff3 = diff3 < 50;
    diff4 = diff4 < 50;
    imshow("Difference", diff);
    imshow("Difference2", diff2);
    
    Mat modline = diff3(customerline);
    imshow("Difference3", diff3);
    imshow("Difference4", diff4);
    imshow("Mod diff3", modline);
    
    namedWindow("originals blended", CV_WINDOW_AUTOSIZE );
    alpha_slider = 0;
    createTrackbar( "Blend %\n", "originals blended", &alpha_slider, alpha_slider_max, on_trackbar );

    /// Show some stuff
    on_trackbar( alpha_slider, 0 );

    waitKey(0);
//    aframe = aframe < 60;
//     uncomment next line when ready with trackbarslide
    /*for( size_t i = 0; i< contours.size(); i++ )
     {
      rng random generator of color from passed interval
     Scalar color = Scalar( rng.uniform(00,00), rng.uniform(00,00), rng.uniform(00,00) );
     drawContours( frame, contours, (int)i, color, 2, 8, hierarchy, 200, Point() );
     } */

//    cframe = cframe > 120;
    //    aframe = imread("/Users/drifter/Dropbox/Feloh/topview.jpg", CV_LOAD_IMAGE_COLOR);
    //    aframe = imread("/Users/drifter/Dropbox/Feloh/brachs.png", CV_LOAD_IMAGE_COLOR);
    
//    src = aframe;
    src = aframe;
    //    erode( aframe, aframe, e_element );
    
    /// Create windows
    namedWindow( "Erosion", CV_WINDOW_AUTOSIZE );
    namedWindow( "Dilation", CV_WINDOW_AUTOSIZE );
    cvMoveWindow( "Dilation", src.cols, 0 );
    
    /// Create Erosion Trackbar
    createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion",
                   &erosion_elem, max_elem,
                   Erosion );
    
    createTrackbar( "Kernel size:\n 2n +1", "Erosion",
                   &erosion_size, max_kernel_size,
                   Erosion );
    
    /// Create Dilation Trackbar
    createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation",
                   &dilation_elem, max_elem,
                   Dilation );
    
    createTrackbar( "Kernel size:\n 2n +1", "Dilation",
                   &dilation_size, max_kernel_size,
                   Dilation );
    
    /// Create window
    namedWindow( morph, CV_WINDOW_AUTOSIZE );
    
    /// Create Trackbar to select Morphology operation
    createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", morph, &morph_operator, max_operator, Morphology_Operations );
    
    /// Create Trackbar to select kernel type
    createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", morph,
                   &morph_elem, max_elem,
                   Morphology_Operations );
    
    /// Create Trackbar to choose kernel size
    createTrackbar( "Kernel size:\n 2n +1", morph,
                   &morph_size, max_kernel_size,
                   Morphology_Operations );
    
    
    /// Default start
    Erosion( 0, 0 );
    Dilation( 0, 0 );
    Morphology_Operations( 0, 0 );
    
    //    imshow("Top view",aframe);
    waitKey(0);
    
    std::cout << "out\n";
    return 0;
}

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void CannyThreshold(int, void*)
{
    /// Reduce noise with a kernel 3x3
    blur( src_gray, detected_edges, Size(3,3) );
    
    /// Canny detector
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratioCanny, kernel_size );
    
    /// Using Canny's output as a mask, we display our result
    dstCanny = Scalar::all(0);
    
    srcCanny.copyTo( dstCanny, detected_edges);
    imshow( window_name, dstCanny );
}

/**
 * @function on_trackbar
 * @brief Callback for trackbar
 */
void on_trackbar( int, void* )
{
    alpha = (double) alpha_slider/alpha_slider_max ;
    beta = ( 1.0 - alpha );
    src1 = bframe;
    src2 = aframe;
    addWeighted( src1, alpha, src2, beta, 0.0, dstslider);
    
    imshow( "originals blended", dstslider );
}


/**  @function Erosion  */
void Erosion( int, void* )
{
    int erosion_type;
    if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
    else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
    else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
    
    Mat element = getStructuringElement( erosion_type,
                                        Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                        Point( erosion_size, erosion_size ) );
    
    /// Apply the erosion operation
    erode( src, erosion_dst, element );
    imshow( "Erosion", erosion_dst );
}

/** @function Dilation */
void Dilation( int, void* )
{
    int dilation_type;
    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
    
    Mat element = getStructuringElement( dilation_type,
                                        Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                        Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    dilate( src, dilation_dst, element );
    imshow( "Dilation", dilation_dst );
}

/**
 * @function Morphology_Operations
 */
void Morphology_Operations( int, void* )
{
    // Since MORPH_X : 2,3,4,5 and 6
    int operation = morph_operator + 2;
    
    /* Make your own kernel mask */
    Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    
    /// Apply the specified morphology operation
    morphologyEx( src, dst, operation, element );
    imshow( morph, dst );
}





//int main(int argc, const char * argv[]) {
//    // insert code here...
//    cframe = imread("/Users/drifter/Dropbox/Feloh/Customer Detection/frame.png");
//    
//    if(not cframe.data)
//        return -1;
//
////    cframe = cframe < 75;
//    
//    cvtColor( cframe, cframe, COLOR_BGR2GRAY );
//    GaussianBlur(cframe, cframe, Size(7,7), 1.5, 1.5);
//    
//    Canny(cframe, cframe, 0, 30, 3);
//    
//    erode( cframe, cframe, e_element );    
//
//    imshow("current frame", cframe);
//    waitKey(0);
//    return 0;
//}
