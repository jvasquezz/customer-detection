#include "/Users/drifter/Dropbox/Feloh/FelohDependencies/FelohDependencies.h"
#include "this.dependencies.h"
//
//  main.cpp
//  Customer Detection
//
//  Created by Rurouni on 12/17/15.
//  Copyright © 2015 Rurouni. All rights reserved.
//

Mat img;
Mat templ;
Mat result;


int main(int argc, const char * argv[]) {
//    int x = 1;
    baseframe = imread(BASEFRAME_DIR);

    Mat Tmage = imread("/Users/drifter/Desktop/wt2.png");
    
    capture.open(VIDEOPATH);

    /* constructor for resize: resize(InputArray src, OutputArray dst, Size dsize) */
    Size size(baseframe.cols,baseframe.rows);

	/* mold: A hollow form or matrix for shaping a single line from the video. */
    /* constructor for Rect: Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height); */
    Rect mold(0,baseframe.rows/4,baseframe.cols,baseframe.rows/2);

    while (1) {
        //store image to matrix
        capture >> aframe;
        /* constructor for resize: resize(InputArray src, OutputArray dst, Size dsize) */
        resize(aframe, aframe, size);

        double alpha = 2.5; double beta; //double input;
        
        beta = ( 1.0 - alpha );
        addWeighted( aframe, alpha, baseframe, beta, 0.0, dst);

        /* difference of two images over a Rect mold showing pixels with < 60 RGB values */
        Mat cline = (baseframe - aframe)(mold) < 60;
        
        /* grabCut(InputArray img, InputOutputArray mask, Rect rect, InputOutputArray bgdModel, InputOutputArray fgdModel, int iterCount, int mode=GC_EVAL ) */
        
        
        /* floodFill(InputOutputArray image, Point seedPoint, Scalar newVal, Rect* rect=0, Scalar loDiff=Scalar(), Scalar upDiff=Scalar(), int flags=4 ) */
//        uchar fillValue = 128;
//        floodFill(cline, Point(200,0), WHITE, (Rect*)0, Scalar(), 8 | FLOODFILL_MASK_ONLY | (fillValue << 8) );
//        floodFill(aframe, Point(0,0), Scalar(200), (Rect*)0, Scalar(), 8 | FLOODFILL_MASK_ONLY);
        
        /* matchTemplate(InputArray image, InputArray templ, OutputArray result, int method) */
        matchTemplate(aframe, Tmage, result, CV_TM_CCOEFF_NORMED);
        normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
        
        Mat addingI =  baseframe + aframe;
        Mat contrast_brightness;
        
//        Mat contrast_brightness = 2*aframe - 2*100;
////        imshow("contrast_brightness", contrast_brightness);
//        
//        contrast_brightness = 3*aframe - 2*100;
//        imshow("contrast_brightness 3c", contrast_brightness);

//        Mat image;
//        GaussianBlur(aframe, image, Size(0,0), 1, 1.5);
//        addWeighted(aframe, alpha, image, beta, 0.0, image);
        
//        aframe = image;

//        contrast_brightness = 4.2*aframe - 2*100;
//        Mat contrasted_baseframe = 4.2*baseframe - 2*100;
//        
//        imshow("contrast_brightness 4c", contrast_brightness);
//        imshow("Contrasted baseframe 4c", contrasted_baseframe);
//        
//        
//        Mat diffX =contrasted_baseframe - contrast_brightness;
        
        
//        cvtColor( diffX, diffX, COLOR_BGR2GRAY );

//        diffX = diffX > 240;
//        imshow("diff contrasted vs base contrasted", diffX);
        
//        imshow("aframe + baseframe", addingI);
//        Mat subsI = baseframe = aframe;
//        imshow("aframe - baseframe", subsI);
//
////        result = result  > 50;
////        Mat m1 = result;
////        Mat m2 = result;
////        
////        imshow("result",result);
//////        resize(m1, m1, size);
//////        resize(result,result,size);
////        
////        m1 = result - m1;
////        m2 = m2 - result;
////        imshow("m1", m1);
////        imshow("m2", m2);
//        
////        imshow("template match", result);
//
        img = aframe;
        templ = Tmage;
        /// Create Trackbar
        char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
        createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );
        
        MatchingMethod( 0, 0 );
        
        waitKey(1);
        int x = 1;
        if(x == 1)
            continue;
//
//        Mat dist;
//        cvtColor( aframe, dist, COLOR_BGR2GRAY );
//        imshow("dist with 0", dist);
//        
//
//        
//        waitKey(1);
//        
////        Mat I = dist;
////        Mat padded;                            //expand input image to optimal size
////        int m = getOptimalDFTSize( I.rows );
////        int n = getOptimalDFTSize( I.cols ); // on the border add zero values
////        copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));
////        
////        Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
////        Mat complexI;
////        merge(planes, 2, complexI);         // Add to the expanded another plane with zeros
////
////        dft(complexI, complexI);            // this way the result may fit in the source matrix
//
////dft(dist, tmp);
////        imshow("fourier", complexI);
//
//
        
        alpha = 0.5; //double input;
        
        beta = ( 1.0 - alpha );
        addWeighted( aframe, alpha, baseframe, beta, 0.0, dst);
//        imshow("originals blended", dst);
        Rect customerline(0,dst.rows/4,dst.cols,dst.rows/2);
        cline = dst(customerline);
        imshow("Blended", cline);
        
////        Canny(dist, dist, 0, 5);
//        Mat image;
//        GaussianBlur(dist, image, Size(0,0), 1, 1.5);
//        addWeighted(dist, alpha, image, beta, 0.0, image);
//
//        imshow("sharpened", image);
//        imshow("dist", dist);
//        distanceTransform(dist, dist, CV_DIST_L2, 3);
//
//        imshow("non-normalized", dist);
//        normalize(dist, dist, 0.0, 1.0, NORM_MINMAX);
//        imshow("normalized", dist);
//        
//        imshow("aframe", aframe);
//        imshow("cline", cline);
        
//        imshow("diff", dst );
        char key = waitKey(1); // waits to display frame
        if (key == 'q') {
            break;
        }
    }
    
//    baseframe = baseframe > 150;
//    cvtColor( aframe, aframe, COLOR_BGR2GRAY );
//    GaussianBlur(aframe, aframe, Size(7,7), 1.5, 1.5);
//    Canny(aframe, aframe, 0, 15, 3);
   
//    findContours(aframe,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE, Point(0,0));

    /* constructor for Rect: Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height); */
//    Rect line(0,baseframe.rows/4,baseframe.cols,baseframe.rows/2);
//    Rect bline(0,baseframe.rows/4,baseframe.cols,baseframe.rows/2);
    
//    Mat afr = aframe(aline);
//    Mat bfr = baseframe(bline);
    
//    imshow("afr", afr);
//    imshow("bfr", bfr);
    
//    double alpha = 0.5; double beta; //double input;
    
    beta = ( 1.0 - alpha );
    addWeighted( aframe, alpha, baseframe, beta, 0.0, dst);
//    imshow("originals blended", dst);
    Rect customerline(0,dst.rows/4,dst.cols,dst.rows/2);
//    Mat cline = dst(customerline);
//    imshow("Blended", cline);
    
//    aframe = aframe > 50;
//    baseframe = baseframe > 50;
//    cvtColor( aframe, aframe, COLOR_BGR2 GRAY );
//    cvtColor( baseframe, baseframe, COLOR_BGR2GRAY );
    srcCanny = aframe;
    cvtColor( srcCanny, src_gray, COLOR_BGR2GRAY );
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
    
    /// Show the image
    CannyThreshold(0, 0);
    
//    GaussianBlur(aframe, aframe, Size(7,7), 100.5, 1.5);
//    GaussianBlur(baseframe, baseframe, Size(7,7), 1.5, 1.5);
//    namedWindow("gaussian blur", CV_WINDOW_AUTOSIZE);
//    int slider = 0;
//    createTrackbar("gaussian blur %", "blurs", &slider, 100, blur_func);

//    Canny(aframe, aframe, 1, 15, 3);
    
    imshow("only canny", aframe);
    waitKey(0);
    
//    baseframe = baseframe < 01;
    Mat diff = baseframe - aframe;
    Mat diff2 = aframe - baseframe;
    Mat diff3 = baseframe - dst;
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
 * @function MatchingMethod
 * @brief Trackbar callback
 */
void MatchingMethod( int, void* )
{
    /// Source image to display
    Mat img_display;
    img.copyTo( img_display );
    
    /// Create the result matrix
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;
    
    result.create( result_rows, result_cols, CV_32FC1 );
    
    /// Do the Matching and Normalize
    matchTemplate( img, templ, result, match_method );
    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
    
    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;
    
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    
    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
    else
    { matchLoc = maxLoc; }
    
    /// Show me what you got
    rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
    rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
    
    imshow( image_window, img_display );
    imshow( result_window, result );
    
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
    src1 = baseframe;
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
