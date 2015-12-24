#include "/Users/drifter/Dropbox/Feloh/FelohDependencies/FelohDependencies.h"
#include "this.dependencies.h"


int main(int argc, const char * argv[]) {
    

    enum {GAUSSIAN, BLUR, MEDIAN};
    
    int sigma = 3;
    int smoothType = GAUSSIAN;
    int ksize = (sigma*5)|1;
    // insert code here...
    VideoCapture cap;
//    cap.open("/Users/drifter/Desktop/capstone/SEGMENTA_720P_20FPS.mp4");
//    cap.open("/Users/drifter/Desktop/capstone/ver.mp4");
//    cap.open("/Users/drifter/Desktop/capstone/Untitled.mp4");
//    cap.open("/Users/drifter/Desktop/capstone/30FPS.mp4");
//    cap.open("/Users/drifter/Desktop/capstone/6FPS.mp4");
//    cap.open("/Users/drifter/Desktop/capstone/1FPS.mp4");
//    cap.open("/Users/drifter/Desktop/capstone/3FPS.mp4");
    cap.open("/Users/drifter/Desktop/capstone/10FPS.mp4");
//    cframe = imread("/Users/drifter/Dropbox/Feloh/Customer Detection/frame.png",0);
    baseframe = imread(BASEFRAME_DIR);
    
    namedWindow( "Laplacian", 0 );
    //
    Mat smoothed, laplace, result;

    /* mold: A hollow form or matrix for shaping a single line from the video. */
    /* constructor for Rect: Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height); */
    Rect mold(0,baseframe.rows/4,baseframe.cols,baseframe.rows/2);
    Rect tightMold(0,baseframe.rows/3.3,baseframe.cols,baseframe.rows/3.3);

    Mat frame, bframe, cusframe;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int thresh = 100;
    bframe = baseframe(mold);
    Mat secBaseFrame, tempY;
    cap >> tempY;
    secBaseFrame = tempY(mold);
    
    
    for(;;) {
//        for(int op=0; op<9; op++) {
//            cap >> frame;
//        }
        cap >> frame;
        cusframe = frame(mold);
        
        Mat grays, basegrays;
        cvtColor(cusframe, grays, COLOR_BGR2GRAY);
        cvtColor(bframe, basegrays, COLOR_BGR2GRAY);
        diff = basegrays - grays;
        
        Mat diff2 = grays - basegrays;
        imshow("Diff2", diff2);
        diff = diff < 60;

//        int ksize = (sigma*5)|1;
//        blur(frame, smoothed, Size(ksize, ksize));
//        if(smoothType == GAUSSIAN)
//            GaussianBlur(frame, smoothed, Size(ksize, ksize), sigma, sigma);
//        else if(smoothType == BLUR)
//            blur(frame, smoothed, Size(ksize, ksize));
//        else
//            medianBlur(frame, smoothed, ksize);
        
//        medianBlur(diff, smoothed, ksize);
//        //
//        Laplacian(smoothed, laplace, CV_16S, 5);
//        convertScaleAbs(laplace, result, (sigma+1)*0.25);
        
        createTrackbar( "Sigma", "Laplacian", &sigma, 15, 0 );
        
        createTrackbar( "Kernel size Erosion:\n 2n +1", "cusframe", &erosion_size, max_kernel_size, Erosion );
        
        createTrackbar( "Kernel size Dilation:\n 2n +1", "cusframe", &dilation_size, max_kernel_size, Dilation );

        /// Create Trackbar to select Morphology operation
        createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", morph, &morph_operator, max_operator, Morphology_Operations );
    
        /// Create Trackbar to select kernel type
        createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", morph, &morph_elem, max_elem,Morphology_Operations );
    
        /// Create Trackbar to choose kernel size
        createTrackbar( "Kernel size:\n 2n +1", morph, &morph_size, max_kernel_size, Morphology_Operations );

        if(verbose)
            printf("Sigma value %d\n", sigma);
        
        /// Default start
        Erosion( 0, 0 );
        Dilation( 0, 0 );
//        Morphology_Operations( 0, 0 );

        medianBlur(diff, smoothed, ksize);
//        result = smoothed;

//        Mat rest, rest2;
//        Laplacian(smoothed, laplace, CV_16S, 5);
//        convertScaleAbs(laplace, rest, (sigma+1)*0.25);
//        
//        Laplacian(rest, laplace, CV_16S, 5);
//        convertScaleAbs(laplace, rest2, (sigma+1)*0.25);

        Laplacian(smoothed, laplace, CV_16S, 5);
        convertScaleAbs(laplace, result, (sigma+1)*0.25);
        
        imshow("diff", diff);

        Mat threshold_output;
//        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        
        /// Detect edges using Threshold
        threshold( result, threshold_output, thresh, 255, THRESH_BINARY );
        /// Find contours
        findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
        
        /// Approximate contours to polygons + get bounding rects and circles
        vector<vector<Point> > contours_poly( contours.size() );
        vector<Rect> boundRect( contours.size() );
        vector<Rect> boundRectOut( contours.size() );
        vector<Point2f>center( contours.size() );
        vector<float>radius( contours.size() );
        

        for( int i = 0; i < contours.size(); i++ )
        {
            approxPolyDP( Mat(contours[i]), contours_poly[i], 10, true );
            boundRect[i] = boundingRect( Mat(contours_poly[i]) );
            //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
        }
        
        mergeOverlappingBoxes(&boundRect, cusframe, &boundRectOut);
        
        /// Draw polygonal contour + bonding rects + circles
        Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
        {

            /** @brief rectangle(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0) */
//            double res = norm(boundRectOut[i].tl() - boundRectOut[i].br());//Euclidian distance
//            if(boundRectOut[i].height > 45 && boundRectOut[i].width > 45) {
//                rectangle( cusframe, boundRectOut[i].tl(), boundRectOut[i].br(), color, 2, 8, 0 );
                rectangle( cusframe, boundRectOut[i].tl(), boundRectOut[i].br(), yellow, 3, 8, 0 );
//            }
//            if( (int)radius[i] > 50) {
////                circle( cusframe, center[i], (int)radius[i], color, 2, 8, 0 );
//                circle(cusframe, center[i], 3, color, 20, 2, 0);
//            }
        }

        
        imshow("Laplacian", result);
//        imshow("draw", drawing);
//        imshow("diff", diff);
//        imshow("diff2", diff2);
        imshow("cusframe", cusframe);
        int c = waitKey(1);
        if( c == ' ' )
            smoothType = smoothType == GAUSSIAN ? BLUR : smoothType == BLUR ? MEDIAN : GAUSSIAN;
        if( c == 'q' || c == 'Q' || (c & 255) == 27 )
            break;
    }
    
    return 0;
}

/**  @function mergeOverlappingBoxes */
void mergeOverlappingBoxes(vector<Rect> *inputBoxes, Mat &image, vector<Rect> *outputBoxes)
{
    Mat mask = Mat::zeros(image.size(), CV_8UC1); // Mask of original image
    Size scaleFactor(-5,-5); // To expand rectangles, i.e. increase sensitivity to nearby rectangles --can be anything
    for (int i = 0; i < inputBoxes->size(); i++)
    {
        double euclianPointDistance = norm(inputBoxes->at(i).tl() - inputBoxes->at(i).br());
        /**  @brief filter boxes, ignore too small or big boxes */
        if((inputBoxes->at(i).height < 45 || inputBoxes->at(i).width < 45)
           || euclianPointDistance < 35 || euclianPointDistance > 600)
            continue;
        
        Rect box = inputBoxes->at(i) + scaleFactor;
        rectangle(mask, box, Scalar(255), CV_FILLED); // Draw filled bounding boxes on mask
    }
    
    imshow("Amask", mask);
    vector<vector<Point>> contoursOverlap;
    /**  @brief Find contours in mask
     If bounding boxes overlap, they will be joined by this function call */
    findContours(mask, contoursOverlap, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int j = 0; j < contoursOverlap.size(); j++)
    {
        outputBoxes->at(j) = boundingRect(contoursOverlap.at(j));
    }
}


/**  @function Erosion  */
void Erosion( int, void* )
{
    int erosion_type;
    if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
    else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
    else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

    Mat element = getStructuringElement( erosion_type, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );

    /// Apply the erosion operation
    erode( diff, diff, element );
    if(verbose)
        imshow( "Erosion", diff );
}

/** @function Dilation */
void Dilation( int, void* )
{
    int dilation_type;
    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

    Mat element = getStructuringElement( dilation_type, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    dilate( diff, diff, element );
    if(verbose)
        imshow( "Dilation", diff );
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
    morphologyEx( diff, diff, operation, element );
    imshow( morph, diff );
}
