#include "this.dependencies.hpp"

/** Global vars */
bool verbose = false;
Mat baseframe;

int main(int argc, const char * argv[]) {
    /** Default values for @function encapsulate_objects */
    int sigma = 3;
    int smoothType = MEDIAN;
    int ksize = (sigma*5)| 1;
    
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

    baseframe = imread(BASEFRAME_DIR);

    namedWindow( "Laplacian", 0 );
    
    /**  mold: A hollow form or matrix for shaping a segment from a frame */
    /**  @constructor Rect
    Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height) */
    Rect MOLD_CUSTOMERLINE_WIDE(0, baseframe.rows/5,baseframe.cols,baseframe.rows/2);
    Rect MOLD_CUSTOMERLINE(0,baseframe.rows/3.3,baseframe.cols,baseframe.rows/3.3);
    Rect MOLD_CONVEYOR_BELT(baseframe.cols/2.61,baseframe.rows/4.7,250,120);
    
    int thresh = 100;
    Mat line_print, belt_print, frame, customer_line;
    line_print = baseframe(MOLD_CUSTOMERLINE_WIDE);
    belt_print = baseframe(MOLD_CONVEYOR_BELT);

    /**  @brief main loop */
    for(;;) {
        cap >> frame;
        if (!frame.data)
            return -1;
            
        /**  @brief set regions of interest (ROI) to scan for objects,  */
        Mat conveyorbelt = frame(MOLD_CONVEYOR_BELT);
        customer_line = frame(MOLD_CUSTOMERLINE_WIDE);
        
        /**  @function @brief
        encapsulate_objects( Mat *areaOI, Mat *baseROI, int ObjectToDetect (0-2), int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE ) */
        encapsulate_objects(&conveyorbelt, &belt_print, OBJECT_ITEM, ksize, sigma, thresh, smoothType);
        encapsulate_objects(&customer_line, &line_print, OBJECT_CUSTOMER, ksize, sigma, thresh, smoothType);
        
        /** Update sigma using trackbar @note change blur method using spacebar, @see smoothType */
        createTrackbar( "Sigma", "Laplacian", &sigma, 15, 0 );;
        
        if(verbose)
            printf("Sigma value %d\n", sigma);
        
        imshow("customer_line", customer_line);
        int c = waitKey(1);
        if( c == ' ' )
        {
            smoothType = smoothType == GAUSSIAN ? BLUR : smoothType == BLUR ? MEDIAN : GAUSSIAN;
            if(verbose) printf("smoothType %d\n1.Gaussian 2. Blur 3. Median:\n", smoothType+1);
        }
        if( c == 'q' || c == 'Q' || (c & 255) == 27 )
            break;
    }
    
    return 0;
}


/** @function encapsulate_objects */
void encapsulate_objects( Mat *areaOI, Mat *baseROI, int METHOD, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE ) {
    Scalar COLOR;
    Mat currentgray, basegray, differs;
    cvtColor(*areaOI, currentgray, COLOR_BGR2GRAY);
    cvtColor(*baseROI, basegray, COLOR_BGR2GRAY);

    if (METHOD == OBJECT_CUSTOMER)
    { /**  Substract from base image the current one, detects/shows people */
        differs = basegray - currentgray;
        COLOR = yellow;
    }
    else if (METHOD == OBJECT_ITEM)
    {
        differs = currentgray - basegray;
        COLOR = lightORANGE;
    }
    differs = differs < 60;
    
    Mat smoothed, laplace, result;
    if(SMOOTHTYPE == GAUSSIAN)
        GaussianBlur(differs, smoothed, Size(KSIZE, KSIZE), SIGMA, SIGMA);
    else if(SMOOTHTYPE == BLUR)
        blur(differs, smoothed, Size(KSIZE, KSIZE));
    else
        medianBlur(differs, smoothed, KSIZE);

    /**  @brief Laplacian(InputArray src, OutputArray dst, int ddepth) */
    Laplacian(smoothed, laplace, CV_16S, 5);
    convertScaleAbs(laplace, result, (SIGMA+1)*0.25);
    
    Mat threshold_output;
    vector<vector<Point> > contours_eo;
    vector<Vec4i> hierarchy;
    
    /// Detect edges using Threshold
    threshold( result, threshold_output, THRESH, 255, THRESH_BINARY );
    /// Find contours
    findContours( threshold_output, contours_eo, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours_eo.size() );
    vector<Rect> boundRect( contours_eo.size() );
    vector<Rect> boundRectOut( contours_eo.size() );
    vector<Point2f>center( contours_eo.size() );
    vector<float>radius( contours_eo.size() );
    
    for( int i = 0; i < contours_eo.size(); i++ )
    {
        approxPolyDP( Mat(contours_eo[i]), contours_poly[i], 10, true );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }
    
    /**   @brief merge overlapping boxes, 
     @note might be better to use function to merge contained boxes only i.e
     @note A is a subset of B if every element of A is contained in B */
    mergeOverlappingBoxes(&boundRect, *areaOI, &boundRectOut, METHOD);
    
    /**  Draw polygonal contour + bonding rects + circles */
    for( int i = 0; i< contours_eo.size(); i++ )
    {
        /** @brief rectangle(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0) */
        rectangle( *areaOI, boundRectOut[i].tl(), boundRectOut[i].br(), COLOR, 2.0, 4, 0 );

        if( ((int)radius[i] > 50) && (METHOD == OBJECT_CUSTOMER) )
        { //circle( *areaOI, center[i], (int)radius[i]/2, blue, 1, 8, 0 );
            circle( *areaOI, center[i], 2, lightGREEN, 2, 8, 0);
        }
    }
}


/**  @function mergeOverlappingBoxes */
void mergeOverlappingBoxes(vector<Rect> *inputBoxes, Mat &image, vector<Rect> *outputBoxes, int MOCI/*(method object customer/item)*/)
{
    Mat mask = Mat::zeros(image.size(), CV_8UC1); // Mask of original image
    Size scaleFactor(-5,-5); // To expand rectangles, i.e. increase sensitivity to nearby rectangles --can be anything
    for (int i = 0; i < inputBoxes->size(); i++)
    {
        double euclianPointDistance = norm(inputBoxes->at(i).tl() - inputBoxes->at(i).br());
        /**  @brief filter boxes, ignore too small or big boxes */
        switch (MOCI) {
            case OBJECT_CUSTOMER:
                if((inputBoxes->at(i).height < 45 || inputBoxes->at(i).width < 45)
                   || euclianPointDistance < 35 || euclianPointDistance > 600)
                    continue;
                break;
            case OBJECT_ITEM:
                break;
        }

        Rect box = inputBoxes->at(i) + scaleFactor;
        rectangle(mask, box, Scalar(255), CV_FILLED); // Draw filled bounding boxes on mask
    }
    
//    imshow("Amask", mask);
    vector<vector<Point>> contoursOverlap;
    /**  @brief Find contours in mask
     If bounding boxes overlap, they will be joined by this function call */
    findContours(mask, contoursOverlap, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int j = 0; j < contoursOverlap.size(); j++)
    {
        outputBoxes->at(j) = boundingRect(contoursOverlap.at(j));
    }
}

