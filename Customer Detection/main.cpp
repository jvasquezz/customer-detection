////*******************surf.cpp******************//
////********** SURF implementation in OpenCV*****//
////**loads video from webcam, grabs frames computes SURF keypoints and descriptors**//  //** and marks them**//
//
////****author: achu_wilson@rediffmail.com****//
//#include "this.dependencies.hpp"
//#include "opencv2/xfeatures2d.hpp"
//
////#include "opencv2/legacy/legacy.hpp"
//
//int main() {
//    Mat image1, outImg1, image2, outImg2;
//    VideoCapture cap;
//    cap.open(0);
//    
//    // vector of keypoints
//    vector<KeyPoint> keypoints1, keypoints2;
//    
//    cap >> image1;
//    cap >> image2;
//    // Read input images
////    image1 = imread("C://Google-Logo.jpg",0);
////    image2 = imread("C://Alex_Eng.jpg",0);
//    
//    SurfFeatureDetector surf(2500);
//    surf.detect(image1, keypoints1);
//    surf.detect(image2, keypoints2);
//    drawKeypoints(image1, keypoints1, outImg1, Scalar(255,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//    drawKeypoints(image2, keypoints2, outImg2, Scalar(255,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//    
//    namedWindow("SURF detector img1");
//    imshow("SURF detector img1", outImg1);
//    
//    namedWindow("SURF detector img2");
//    imshow("SURF detector img2", outImg2);
//    
//    SurfDescriptorExtractor surfDesc;
//    Mat descriptors1, descriptors2;
//    surfDesc.compute(image1, keypoints1, descriptors1);
//    surfDesc.compute(image2, keypoints2, descriptors2);
//    
////    BFMatcher<L2<float>> matcher;
//    BFMatcher matcher;
////    BruteForceMatcher<L2<float>> matcher;
//    vector<DMatch> matches;
//    matcher.match(descriptors1,descriptors2, matches);
//    
//    nth_element(matches.begin(), matches.begin()+24, matches.end());
//    matches.erase(matches.begin()+25, matches.end());
//    
//    Mat imageMatches;
//    drawMatches(image1, keypoints1, image2, keypoints2, matches, imageMatches, Scalar(255,255,255));
//    
//    namedWindow("Matched");
//    imshow("Matched", imageMatches);
//    
//    cv::waitKey(0);
//    return 0;
//    
//}


////#include <stdio.h>
////#include "opencv2/features2d/features2d.hpp"
////#include "opencv2/highgui/highgui.hpp"
////#include "opencv2/imgproc/imgproc_c.h"
//
//using namespace std;
//int main(int argc, char** argv)
//{
//    CvMemStorage* storage = cvCreateMemStorage(0);
//    cvNamedWindow("Image", 1);
//    int key = 0;
//    static CvScalar red_color[] ={0,0,255};
//    CvCapture* capture = cvCreateCameraCapture(0);
//    CvMat* prevgray = 0, *image = 0, *gray =0;
//    while( key != 'q' )
//    {
//        int firstFrame = gray == 0;
//        IplImage* frame = cvQueryFrame(capture);
//        if(!frame)
//            break;
//        if(!gray)
//        {
//            image = cvCreateMat(frame->height, frame->width, CV_8UC1);
//        }
//        //Convert the RGB image obtained from camera into Grayscale
//        cvCvtColor(frame, image, CV_BGR2GRAY);
//        //Define sequence for storing surf keypoints and descriptors
//        CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
//        int i;
//        
//        //Extract SURF points by initializing parameters
//        CvSURFParams params = cvSURFParams(500, 1);
//        cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );
//        printf("Image Descriptors: %d\n", imageDescriptors->total);
//        
//        //draw the keypoints on the captured frame
//        for( i = 0; i < imageKeypoints->total; i++ )
//        {
//            CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, i );
//            CvPoint center;
//            int radius;
//            center.x = cvRound(r->pt.x);
//            center.y = cvRound(r->pt.y);
//            radius = cvRound(r->size*1.2/9.*2);
//            cvCircle( frame, center, radius, red_color[0], 1, 8, 0 );
//        }
//        cvShowImage( "Image", frame );
//        
//        cvWaitKey(30);
//    }
//    cvDestroyWindow("Image");
//    return 0;
//}



#include "this.dependencies.hpp"

/** Global variables */
bool verbose = false;
Mat baseframe;
Mat untouch_frame;
Mat last_template;// = Mat(1,1, CV_64F, 0.0);
int flag = 1;
Mat sketchMat;
vector<Point2d> prevPoints;
Scalar stain[10];
vector<Point2d> c;
int iota = 0;
int mu = 0;

class Customer
{
public:
    int id;
    vector<MatND> histog;
    double time_introduced;
    double last_recorded_time;
};

vector<Customer> customer;

void instantiate_newCustomers( MatND* newHistogram )
{
    Customer tmp;
    tmp.id = mu;
    tmp.histog.push_back(*newHistogram);
    customer.push_back(tmp);
    mu++;
}

void instantiate_newCustomers( vector<MatND>* vec_histograms )
{
    for (int each = 0; each < vec_histograms->size(); each++ ) {
        
        Customer tmp;
        tmp.id = mu;
        tmp.histog.push_back(vec_histograms->at(each));
        customer.push_back(tmp);
        mu++;
    }
}

/**  @function CalcHistogram of given image */
void CalcHistogram( vector<MatND>* destination, vector<Mat> originals )
{

    for (int i = 0; i < originals.size(); i++) {
        Mat hsv_histogram;
        
        /// Convert to HSV
        cvtColor(originals[i], hsv_histogram, COLOR_BGR2HSV);
        
        /// Using 50 bins for hue and 60 for saturation
        int h_bins = 50; int s_bins = 60;
        int histSize[] = { h_bins, s_bins };
        
        // hue varies from 0 to 179, saturation from 0 to 255
        float h_ranges[] = { 0, 180 };
        float s_ranges[] = { 0, 256 };
        
        const float* ranges[] = { h_ranges, s_ranges };
        
        // Use the o-th and 1-st channels
        int channels[] = { 0, 1 };
        
        /// Histograms
        MatND HIST_CONVERT;
        
        /// Calculate the histograms for the HSV images
        calcHist( &hsv_histogram, 1, channels, Mat(), HIST_CONVERT, 2, histSize, ranges, true, false );
        normalize( HIST_CONVERT, HIST_CONVERT, 0, 1, NORM_MINMAX, -1, Mat() );

        destination->push_back(HIST_CONVERT);
    }
}

void CompareHistogram ( vector<MatND> kappa )
{

    for (int i = 0; i < kappa.size(); i++)
    {
        double customer_kappa = 0.0;
        for ( int k = 0; k < customer.size(); k++ )
        {
            int pos_last_histogram = (int)customer[i].histog.size() - 1;
            if (compareHist(customer[k].histog[pos_last_histogram], kappa[i], CV_COMP_CORREL) > .95)
            {
                customer_kappa = compareHist(customer[i].histog[pos_last_histogram], kappa[k], CV_COMP_CORREL);
                customer[k].histog.push_back(kappa[i]);
                break;
            }
        }
        if (customer_kappa < .95)
        {
            instantiate_newCustomers(&kappa[i]);
        }      
    }
    
//    double epsilon_iota = 0.0;
//    cout << "Beginning comparison of histogram with new set of objects\n\n";
//    /// Apply the histogram comparison methods
//    for( int i = 0; i < kappa.size(); i++ )
//    {
//        cout << "Start i = kappa[" << i << "]\n\n";
//        for ( int k = 0; k < gamma.size(); k++ )
//        {
//            if (epsilon_iota < compareHist(kappa[i], gamma[k], CV_COMP_CORREL)) {
//                
//                epsilon_iota = compareHist(kappa[i], gamma[k], CV_COMP_CORREL);
//            }
////            cout << "vs k = kappa[" << k << "]\n";
////            for (int lambda = 0; lambda < 4; lambda ++)
////            {
////                int compare_method = lambda;
////                double epsilon_iota = compareHist( kappa[i], gamma[k], compare_method );
////                cout << "Method " << compare_method << ": " << epsilon_iota;
////                cout << '\n';
////            }
//            cout << "\n";
//        }
//        cout << "\n";
//    }

}



/**  @main */
int main(int argc, const char * argv[]) {
    /** Default values for @function encapsulateObjects */
    int sigma = 3;
    int smoothType = MEDIAN;
    ///smoothType = BILATERAL_FILTER;
    int ksize = (sigma*5)| 1;
    
    // insert code here...
    VideoCapture cap;
    String capstone_dir = "/Users/drifter/Desktop/capstone/";
    cap.open(capstone_dir+"SEGMENTA_720P_20FPS.mp4");
    /** more vid files:
     @a SEGMENTA_720P_20FPS.mp4
     @b 10FPS.mp4
     @b ver.mp4
     @c Untitled.mp4
     @e 30FPS.mp4
     @a 6FPS.mp4   */

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

    line_print.copyTo(sketchMat);
    
    stain[0] = paint_yellow;
    stain[1] = paint_red;
    stain[2] = paint_lightGREEN;
    stain[3] = paint_lightBLUE;
    stain[4] = paint_lightORANGE;
    stain[5] = paint_ade004;
    stain[6] = paint_blue;

    vector<MatND> past_vHistograms;
    
    /**  @brief main loop */
    for(;;) {
//        for(int i = 0; i < 3; i++) {
//            cap >> frame;
//        }
        cap >> frame;
        if (!frame.data)
            return -1;
        
        
        /**  @brief set regions of interest (ROI) to scan for objects,  */
        Mat conveyorbelt = frame(MOLD_CONVEYOR_BELT);
        customer_line = frame(MOLD_CUSTOMERLINE_WIDE);
        customer_line.copyTo(untouch_frame);
        
        /**  @function @brief
         encapsulateObjects(Mat* instanceROI, Mat* baseIMG, int targetObject, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE)*/
        encapsulateObjects(&conveyorbelt, &belt_print, OBJECT_ITEM, ksize, sigma, thresh, smoothType);
        vector<Mat> customersDetected = encapsulateObjects(&customer_line, &line_print, OBJECT_CUSTOMER, ksize, sigma, thresh, smoothType);
        
        vector<MatND> vec_histograms;
        CalcHistogram( &vec_histograms, customersDetected );
        
        
        
        if (past_vHistograms.size()) { /**  @note if there are objects detected in previous frame */
            CompareHistogram( vec_histograms );
        } else {
            instantiate_newCustomers( &vec_histograms );
        }
        
        /** Update sigma using trackbar @note change blur method using spacebar, @see smoothType */
        createTrackbar( "Sigma", "Laplacian", &sigma, 15, 0 );;
        
        if(verbose)
            printf("Sigma value %d\n", sigma);
        
        imshow("customer_line", customer_line);
        imshow("sketchMat", sketchMat);
        
        int c = waitKey(1);
        if( c == ' ' )
        {
            smoothType = smoothType == GAUSSIAN ? BLUR : smoothType == BLUR ? MEDIAN : smoothType == MEDIAN ? BILATERAL_FILTER : GAUSSIAN;
//            smoothType = smoothType == GAUSSIAN ? BLUR : smoothType == BLUR ? MEDIAN : GAUSSIAN;
            if(verbose) printf("smoothType %d\n1.Gaussian 2. Blur 3. Median:\n", smoothType+1);
        }
        if( c == 'q' || c == 'Q' || (c & 255) == 27 )
            break;
        
        past_vHistograms.swap( vec_histograms );
    }   /*end main loop */
    
    return 0;
}


/***  @function encapsulateObjects
         @return array of croppedObjects i.e customers, other objects */
vector<Mat> encapsulateObjects( Mat *instanceROI, Mat *baseROI, int METHOD, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE )
{
    Scalar COLOR;
    Mat currentgray, basegray, differs;
    cvtColor(*instanceROI, currentgray, COLOR_BGR2GRAY);
    cvtColor(*baseROI, basegray, COLOR_BGR2GRAY);

    if (METHOD == OBJECT_CUSTOMER)
    { /**  Substract from base image the current one, detects/shows people */
        differs = basegray - currentgray;
        COLOR = paint_yellow;
    }
    else if (METHOD == OBJECT_ITEM)
    {
        differs = currentgray - basegray;
        COLOR = paint_lightORANGE;
    }
    differs = differs < 60;
    
    Mat smoothed, laplace, result;
    if(SMOOTHTYPE == MEDIAN)
        medianBlur(differs, smoothed, KSIZE);
    else if(SMOOTHTYPE == GAUSSIAN)
        GaussianBlur(differs, smoothed, Size(KSIZE, KSIZE), SIGMA, SIGMA);
    else if(SMOOTHTYPE == BILATERAL_FILTER) /* bilateral filter smooths and sharpens edges */
    /** @brief bilateralFilter is computationally heavy, keep d=5 for real-time, sigmas can be same (ranges 10-200)*/
        bilateralFilter(differs, smoothed, 5, 50, 50);
    else /* BLUR */
        blur(differs, smoothed, Size(KSIZE, KSIZE));
    

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
        if( (radius[i] > 35) && (METHOD == OBJECT_CUSTOMER) )
        { //circle( *instanceROI, center[i], (int)radius[i]/2, blue, 1, 8, 0 );
//            circle( *instanceROI, center[i], 2, lightGREEN, 2, 8, 0);
//            circle(*instanceROI, center[i], 8, red, 2, 4, 0);
            ;
        }
    }
    

    /**   @brief merge overlapping boxes,  @return number of boxes
     @note might be better to merge contained boxes only i.e
     @note A is a subset of B if every element of A is contained in B */
    int overlapContours_size = mergeOverlappingBoxes(&boundRect, *instanceROI, &boundRectOut, METHOD);
    

    vector<Mat> croppedObject;
    
    /**  Draw polygonal contour + bonding rects + circles */
    for( int i = 0; i< overlapContours_size/*contours_eo.size()*/; i++ )
    {
//        matchTemplate(untouch_frame, croppedObject[i], result, CV_TM_CCOEFF_NORMED);
        
        /**  @brief rectangle(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0) */
        rectangle( *instanceROI, boundRectOut[i].tl(), boundRectOut[i].br(), COLOR, 2.0, 4, 0 );


        if (METHOD == OBJECT_CUSTOMER)
        {
            /**  @brief c center of rectangle, saves coordinates */
            Rect r(boundRectOut[i]);
            c.push_back(Point2d(r.x + r.width / 2, r.y + r.height / 2));
            // c is center of rect
            char coordi[500];
            /**  @brief coordinates on base 5 */
            sprintf(coordi, "[%d,%d]", (int)(100*c[i].x)/1000, (int)(100*c[i].y)/1000);
            putText(*instanceROI, "Party123586", c[i], 6, .5, WHITE);
//            putText(*instanceROI, coordi, c[i], 6, .5, WHITE);
        }
        
        croppedObject.push_back( untouch_frame(boundRectOut[i]) );
    }
    
    
//    double distanceArray[overlapContours_size];
    vector<Point2d> a;
    vector<Point2d> b;
    int tmpb = 0;

    if ( !prevPoints.empty() && METHOD == OBJECT_CUSTOMER )
    {
//        printf("prevPoints.size and c,size,  %lu v %lu\n ", prevPoints.size(), c.size() );
        if ( prevPoints.size() == c.size() )
        {
            for (int i = 0; i < overlapContours_size; i++)
            {
                double min = 100000;
                for (int j = 0; j < overlapContours_size; j++)
                {
//                    printf("norm euclidean distance %.2f at (%d, %d)\n", norm( prevPoints[i] - c[j]), i, j );
                    if ( min > (norm( prevPoints[i] - c[j]))  )
                    {
                        min = norm( prevPoints[i] - c[j] );
//                        printf("min at this point is %.2f (%d,%d)\n", min, i, j );
                        tmpb = j;
                    }
                }
//                printf("min at this point is %.2f (%d,%d)\n", min, tmpa, tmpb );
//                printf("\n\n");
                a.push_back(prevPoints[i]);
                b.push_back(c[tmpb]);
            }
            
            
//            if ( iota++  > 3 ) {
//                iota = 0;
                for (int i = 0; i < overlapContours_size; i++ )
                {
                    //                cout << "lines drawn " << i;
                    line(sketchMat, a[i], b[i], stain[i], 2, 4);
                }
//            }
        }
    }
    

    
//                distanceArray[i] = norm( prevPoints[i]-c[i] );
//                if ( distanceArray[tmp] > distanceArray[i] )
//                    tmp  = i;
//                prevPoints[i]
//            }
//            prevPoints[i]
//    }
    
    
    
    /**  @brief distance between points */
//    int tmp = 0;
//    double distanceArray[overlapContours_size];
//    for (int i = 0; i < overlapContours_size; i++ ) {
//        distanceArray[i] = norm( prevPoints[i]-c[i] );
//        if ( distanceArray[tmp] > distanceArray[i] )
//            tmp  = i;
//    }
    
    /**  @brief print coordinates on frame */
//    char coordi[500];
//    /**  @brief coordinates on base 5 */
//    sprintf(coordi, "[%d,%d]", (int)(100*c[tmp].x)/1000, (int)(100*c[tmp].y)/1000);
//    putText(sketchMat, coordi, c[tmp], 6, .5, paint_purple);
//
//    
//    /**  @brief  draws line to consequent point */
//    if (METHOD == OBJECT_CUSTOMER)
//    {
////        double res = cv::norm(prevPoints-c[tmp]);//Euclidian distance
////        if (prevPoints != c[tmp]) {
//            line(sketchMat, prevPoints[tmp], c[tmp], paint_ade004);
////        }
//        prevPoints[tmp] = c[tmp];
//    }
//    
    /**  @brief set prevPoints to just used points */


    
    if ( METHOD == OBJECT_CUSTOMER ) {
//        for (unsigned i=0; i<prevPoints.size(); i++)
//            std::cout << "before swap" << prevPoints[i];
//        std::cout << '\n';
            prevPoints.clear();
            prevPoints.swap(c);
            c.clear();
    }
    
    
    for (int i = 0; i < overlapContours_size; i++ )
    {
        ;//        SURF
    }
    
    /* matchTemplate(InputArray image, InputArray templ, OutputArray result, int method) */
    if ( overlapContours_size > 1 ) {
//        prevPoints[tmp] = c[tmp+1];
//        exit(20);
//        if (flag == 1) {
//            croppedObject[1].copyTo(last_template);
//            flag = 0;
//        }
//        croppedObject[1].copyTo(last_template);
//        matchTemplate(untouch_frame, last_template, result, CV_TM_CCOEFF_NORMED);
//        normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
//        imshow("matchTemplate", result);
//        imshow("windname", croppedObject[1]);
        ;
    }
    

    
//    char windname[100];
//    for( int ap = 0; ap < overlapContours_size; ap++ ) {
//        sprintf(windname, "croppedObject %d", ap);
//        namedWindow(windname);
//        imshow(windname, croppedObject[ap]);
//    }
    return croppedObject;
}


/**  @function mergeOverlappingBoxes */
int mergeOverlappingBoxes(vector<Rect> *inputBoxes, Mat &image, vector<Rect> *outputBoxes, int MOCI/*(method object customer/item)*/)
{
    Mat mask = Mat::zeros(image.size(), CV_8UC1); // Mask of original image
    Size scaleFactor(-10,-10); // To expand rectangles, i.e. increase sensitivity to nearby rectangles --can be anything
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
    
    imshow("Amask", mask);
    vector<vector<Point>> contoursOverlap;
    /**  @brief Find contours in mask
     If bounding boxes overlap, they will be joined by this function call */
    findContours(mask, contoursOverlap, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int j = 0; j < contoursOverlap.size(); j++)
    {
        outputBoxes->at(j) = boundingRect(contoursOverlap.at(j));
    }
    
    return (int)contoursOverlap.size();
}

