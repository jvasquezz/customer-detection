
#include "this.dependencies.hpp"

/** Global variables */
bool verbose = true;
bool verbose2 = false;

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
const int LINE_OBJ_CREATION[2] = {110, 107} ;

class Customer
{
public:
    int id;
    vector<MatND> histog;
    vector<Point2d> position;
    double time_introduced;
    double last_recorded_time;
};

vector<Customer> customer;

void instantiate_newCustomers( Customer newCustomer )
{
    newCustomer.id = mu;
    customer.push_back(newCustomer);
    mu++;
}

void instantiate_newCustomers( vector<Customer> vecCustomer )
{
    for (int each = 0; each < vecCustomer.size(); each++ )
    {
        instantiate_newCustomers(vecCustomer[each]);
    }
}

/**  @function CalcHistogram of given image 
 @brief returns the histograms of a given array of images (Mats)*/
void CalcHistogram( vector<Customer>* destination, vector<Customer> originals )
{

    for (int i = 0; i < originals.size(); i++) {
        Mat hsv_histogram;
        
        int pos_last_histogram = (int)originals[i].histog.size() - 1;
        /// Convert to HSV
        cvtColor(originals[i].histog[pos_last_histogram], hsv_histogram, COLOR_BGR2HSV);
        
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
        Customer HIST_CONVERT;

        MatND placeholdMatND;
        HIST_CONVERT.id = originals[i].id;
        HIST_CONVERT.position = originals[i].position;
        /// Calculate the histograms for the HSV images
        calcHist( &hsv_histogram, 1, channels, Mat(), placeholdMatND, 2, histSize, ranges, true, false );
        HIST_CONVERT.histog.push_back(placeholdMatND);
        normalize( HIST_CONVERT.histog.back(), HIST_CONVERT.histog.back(), 0, 1, NORM_MINMAX, -1, Mat() );

        
        destination->push_back(HIST_CONVERT);
    }
}

void CompareHistogram ( vector<Customer> kappa )
{
    if (customer.size() == 0)
    {
        if ((int)(100*kappa.back().position.back().x)/1000 < LINE_OBJ_CREATION[0]
            && (int)(100*kappa.back().position.back().x)/1000 > LINE_OBJ_CREATION[1])
        {
            cout << kappa.back().position.back().x << " KAPA opsition when object is created \n\n";
            instantiate_newCustomers(kappa.back());
        }
        else
        {
            return;
        }
    }
    
    int sav = -1;
    int TAU = (int)customer.size();
    double customer_kappa = 0.0;
    int manytopop = 0;
    for (int i = 0; i < kappa.size(); i++)
    {
        customer_kappa = 10000;
        double correl_histogram = 0.0;
        for ( int k = 0; k < TAU; k++ )
        {
            if (customer_kappa > norm(customer[k].position.back() - kappa[i].position.back()))
            {
                correl_histogram = compareHist(customer[k].histog.back(), kappa[i].histog.back(), CV_COMP_CORREL);
                customer_kappa = norm(customer[k].position.back() - kappa[i].position.back());
                sav = k;
            } /* end if */
        } /* end inner loop */
        

        /**  @brief object creation */
        if ((int)(100*kappa[i].position.back().x)/1000 < LINE_OBJ_CREATION[0]
            && (int)(100*kappa[i].position.back().x)/1000 > LINE_OBJ_CREATION[1]
            && correl_histogram < .8)
        {
            instantiate_newCustomers(kappa[i]);
        }
        else if ((int)(100*kappa[i].position.back().x)/1000 > 30)
        {
            customer[sav].histog.push_back(kappa[i].histog.back());
            customer[sav].position.push_back(kappa[i].position.back());
        }
        
//        double euclidean_dist = norm(customer[sav].position.back() - kappa[i].position.back());
//        if (customer_kappa > .8 && euclidean_dist < 300 )
//        {
//            customer[sav].histog.push_back(kappa[i].histog.back());
//            customer[sav].position.push_back(kappa[i].position.back());
//        }
//        else if ( euclidean_dist < 150 )
//        {
//            customer[sav].histog.push_back(kappa[i].histog.back());
//            customer[sav].position.push_back(kappa[i].position.back());
//        }
//        else if ((int)(100*kappa[i].position.back().x)/1000 < 110)
//        {
//            cout << kappa[i].position.back().x << " KAPA opsition when object is created \n\n";
//            Customer tempKap = kappa[i];
//            instantiate_newCustomers(tempKap);
//        }
        
    } /* end outer loop */
    if (manytopop) {;
//        customer.pop_back();
    }
//    for (int i = 0; i < manytopop; i++) {
    
//    }


} /*end CompareHistogram */



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
    cap.open(capstone_dir+"10FPS.mp4");
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
    line_print = baseframe(MOLD_CUSTOMERLINE);
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
//        for(int i = 0; i < 2; i++) {
//            cap >> frame;
//        }
        cap >> frame;
        if (!frame.data)
            return -1;
        
//
////        Mat mask;
//        Mat mask = Mat::zeros( frame.rows, frame.cols, CV_8UC1 );
//        inRange(frame, paint_red, Scalar(127,127,255), mask);
//
//        imshow("mask2", mask);
////        Mat transparent;
////        copy(frame, transparent, mask);
////
////        imshow("transparent", transparent);
////        
//        Vec3b colorRef(255,0,0); // for ''pure'' blue
//        Vec3b paint_RED(0,0,255);
//
//        for (int i = 0; i < frame.rows; i++ )
//        {
//            for (int k = 0; k < frame.cols; k++ )
//            {
//                if(frame.at<Vec3b>(Point(k,i)) == paint_RED)
////                if(frame.at<Scalar>(k,i) < frame.at<Scalar>(k,i))
//                {
//                    mask.at<Vec3b>(Point(k,i)) = Vec3b(255,255,255);
//                }
//            }
//        }
//
//        imshow("mask", mask);
//        
        
        /**  @brief set regions of interest (ROI) to scan for objects,  */
        Mat conveyorbelt = frame(MOLD_CONVEYOR_BELT);
        customer_line = frame(MOLD_CUSTOMERLINE);
        customer_line.copyTo(untouch_frame);
        Mat displays = frame(MOLD_CUSTOMERLINE_WIDE);
        
        /**  @function @brief
         encapsulateObjects(Mat* instanceROI, Mat* baseIMG, int targetObject, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE)*/
        encapsulateObjects(&conveyorbelt, &belt_print, OBJECT_ITEM, ksize, sigma, thresh, smoothType);
        vector<Customer> customersDetected = encapsulateObjects(&customer_line, &line_print, OBJECT_CUSTOMER, ksize, sigma, thresh, smoothType);
        
        vector<Customer> vec_customers;
        CalcHistogram( &vec_customers, customersDetected );
        
        
        if (vec_customers.size())  /**  @note if there are customers already detected */
        {
            CompareHistogram( vec_customers );
        }
        
        for (int i = 0; i < customer.size(); i++)  /**  @note puts labels on Customer */
        {
            char identifier[100];
            sprintf(identifier, "C%d", customer[i].id);
            putText(customer_line, identifier, customer[i].position.back(), 3, 1, WHITE, 1.5, 40);
        }
        
        
        /** Update sigma using trackbar @note change blur method using spacebar, @see smoothType */
        createTrackbar( "Sigma", "Laplacian", &sigma, 15, 0 );;
        
        if(verbose)
            printf("Sigma value %d\n", sigma);
        
//        imshow("customer_line", customer_line);
        imshow("customer_line", displays);
        if (verbose2)
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
        
    }   /*end main loop */
    
    return 0;
}


/***  @function encapsulateObjects
         @return array of croppedObjects i.e customers, other objects */
vector<Customer> encapsulateObjects( Mat *instanceROI, Mat *baseROI, int METHOD, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE )
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
    
    if(verbose2)
        imshow("differs39", differs);
    
    
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

    if(verbose2)
        imshow("result@#3", result);
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
    

    vector<Customer> croppedObject;
    Rect r;
    /**  Draw polygonal contour + bonding rects + circles */
    for( int i = 0; i< overlapContours_size/*contours_eo.size()*/; i++ )
    {
//        matchTemplate(untouch_frame, croppedObject[i], result, CV_TM_CCOEFF_NORMED);
        Customer tmp;
        /**  @brief rectangle(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0) */
        rectangle( *instanceROI, boundRectOut[i].tl(), boundRectOut[i].br(), COLOR, 2.0, 4, 0 );


        if (METHOD == OBJECT_CUSTOMER)
        {
            /**  @brief c center of rectangle, saves coordinates */
            r = Rect(boundRectOut[i]);
            c.push_back(Point2d(r.x + r.width / 2, r.y + r.height / 2));
            // c is center of rect
            char coordi[100];
            /**  @brief coordinates on base 5 */
            sprintf(coordi, "[%d,%d]", (int)(100*c[i].x)/1000, (int)(100*c[i].y)/1000);
            putText(*instanceROI, coordi, c[i], 6, .5, WHITE);

        }
        tmp.histog.push_back(untouch_frame(boundRectOut[i]));
        tmp.position.push_back(Point2d(r.x + r.width / 2, r.y + r.height / 2));
        croppedObject.push_back(tmp);

    }
//    croppedObject[i].histog.push_back( untouch_frame(boundRectOut[i]) );
//    croppedObject[i].position.push_back(Point2d(r.x + r.width / 2, r.y + r.height / 2));
    
    
    
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
            
            
            for (int i = 0; i < overlapContours_size; i++ )
            {
                line(sketchMat, a[i], b[i], stain[i], 2, 4);
            }
        }
    }
    
    
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
//        double euclianPointDistance = norm(inputBoxes->at(i).tl() - inputBoxes->at(i).br());
        /**  @brief filter boxes, ignore too small or big boxes */
        switch (MOCI) {
            case OBJECT_CUSTOMER:
                if((inputBoxes->at(i).height * inputBoxes->at(i).width) < 35000)
                    continue;
                break;
            case OBJECT_ITEM:
                break;
        } /**  end switch */

        Rect box = inputBoxes->at(i) + scaleFactor;
        rectangle(mask, box, Scalar(255), CV_FILLED); // Draw filled bounding boxes on mask
    }
    

    if (verbose2)
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

