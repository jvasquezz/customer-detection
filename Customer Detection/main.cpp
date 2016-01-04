
#include "this.dependencies.hpp"
#include <array>

/** Global variables */
bool verbose = 0;
bool verbose2 = 0;
bool flag = true;

Mat baseframe;
Mat untouch_frame;
Mat last_template;// = Mat(1,1, CV_64F, 0.0);

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
    bool track;
    vector<MatND> histog;
    vector<Point2d> position;
    double time_introduced;
    double last_recorded_time;
};

/** List of Customers to track @see class Customer  */
deque<Customer> track_customer;

/**  @function MAIN */
int main() {
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
    
    /**  mold: A hollow form or matrix for shaping a segment from frame */
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
        
        
        /**  @brief set regions of interest (ROI) to scan for objects,  */
        Mat conveyorbelt = frame(MOLD_CONVEYOR_BELT);
        customer_line = frame(MOLD_CUSTOMERLINE);
        customer_line.copyTo(untouch_frame);
        Mat displays = frame(MOLD_CUSTOMERLINE_WIDE);
        
        /**  @function @brief
         encapsulateObjects(Mat* instanceROI, Mat* baseIMG, int targetObject, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE)*/
        encapsulateObjects(&conveyorbelt, &belt_print, OBJECT_ITEM, ksize, sigma, thresh, smoothType);
        deque<Customer> new_detected = encapsulateObjects(&customer_line, &line_print, OBJECT_CUSTOMER, ksize, sigma, thresh, smoothType);
        
        linkCustomers(&new_detected, &track_customer);
        
//        vector<Customer> vec_customers;
//        initializeObjectsDetected(&vec_customers, customersDetected);
//        CalcHistogram( &vec_customers, customersDetected );
        
        
//        if (vec_customers.size())  /**  @note if there are customers already detected */
//        {
//            CompareHistogram( vec_customers );
//        }
        
        for (int i = 0; i < track_customer.size(); i++)  /**  @note puts labels on Customer */
        {
            char identifier[100];
            sprintf(identifier, "      C%d", track_customer[i].id);
            if (track_customer[i].track)
            {
                putText(customer_line, identifier, track_customer[i].position.back(), 3, .8, WHITE, 1.5, 40);
            }
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
        
    }   /* end main loop */
    
    return 0;
}


/**
 Instantiates newly detected objects
 @function customerList_add
 @param ttcustomer an instance of Customer
 @see overloaded customerList_add(deque<Customer> customers)
 @return void
 */
void customerList_add( Customer ttcustomer)
{
    ttcustomer.id = mu++;
    track_customer.push_back(ttcustomer);
}
/**
 Instantiates newly detected objects
 @function customerList_add
 @param ttcustomer a queue of new Customer instances
 @see overloaded of customerList_add
 @return size of customer list
 */
unsigned int customerList_add(deque<Customer> ttcustomers)
{
    for (int ttc_index = 0; ttc_index < ttcustomers.size(); ttc_index++)
        customerList_add(ttcustomers[ttc_index]);
    return (int)track_customer.size();
}

/**
 Will push new customer into its respective already IDed customer. If new, then create new customer
 @function linkCustomers will push new customer into its respective already IDed customer. If new, then create new customer
 @param current_detected the objects detected in current frame
 @param anchor_customer deque List of customers already in list
 @see customerList_add(Customer custom)
 @return void
 */
void linkCustomers(deque<Customer>* current_detected, deque<Customer>* anchor_customer)
{
    /** @brief first instance of a customer
     @see if there are not customers to compare to and there is at least a customer detected */
    if (!anchor_customer->size() && current_detected->size())
        customerList_add(*current_detected);
    
    
    /**  @brief when same number of customers detected */
    double distance_obj_to_obj;
    vector<int> linker_customer_index(current_detected->size());
    /**  @start finding each object detected to all the customers in the arraylist of Customers */
    for (int cust = 0; cust < anchor_customer->size(); cust++)
    {
        /**  @note push back new data into customer which is closest i.e. customer pos to newlydetectedobject pos in frame coordinate */
        distance_obj_to_obj = 1000;
        for (int nfound = 0; nfound < current_detected->size(); nfound++)
        {
            if (norm(anchor_customer->at(cust).position.back() - current_detected->at(nfound).position.back()) < distance_obj_to_obj)
            {
                /** save index with smallest distance from prev and current list of customers */
                distance_obj_to_obj = norm(anchor_customer->at(cust).position.back() - current_detected->at(nfound).position.back());
                linker_customer_index[cust] = nfound;   ///gets the index of new_customer corresponding to index to customer list
            }
        } /** end inner for */
    } /** end outer for */
    
    
    /**  @var CSIZE: number of customer positions to push into arrayList of Customers */
    const unsigned int CSIZE = (int)min(anchor_customer->size(), current_detected->size());
    /**  @brief push back updated position of customers into array of position in customer */
    for (int linker_index = 0; linker_index < CSIZE; linker_index++)
    {
        anchor_customer->at(linker_index).position.push_back(current_detected->at(linker_customer_index[linker_index]).position.back());
        
        /**  @brief track set to true when passed predetermined line threshold */
        if (!anchor_customer->at(linker_index).track)
            if ((int)(anchor_customer->at(linker_index).position[0].x/100) == 11 &&
                (int)(anchor_customer->at(linker_index).position.back().x/100) < 10)
                anchor_customer->at(linker_index).track = true;
    }
    
    if (verbose)
        printf("\nsize of \nanchor_customers: %d \ncurrent detected: %d\n", (int)anchor_customer->size(), (int)current_detected->size());
    
    /**  @brief find index of customers that are completely new */
    bool tmp[current_detected->size()];
    for (int i = 0; i < current_detected->size(); i++)
        tmp[linker_customer_index[i]] = true;
    
    /**  @brief instantiate new customers */
    for (int i = 0; i < current_detected->size(); i++)
        if (tmp[i] == false)
            customerList_add(current_detected[i]);
    
    /**  @end linkCustomers */
}



/***  @function encapsulateObjects
         @return array of croppedObjects i.e customers and other detected objects */
deque<Customer> encapsulateObjects( Mat *instanceROI, Mat *baseROI, int METHOD, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE )
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
        {
            circle( *instanceROI, center[i], (int)radius[i]/2, paint_blue, 1, 8, 0 );
            circle( *instanceROI, center[i], 2, paint_green, 2, 8, 0);
//            circle(*instanceROI, center[i], 8, paint_red, 2, 4, 0);
//            ;
        }
    }
    

    /**   @brief merge overlapping boxes,  @return number of boxes
     @note might be better to merge contained boxes only i.e
     @note A is a subset of B if every element of A is contained in B */
    int overlapContours_size = mergeOverlappingBoxes(&boundRect, *instanceROI, &boundRectOut, METHOD);
    

    deque<Customer> croppedObject;
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
            sprintf(coordi, "[%d,%d]: ", (int)(c[i].x/100), (int)(c[i].y/100));
            putText(*instanceROI, coordi, c[i], 6, .7, WHITE);

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
    
    
    if ( METHOD == OBJECT_CUSTOMER ) {
//        for (unsigned i=0; i<prevPoints.size(); i++)
//            std::cout << "before swap" << prevPoints[i];
//        std::cout << '\n';
            prevPoints.clear();
            prevPoints.swap(c);
            c.clear();
    }
    
    return croppedObject;
}


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
int mergeOverlappingBoxes(vector<Rect> *inputBoxes, Mat &image, vector<Rect> *outputBoxes, int MOCI/*(method object customer/item)*/)
{
    Mat mask = Mat::zeros(image.size(), CV_8UC1); // Mask of original image
    Size scaleFactor(-10,-10); // To expand rectangles, i.e. increase sensitivity to nearby rectangles --can be anything
    for (int i = 0; i < inputBoxes->size(); i++)
    {
        //// double euclianPointDistance = norm(inputBoxes->at(i).tl() - inputBoxes->at(i).br());
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
        /**  Draw filled bounding boxes on mask */
        rectangle(mask, box, Scalar(255), CV_FILLED);
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

