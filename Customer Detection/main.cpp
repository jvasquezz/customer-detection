
#include "this.dependencies.hpp"
#include <array>

/** Global variables */
bool verbose = 0;
bool verbose2 = 0;
bool flag = 0;
bool OPTFLOW_ON = 0;

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
    
    cap.open(capstone_dir+"Untitled.mp4");
    const double FPS_CAP=cap.get(CV_CAP_PROP_FPS);
    /** more vid files:
     @a SEGMENTA_720P_20FPS.mp4
     @c 20FPS720P1416.mp4
     @b 10FPS.mp4
     @b ver.mp4
     @e 2CART.mp4
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
        /** Skip frames in order to use video as it if it was 10FPS */
        for(int i = 0; i < (int)FPS_CAP/6; i++)
            cap >> frame;
        //        cap >> frame;
        
        if (!frame.data)
            return -1;
        
        
        /**  @brief set regions of interest (ROI) to scan for objects  */
        Mat conveyorbelt = frame(MOLD_CONVEYOR_BELT);
        customer_line = frame(MOLD_CUSTOMERLINE);
        customer_line.copyTo(untouch_frame);
        Mat displays = frame(MOLD_CUSTOMERLINE_WIDE);
        
        /**  @function @brief
         encapsulateObjects(Mat* instanceROI, Mat* baseIMG, int targetObject, int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE)*/
        encapsulateObjects(&conveyorbelt, &belt_print, OBJECT_ITEM, ksize, sigma, thresh, smoothType);
        deque<Customer> new_detected = encapsulateObjects(&customer_line, &line_print, OBJECT_CUSTOMER, ksize, sigma, thresh, smoothType);
        
        linkCustomers(&new_detected, &track_customer);
        
        
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
        
        imshow("customer_line", displays);
        if (OPTFLOW_ON)
            imshow("sketchMat", sketchMat);
        
        int c = waitKey(1);
        if( c == ' ' )
        {
            smoothType = smoothType == GAUSSIAN ? BLUR : smoothType == BLUR ? MEDIAN : smoothType == MEDIAN ? BILATERAL_FILTER : GAUSSIAN;
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
    ttcustomer.track = false;
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
    unsigned int SIZEAnchor = (int)anchor_customer->size();
    unsigned int SIZECurrent = (int)current_detected->size();
    
    /**  initialize customers when list is empty */
    if (anchor_customer->empty() && not current_detected->empty())
    {
        for (int i = 0; i < SIZECurrent; i++)
        {
            if (current_detected->at(i).position.back().x/10 > 25 )
            {
                customerList_add(current_detected->at(i));
            }
        }
        return;
    }
    
    double distance_obj_to_obj = 1000.0;
    /** @var Aconnected @var Dconnected */
    /** Aconnected takes the link from Customer list to the newly detected
     Dconnected takes the link from newly detected to Customers */
    vector<int> Aconnected, Dconnected;
    for (int i = 0; i < SIZEAnchor; i++)
        Aconnected.push_back(-1);
    for (int i = 0; i < SIZECurrent; i++)
        Dconnected.push_back(-1);
    
    /**  check every customer against newly detected */
    for (int i = 0; i < SIZEAnchor; i++)
    {
        /** @var tmp will hold the index of each of the new detected objects to connect it with customer */
        int tmp = -1;
        distance_obj_to_obj = 1000.0;
        for (int k = 0; k < SIZECurrent; k++)
        {
            /** @if distance between an object and a customer pos is smaller than placeholder than update connection */
            if (norm(anchor_customer->at(i).position.back() - current_detected->at(k).position.back()) < distance_obj_to_obj)
            {
                distance_obj_to_obj = norm(anchor_customer->at(i).position.back() - current_detected->at(k).position.back());
                tmp =  k;
            }
        }
        /**  @if there was an update, a new closer distance between objects, then update connections */
        if (Aconnected[tmp] == -1 && tmp != -1)
        {
            Aconnected[tmp] = i;
            Dconnected[i] = tmp;
        }
        
    }
    
    for (int i = 0; i < Aconnected.size()/*min(SIZECurrent,SIZEAnchor)*//*connected.size()*/; i++)
    {
        if (Aconnected[i] != -1)
        {
            anchor_customer->at(Aconnected[i]).position.push_back(current_detected->at(i).position.back());
            Customer TEMP = anchor_customer->at(i);
            Point TMP = TEMP.position.back();
            cout << TMP << "\n";
        }
        printf("connections %d to %d\n\n", i, Aconnected[i]);
    }
    puts("");
    for (int i = 0; i < Dconnected.size()/*min(SIZECurrent,SIZEAnchor)*//*connected.size()*/; i++)
    {
        if (Dconnected[i] == -1)
        {
            customerList_add(current_detected->at(i));
        }
    }
    
    
    
    /**  @code */
    /**  @var CSIZE: number of customer positions to push into arrayList of Customers */
    const unsigned int CSIZE = (int)min(SIZECurrent, SIZEAnchor);
    /**  @brief push back updated position of customers into array of position in customer */
    for (int linker_index = 0; linker_index < CSIZE; linker_index++)
    {
        if (distance_obj_to_obj > 90 || Aconnected[linker_index] == -1)
            continue;
        
        anchor_customer->at(linker_index).position.push_back(current_detected->at(Aconnected[linker_index]).position.back());
        
        /**  @brief set track to TRUE when threshold is crossed */
        if (!anchor_customer->at(linker_index).track)
            if ((int)(anchor_customer->at(linker_index).position[0].x/10) > 105 &&
                (int)(anchor_customer->at(linker_index).position.back().x/10) < 98)
                anchor_customer->at(linker_index).track = true;
        
        /**  @brief stop tracking object when customer has checked out */
        if ((int)(anchor_customer->at(linker_index).position.back().x/10) < 25)
        {
            anchor_customer->at(linker_index).track = false;
        }
    }
    /**  @endcode */
    
}

//void linkCustomers(deque<Customer>* current_detected, deque<Customer>* anchor_customer)
//{
//    /** if there aren't customers to compare to and there is at least one customer detected
//     @brief first instance of customers */
//    if (!anchor_customer->size() && current_detected->size())
//    {
//        cout << "INIT CUSTOMERS\n";
//        customerList_add(*current_detected);
//        return;
//    }
//    unsigned long CDSIZE = current_detected->size();
//    unsigned long ACSIZE = anchor_customer->size();
//
//    /**  @brief when same number of customers detected */
//    double distance_obj_to_obj = 1000.0;
//    vector<bool> linked;
//    bool b = false;
////    generate_n(back_inserter(linked), anchor_customer->size(), [&b]() { return (b = !b); });
//
//
//    vector<int> linker_customer_index;
//    for (int i = 0; i < anchor_customer->size(); i++)
//    {
//        linker_customer_index.push_back(-1);
//        linked.push_back(false);
//    }
//
//    for (int i = 0; i < anchor_customer->size(); i++)
//        cout << linked[i] << "\n";
//
////    for (int i = 0; i < linker_customer_index.size(); i++)
////    {
////        printf("linker %d to %d\n", i, linker_customer_index[i]);
////    }
//
//    /**  @start finding each object detected to all the customers in the arraylist of Customers */
//    for (int cust = 0; cust < anchor_customer->size(); cust++)
//    {
//        distance_obj_to_obj = 1000.0;
//        /**  @note push back new data into customer which is closest i.e. customer pos to newlydetectedobject pos in frame coordinate */
//        for (int nfound = 0; nfound < current_detected->size(); nfound++)
//        {
//            if (norm(anchor_customer->at(cust).position.back() - current_detected->at(nfound).position.back()) < distance_obj_to_obj)
//            {
//                /** save index with smallest distance from prev and current list of customers */
//                distance_obj_to_obj = norm(anchor_customer->at(cust).position.back() - current_detected->at(nfound).position.back());
//                if (linked[nfound] == false)
//                {
//                    linker_customer_index[cust] = nfound;   ///gets the index of new_customer corresponding to index to customer list
//                }
//            }
//        } /** end inner for */
//        linked[linker_customer_index[cust]] = true;
//    } /** end outer for */
//
//    if (distance_obj_to_obj > 10.0)
//        printf("distance obj to obj:\n%.2f\n", distance_obj_to_obj);
//
//    for (int i = 0; i < linker_customer_index.size(); i++)
//    {
//        printf("linker %d to %d\n", i, linker_customer_index[i]);
//    }
//    puts("");
//
//
//    for (int i = 0; i < anchor_customer->size(); i++)
//    {
//        printf("Customer[%d] \ncurrent and at(0) positionX:\ncurrent: %d\nat(0): %d\n", i,
//               (int)anchor_customer->at(i).position.back().x/10,
//               (int)anchor_customer->at(i).position[0].x/10);
//    }
//    puts("");
//
//
//    /**  @var CSIZE: number of customer positions to push into arrayList of Customers */
//    const unsigned int CSIZE = (int)min(anchor_customer->size(), current_detected->size());
//    /**  @brief push back updated position of customers into array of position in customer */
//    for (int linker_index = 0; linker_index < CSIZE; linker_index++)
//    {
//        if (distance_obj_to_obj > 90 || linker_customer_index[linker_index] == -1)
//            continue;
//
//        anchor_customer->at(linker_index).position.push_back(current_detected->at(linker_customer_index[linker_index]).position.back());
//
//        /**  @brief set track to TRUE when threshold is crossed */
//        if (!anchor_customer->at(linker_index).track)
//            if ((int)(anchor_customer->at(linker_index).position[0].x/10) > 105 &&
//                (int)(anchor_customer->at(linker_index).position.back().x/10) < 98)
//                anchor_customer->at(linker_index).track = true;
//
//        /**  @brief stop tracking object when customer has checked out */
//        if ((int)(anchor_customer->at(linker_index).position.back().x/10) < 25)
//        {
//            anchor_customer->at(linker_index).track = false;
//        }
//    }
//
//
//
//    if (verbose)
//        printf("\nsize of \nanchor_customers: %d \ncurrent detected: %d\n", (int)anchor_customer->size(), (int)current_detected->size());
//
//
//    /**  @brief find index of customers that are completely new */
//    vector<bool> tmp;
//    generate_n(back_inserter(tmp), current_detected->size(), [&b]() { return (b = !b); });
//
//    for (int i = 0; i < current_detected->size(); i++)
//        if (linker_customer_index[i] != -1)
//            tmp[linker_customer_index[i]] = true;
//
//    for (int i = 0; i < tmp.size(); i++)
//        cout << tmp[i] << "\n";
//
//    /**  @brief instantiate new customers */
//    for (int i = 0; i < current_detected->size(); i++)
//        if (tmp[i] == false)
//        {
//            cout << "ADD INIT CUSTOMERS\n\n";
//            customerList_add(current_detected[i]);
//        }
//
//    if (CDSIZE != ACSIZE)
//    {
//        ;
//    }
//
//    /**  @endcode linkCustomers */
//}


/**
 Will push new customer into its respective already IDed customer. If new, then create new customer
 @function encapsulateObjects
 @param *instanceROI pointer to area of interest where we want to find the objects
 @param *baseROI pointer to the area of interest with no objects present i.e. the background
 @param METHOD which object are we looking for? e.g. OBJ_ITEM? OBJ_CUSTOMER?
 @param KSIZE aperture liner size; must be odd and greater than 1 i.e. 3,5,7 ...
 @param SIGMA Gassian kernel standard deviation in X
 @param THRESH threshold value to be used in threshold to detect edges on final result from Laplacian
 @param SMOOTHTYPE blur type to be used i.e. MEDIAN, GAUSSIAN, BLUR or BILATERAL_FILTER
 @see customerList_add(Customer custom)
 @return array of customers and other detected objects
 */
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
            sprintf(coordi, "[%d,%d]: ", (int)(c[i].x/10), (int)(c[i].y/10));
            putText(*instanceROI, coordi, c[i], 6, .7, WHITE);
            
        }
        tmp.histog.push_back(untouch_frame(boundRectOut[i]));
        tmp.position.push_back(Point2d(r.x + r.width / 2, r.y + r.height / 2));
        croppedObject.push_back(tmp);
        
    }
    
    if (OPTFLOW_ON &&
        (OBJECT_CUSTOMER == METHOD))
        CustomerOpticalFlow(overlapContours_size);
    
    /** @brief if OBJ_CUSTOMER we update and clear points  */
    if (OBJECT_CUSTOMER == METHOD)
    {
        prevPoints.clear();
        prevPoints.swap(c);
        c.clear();
    }
    
    return croppedObject;
}

/**
 Draws line as the object is moving
 @function CustomerOpticalFlow
 @param noObjects_TDOF number of objects to be tracked in mask. TDOF(ToDisplayOpticalFlow)
 @see where is called, in function encapsulateObjects
 @see OPTFLOW_ON switch at top
 @return null
 */
void CustomerOpticalFlow(int noObjects_TDOF)
{
    vector<Point2d> a;
    vector<Point2d> b;
    int tmpb = 0;
    
    /** @code */
    /** Tracks a customer using a line on mask */
    if ( prevPoints.size() == c.size() )
    {
        for (int i = 0; i < noObjects_TDOF; i++)
        {
            double min = 10000;
            for (int j = 0; j < noObjects_TDOF; j++)
            {
                if ( min > (norm( prevPoints[i] - c[j]))  )
                {
                    min = norm( prevPoints[i] - c[j] );
                    tmpb = j;
                }
            }
            a.push_back(prevPoints[i]);
            b.push_back(c[tmpb]);
        }
        
        /** Draws line over mask */
        for (int i = 0; i < noObjects_TDOF; i++ )
            line(sketchMat, a[i], b[i], stain[i], 2, 4);
    }
    
    /** @endcode */
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

