
#include "this.dependencies.hpp"
#include <array>

/** Global variables */
bool verbose = 0;
bool verbose2 = 0;
bool verbose_linkingCustomers = 0;
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
    
    cap.open(capstone_dir+"2CART.mp4");
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
        
        int CAP_CURRENT_FRAME = (int)cap.get(CV_CAP_PROP_POS_FRAMES);
//        if (CAP_CURRENT_FRAME < 4300)
//            continue;
        
        if (!frame.data)
            return -1;
        
        cout << "running frame: " << CAP_CURRENT_FRAME << "\n";
        cout << "------------------------\n\n";
        

        
        /**  @brief set regions of interest (ROI) to scan for objects  */
        Mat conveyorbelt = frame(MOLD_CONVEYOR_BELT);
        customer_line = frame(MOLD_CUSTOMERLINE);
        customer_line.copyTo(untouch_frame);
        Mat displays = frame(MOLD_CUSTOMERLINE_WIDE);
        char buffer[8];
        sprintf(buffer, "FPS %d", CAP_CURRENT_FRAME);
        putText(displays, buffer, Point(0+30,0+35), 3, .5, WHITE, 1.5, 40);
        
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
            if (current_detected->at(i).position.back().x/10 > 110)
            {
                customerList_add(current_detected->at(i));
            }
        }
        return;
    }
    
    /**  delete customers from list when they have finished */
    for (int i = 0; i < SIZEAnchor; i++)
    {
        if (anchor_customer->at(i).position.back().x/10 < 25)
        {
            anchor_customer->erase(anchor_customer->begin()+i);
            anchor_customer->shrink_to_fit();
        }
    }
    
    /**  Declare 2D array in order to store all distances from
        every Customer in customerList to every newly detected object */
    vector<vector<double> > distance_obj_to_obj(SIZEAnchor, vector<double>(SIZECurrent));
    for (int a = 0; a < SIZEAnchor; a++)
    {
        for (int d = 0; d < SIZECurrent; d++)
        {
            distance_obj_to_obj[a][d] = 1000.0;
            ///printf("distance: (%d,%d) %.1f\n", a, d, distance_obj_to_obj[a][d]);
        }
    }
    
    /**  Initialize distances on 2D array */
    /** The 2D array serves to save the index of the Customer list vs the newly detected list position */
    for (int i = 0; i < SIZEAnchor; i++)
    {
        for (int k = 0; k < SIZECurrent; k++)
        { /** save all distances */
            distance_obj_to_obj[i][k] = norm(anchor_customer->at(i).position.back() - current_detected->at(k).position.back());
        }
    }
    
    /** @var AconnectsD @var DconnectsA */
    /** AconnectsD takes the link from Customer list to a newly detected object
     DconnectsA takes the link from newly detected to Customer's list */
    vector<int> AconnectsD, DconnectsA;
    for (int i = 0; i < SIZEAnchor; i++)
        AconnectsD.push_back(-1);
    for (int i = 0; i < SIZECurrent; i++)
        DconnectsA.push_back(-1);

    double n_mins = 1000; /**  starting value for min value */
    vector<bool> alpha;
    vector<bool> delta;
    for (int i = 0; i < SIZEAnchor; i++)
        alpha.push_back(true);
    for (int i = 0; i < SIZECurrent; i++)
        delta.push_back(true);

    /** finds all possible connecting objects from customer list to newly detected objects */
    for (int iter = 0; iter < min(SIZECurrent, SIZEAnchor); iter++)
    {
        /** finds the first minimum value in 2D vector. The position
         of this value (a,d) are our connecting object positions in the lists 
         a for position in Customer list to d in current detected objects */
        n_mins = 1000;
        for (int a = 0; a < SIZEAnchor; a++)
        {
            /** do not find any more connection for a row which is already connected */
            if (alpha[a] == true)
            {
                for (int d = 0; d < SIZECurrent; d++)
                {
                    if ((distance_obj_to_obj[a][d] < n_mins) &&
                        (delta[d] == true)) /** do not find more connections for column already connected */
                    {
                        n_mins = distance_obj_to_obj[a][d];
                        AconnectsD[a] = d;
                        DconnectsA[d] = a;
                        /** need to find next minimun not in this row or column i.e. not in a and d
                            Therefore, set flags to false to skip this row and column */
                        alpha[a] = false;
                        delta[d] = false;
                    }
                }
            }
        }
    }
    
    /**  @if there was an update, a new closer distance between objects, then update connections */
//    if (AconnectsD[i] == -1 && tmp != -1 && DconnectsA[tmp] == -1)
//    {
//        AconnectsD[i] = tmp;
//        DconnectsA[tmp] = i;
//    }
    
    
    /**   Push new position of newly detected into Customer list
            ignore when not found new connection for a given customer i.e. keeping previous last position
            @note that if the new detected displaces too much without detecting it might be difficult to relate this objects together */
    for (int a = 0; a < AconnectsD.size()/*min(SIZECurrent,SIZEAnchor)*//*connected.size()*/; a++)
    {
        Point TMP, TMPminus1;
        if (AconnectsD[a] != -1 && distance_obj_to_obj[a][AconnectsD[a]] < 90)
        {
            anchor_customer->at(a).position.push_back(current_detected->at(AconnectsD[a]).position.back());
        }
        Customer TEMP = anchor_customer->at(a);
        TMP = TEMP.position.back();
        TMPminus1 = TEMP.position[TEMP.position.size()-2];
        cout << "[" << TMPminus1.x/10 << ", " << TMPminus1.y/10 << "]\n";
        cout << "[" << TMP.x/10 << ", " << TMP.y/10 << "]\n";
        printf("connections %d to %d\n\n", a, AconnectsD[a]);
    }
    puts("");
    
    /**  Create new customers when there is a detected object that is introduced in the current frame 
            i.e. a newly detected object which wasnt connected to a respective object in Customer list */
    for (int i = 0; i < DconnectsA.size()/*min(SIZECurrent,SIZEAnchor)*//*connected.size()*/; i++)
    {
        /** create new Customer only if started behind thresh */
        if ((DconnectsA[i] == -1) &&
            (current_detected->at(i).position.back().x/10 > 110))
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
        if (distance_obj_to_obj[linker_index][AconnectsD[linker_index]] > 90 || AconnectsD[linker_index] == -1)
            continue;
        
        /**  @brief set track to TRUE when threshold is crossed */
        if (!anchor_customer->at(linker_index).track)
            if ((int)(anchor_customer->at(linker_index).position[0].x/10) > 105 &&
                (int)(anchor_customer->at(linker_index).position.back().x/10) < 98)
                anchor_customer->at(linker_index).track = true;
        
        /**  @brief stop tracking object when customer has checked out */
    }
    /**  @endcode */
//    cout << "PASS " << __LINE__ << " LINE\n\n";
    
}


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

