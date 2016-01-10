
#include "dependencies.hpp"

/** Default smooth type to run application, change using spacebar */
Smooth_tier smoothTier = GAUSSIAN;

/** Verbose variable flags */
bool SHOW_OVERLAPPING_BOXES = true;
bool SHOW_P2POINT_CONNECTS = false;
bool BISECT_F2FRAME = false;
bool SHOW_EDGES = true;
bool OPTFLOW_ON = false;
bool SHOW_DIFF = true;
bool verbose = 0;
bool verbose2 = 0;


/** threshold lines which divide the zones */
/** use formula */

//double y = 0.5357*x + 197.38;
//const int Z1_CROSSOVER_LINE = 25.6;
//const int Z2_CROSSOVER_LINE = 51.2;
//const int Z3_CROSSOVER_LINE = 76.8;
//const int Z4_CROSSOVER_LINE = 102.4;
//
//const int Z1_CART_SIZE_RANGES[2] = {220,500};
//const int Z2_CART_SIZE_RANGES[2] = {200,500};
//const int Z3_CART_SIZE_RANGES[2] = {246,500};
//const int Z4_CART_SIZE_RANGES[2] = {290,500};
//const int Z5_CART_SIZE_RANGES[2] = {290,500};

/** Areas (Z) which the ROI is divided into */
//const int Z1_CART_SIZE_RANGES[2] = {290,500};
//const int Z1_CROSSOVER_LINE = 96;
///** Aveg = 246  +-16  (try 2 standard deviations?) */
//const int Z2_CART_SIZE_RANGES[2] = {246,500};
//const int Z2_CROSSOVER_LINE = 64;
///** Aveg = 210  +-6 (try 2 sd?)  -Max range (450-500) */
//const int Z3_CART_SIZE_RANGES[2] = {200,500};
//const int Z3_CROSSOVER_LINE = 32;
///** Aveg = 215  +-4 */
//const int Z4_CART_SIZE_RANGES[2] = {220,550};


/** Threshold constant variables */
const int INSTANT_DISPLACEMENT_TOLERANCE = 250;
const int CART_DETECTED_AT_START_OF_LINE = 100;
const int OBJ_CREATION_LINE = 100;
const int OBJ_DELETION_LINE = 15;
const int MIN_AREA = 22000;
/**  @brief set value to desired FPS rate, recommended 10-15 */
/**  @discussion The human eye and its brain interface, the human visual system, can
    process 10 to 12 separate images per second, perceiving them individually */
const int FPS_DESIRED_FREQUENCY = 10;
const int HARD_CODED_SIGMA = 20;

/** Global variables */
Mat baseframe;
Mat untouch_frame;

Mat sketchMat;
vector<Point2d> prevPoints;
Scalar stain[10];
vector<Point2d> c;
/** unique Customer identification number */
static int mu_uid = 0;

unsigned int number_of_objects_detected;
unsigned int Number_Of_Elements;
/**
 @class Customer
 @discussion Customer class with attributes to track positions, bounding rectangles and times an object is in the line
 */
class Customer
{
public:
    int id;
    bool track;
    int type; ///type of object detected
    vector<MatND> histog;
    vector<Point2d> position;
    vector<Rect> bounding;
    double time_introduced;
    double last_recorded_time;
};

/** List of Customers to track @see class Customer  */
deque<Customer> track_customer;

/**  @function MAIN */
int main() {
    /** Default values for function encapsulateObjects */
    int sigma = 3;  ///default sigma at 3
    Smooth_tier smoothType = smoothTier;
    int ksize = (sigma*5)| 1;
    
    // insert code here...
    VideoCapture cap;
    String capstone_dir = "/Users/drifter/Desktop/capstone/";
    
    cap.open(capstone_dir+"Jan-8f10FPS.mp4");
    const double FPS_CAP=cap.get(CV_CAP_PROP_FPS);
    /** more vid files:
     @e Jan-8e_Preprocess10FPS.mp4
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
    
    /**  @constructor Rect
     Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height) */
    /** @fix CHANGE REGION (AREA) OF INTEREST ROI */
    Rect ROI_CUSTOMERLINE_WIDE(0, baseframe.rows/5,baseframe.cols,baseframe.rows/2);
    Rect ROI_CUSTOMERLINE(0,baseframe.rows/3.3,baseframe.cols,baseframe.rows/3.3);
    Rect ROI_CUSTOMERLINE_NARROW(0,baseframe.rows/2.45,1280,113);  ////113 vs 131 vs 145
    Rect ROI_CONVEYOR_BELT(baseframe.cols/2.61,baseframe.rows/4.7,250,112);

    
    int thresh = 100;
    Mat line_print, belt_print, frame, customer_line;
    line_print = baseframe(ROI_CUSTOMERLINE_NARROW);
    belt_print = baseframe(ROI_CONVEYOR_BELT);
    
    line_print.copyTo(sketchMat);
    
    stain[0] = paint_yellow;
    stain[1] = paint_red;
    stain[2] = paint_lightGREEN;
    stain[3] = paint_lightBLUE;
    stain[4] = paint_lightORANGE;
    stain[5] = paint_ade004;
    stain[6] = paint_blue;
    
    /** @redundance vector<MatND> past_vHistograms; */
    
    /**  @brief main loop */
    for(;;)
    {
        number_of_objects_detected = 0;
        /** Skip frames in order to use video as it if it was different rate of FPS, */
        for(int i = 0; i < (int)FPS_CAP/FPS_DESIRED_FREQUENCY; i++)
            cap >> frame;
        
        if(!frame.data)
            continue;
        
        int CAP_CURRENT_FRAME = (int)cap.get(CV_CAP_PROP_POS_FRAMES);
        if (BISECT_F2FRAME)
        {
            cout << "running frame: " << CAP_CURRENT_FRAME << "\n";
            cout << "------------------------\n\n";
        }
        
        /**  @abstract set regions of interest (ROI) to scan for objects  */
        Mat conveyorbelt = frame(ROI_CONVEYOR_BELT);
        customer_line = frame(ROI_CUSTOMERLINE_NARROW);
        customer_line.copyTo(untouch_frame);
        Mat displays = frame(ROI_CUSTOMERLINE_WIDE);
        char buffer[20];
        sprintf(buffer, "%6d", CAP_CURRENT_FRAME);
        putLabel(displays, buffer, Point(30,20), 6, Scalar2(76,153,0));
        
        
        /** @brief ignore returning values when detecting items on conveyor */
        deque<Customer> dev_null = encapsulateObjects(&conveyorbelt, &belt_print, OBJECT_ITEM, ksize, sigma, thresh, smoothType);
        /** @brief  hold all customers detected in current frame */
        deque<Customer> new_detected =
        encapsulateObjects(&customer_line, &line_print, OBJECT_CUSTOMER, ksize, sigma, thresh, smoothType);
        
/** @fix */
//        if (!number_of_objects_detected) {
//            baseframe = frame;
//            imshow("BASEFRAME",baseframe);
//        }
        
        /** @brief  linkCustomers pushes currently detected to its respective customer in the list, create if is a new customer */
        linkCustomers(&new_detected, &track_customer);
        
        /**  @brief put uid label on Customers in track customer mode */
        for (int i = 0; i < track_customer.size(); i++)
        {
            if (track_customer[i].track)
            {
                char identifier[20];
                sprintf(identifier, " C%d %dx%d", track_customer[i].id, track_customer[i].bounding.back().width, track_customer[i].bounding.back().height);
                putLabel(customer_line, identifier, track_customer[i].bounding.back().tl(), 10, paint_royal_blue);
                //putLabel(customer_line, identifier, track_customer[i].bounding.back().tl(), 3.5, paint_royal_blue);
            } else
            {
                char identifier[20];
                sprintf(identifier, "     %dx%d", track_customer[i].bounding.back().width, track_customer[i].bounding.back().height);
                putLabel(customer_line, identifier, track_customer[i].bounding.back().tl(), 10, paint_royal_blue);
            }
        }
        
        /** Update sigma using trackbar @note change blur method using spacebar, @see smoothType */
        sigma = HARD_CODED_SIGMA;
        createTrackbar( "Sigma", "Laplacian", &sigma, 30, 0 );;

        if(verbose)
            printf("Sigma value %d\n", sigma);
        
        imshow("customer_line", displays);
        if (OPTFLOW_ON)
            imshow("sketchMat", sketchMat);
        
        int c = waitKey(1);
        
        /** @discussion adds PAUSE key to video */
        if (c == 'p')
        {
            char PAUSED[20] = "PAUSED";
            putLabel(displays, PAUSED, Point(0,0), 6.3, paint_sea);
            imshow("customer_line", displays);
            waitKey(0);
        }
        
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
 */
void customerList_add( Customer ttcustomer)
{
    ttcustomer.id = mu_uid++;
    ttcustomer.track = false;
    track_customer.push_back(ttcustomer);
}


/**
 Instantiates newly detected objects
 @function customerList_add
 @param ttcustomers a queue of new Customer instances
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
 @function encapsulateObjects
 @code encapsulateObjects(Mat* instanceROI, Mat* baseIMG, int targetObject,
    int KSIZE, int SIGMA, int THRESH, int SMOOTHTYPE);
 @endcode
 @param instanceROI pointer to area of interest where we want to find the objects
 @param baseROI pointer to the area of interest with no objects present i.e. the background
 @param METHOD which object are we looking for? e.g. OBJ_ITEM? OBJ_CUSTOMER?
 @param KSIZE aperture liner size; must be odd and greater than 1 i.e. 3,5,7 ...
 @param SIGMA Gassian kernel standard deviation in X
 @param THRESH threshold value to be used in threshold to detect edges on final result from Laplacian
 @param SMOOTHTYPE blur type to be used i.e. MEDIAN, GAUSSIAN, BLUR or BILATERAL_FILTER
 @see customerList_add(Customer custom)
 @return array of customers and other detected objects
 */
deque<Customer>
encapsulateObjects( Mat* instanceROI, Mat* baseROI, Pick_object METHOD, int KSIZE, int SIGMA, int THRESH, Smooth_tier SMOOTHTYPE )
{
    Scalar COLOR;
    Mat currentgray, basegray, differs;
    cvtColor(*instanceROI, currentgray, COLOR_BGR2GRAY);
    cvtColor(*baseROI, basegray, COLOR_BGR2GRAY);
    
    if (OBJECT_CUSTOMER == METHOD)
    { /**  Substract from base image the current one, detects/shows people */
        differs = basegray - currentgray;
        COLOR = paint_yellow;
    }
    else if (OBJECT_ITEM == METHOD)
    {
        differs = currentgray - basegray;
        COLOR = paint_lightORANGE;
    }
    differs = differs < 60;
    
    
    
    /** if there is no objects in picture, update baseframe */
    Mat NonZero_Locations;
    findNonZero(differs, NonZero_Locations);
    Number_Of_Elements = (int)NonZero_Locations.total();

    if (!Number_Of_Elements) {
        imshow("BASE", baseframe);
    }

    
    
    
    /** @bookmark */
    for (int y = 0; y < 2; y++)
    {
        for (int x = 0; x < differs.cols; x++)
        {
            differs.at<uchar>(y,x) = 255; //white
            differs.at<uchar>(differs.rows-1-y,x) = 255;
        }
    }
    
    if(SHOW_DIFF && OBJECT_CUSTOMER == METHOD)
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
    
    
    /**  @abstract Laplacian(InputArray src, OutputArray dst, int ddepth) */
    Laplacian(smoothed, laplace, CV_16S, 5);
    convertScaleAbs(laplace, result, (SIGMA+1)*0.25);
    
    
    if(SHOW_EDGES)
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
        /** approxPolyDP(InputArray curve, OutputArray approxCurve, double epsilon, bool closed) */
        approxPolyDP( Mat(contours_eo[i]), contours_poly[i], 10, true );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
        if( (radius[i] > 35) && (OBJECT_CUSTOMER == METHOD) )  /* @fix change 35 to a const variable */
        {
            circle( *instanceROI, center[i], (int)radius[i]/1.5, paint_blue, 1, 8, 0 );
            circle( *instanceROI, center[i], 2, paint_green, 2, 8, 0);
            ///number_of_objects_detected++;
            ///circle(*instanceROI, center[i], 8, paint_red, 2, 4, 0);
        }
    }
    
    
    /**  @brief merge overlapping boxes, returns number of boxes
     @warning might be better to merge contained boxes only
     @note i.e: A is a subset of B if every element of A is contained in B */
    int overlapContours_size = mergeOverlappingBoxes(&boundRect, *instanceROI, &boundRectOut, METHOD, center);
    
    deque<Customer> croppedObject;
    Rect r;
    float areaRs = 0, density_conveyor = 0;
    float ROI_rows = (instanceROI->rows);
    float ROI_cols = (instanceROI->cols);
    Rect over_frame(0,0, ROI_cols, ROI_rows);
    /**  Draw polygonal contour + bonding rects + circles */
    for( int i = 0; i< overlapContours_size/*contours_eo.size()*/; i++ )
    {
        //        matchTemplate(untouch_frame, croppedObject[i], result, CV_TM_CCOEFF_NORMED);
        Customer tmp;
        /**  @brief rectangle(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0) */
        rectangle( *instanceROI, boundRectOut[i].tl(), boundRectOut[i].br(), COLOR, 2.0, 4, 0 );
        
        
        if (OBJECT_CUSTOMER == METHOD)
        {
            /**  @brief c center of rectangle saves coordinates */
            r = Rect(boundRectOut[i]);
            c.push_back(Point2d(r.x + r.width / 2, r.y + r.height / 2));
            // c is center of rect
            char text_coordinates[100];
            /**  @brief coordinates on base 5 */
            sprintf(text_coordinates, " (%d,%d)", (int)(c[i].x/10), (int)(c[i].y/10));
            putLabel(*instanceROI, text_coordinates, c[i], 7, paint_royal_orange);
        }
        /** to find density of conveyor add all the areas of the Rects */
        if (OBJECT_ITEM == METHOD)
        {
            r = Rect(boundRectOut[i]);
            areaRs += (over_frame.height*r.width);
        }
        
        ///tmp.histog.push_back(untouch_frame(boundRectOut[i]));
        tmp.bounding.push_back(r);
        tmp.position.push_back(Point2d(r.x + r.width / 2, r.y + r.height / 2));
        croppedObject.push_back(tmp);
        
    }
    
    /** compute density over the total area of conveyor */
    if (OBJECT_ITEM == METHOD)
    {
        /** Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height) */
        float total_conveyor_area = over_frame.height * over_frame.width;
        density_conveyor = (areaRs / total_conveyor_area);
        char buff[20];
        sprintf(buff, " p ~%2.0d%c", ((100 * density_conveyor)>=100)?99:((int)(100*density_conveyor)), '%');
        putLabel(*instanceROI, buff, Point(0,0), 7,  Scalar2(0,102,204));
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
 Will push new customer into its respective already IDed customer. If new, then create new customer
 @function linkCustomers will push new customer into its respective already IDed customer. If new, then create new customer
 @param current_detected the objects detected in current frame
 @param anchor_customer deque List of customers already in list
 @see customerList_add(Customer custom)
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
            if (current_detected->at(i).position.back().x/10 > CART_DETECTED_AT_START_OF_LINE)
            {
                customerList_add(current_detected->at(i));
            }
        }
        return;
    }
    
    /**  delete customers from list when they have finished shopping i.e. they crossed threshold */
    for (int i = 0; i < SIZEAnchor; i++)
    {
        if (anchor_customer->at(i).position.back().x/10 < OBJ_DELETION_LINE)
        {
            /** write  object to file */
            anchor_customer->erase(anchor_customer->begin()+i);
            anchor_customer->shrink_to_fit();
             /** @fix maybe update size variables, i.e. SIZEAnchor and SIZECurrent */
            /**  @fix set a flag of object that I deleted, the one I threw away should be checked */
            return;
            
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
    
    /** Initialize distances on 2D array
     The 2D array serves to save the index of the Customer list vs the newly detected list indexes */
    for (int i = 0; i < SIZEAnchor; i++)
    {
        for (int k = 0; k < SIZECurrent; k++)
        { /** save all distances */
            distance_obj_to_obj[i][k] = norm(anchor_customer->at(i).position.back() - current_detected->at(k).position.back());
        }
    }
    
    /** @var AconnectsD 
        @var DconnectsA */
    /** AconnectsD takes the index linking from Customer list to the newly (D)etected object
     DconnectsA takes the link from newly (D)etected to the (A)nchor Customer's list
     e.g. in other words we are mapping from set A to set D and viceversa
     Mapping, any prescribed way of assigning to each object in one set a particular object in another (or the same) set
     In order to keep track of the elements that have linked vs the ones that might need instantiation */
    /** @fix use doubly linked list instead */
    vector<int> AconnectsD, DconnectsA;
    
    
    /**  starting value for n_min */
    double n_mins = 1000;
    /** We create boolean arrays to keep track of Columns and Rows that have been used
     We cannot map on object to two, therefore, our mapping needs to be one-to-one to preserve integrity of Customer linking
     e.g. after selecting smallest double value we cannot use that smallest value's row and column to find the next */
    vector<bool> alpha, delta;
    
    for (int i = 0; i < SIZEAnchor; i++)
    {
        AconnectsD.push_back(-1);
        alpha.push_back(true);
    }
    for (int i = 0; i < SIZECurrent; i++)
    {
        DconnectsA.push_back(-1);
        delta.push_back(true);
    }
    
    
    /** temporary holder for a or d elements in A or D set */
    int _a_ = -1, _d_ = -1;
    /**
     @discussion 2D array, if you find smallest double value is clearly 1.1 at intersection a1xd2. a1 connects to d2
                                    a1    a2    a3     a4
                                +------------------------+
                            d1 | 2.3 | 1.5 | 7.3 | 3.5  |
                                 |- - - - - - - - - - - - - -|
                            d2 | 1.1 | 3.5 | 9.1 | 3.3  |
                                 |- - - - - - - - - - - - - -|
                            d3 | 4.5 | 6.2 | 7.1 | 2.7  |
                                +------------------------+
     next smallest distance is 1.5 at intersection a2xd1, a2 connects d1
     notice we keep connecting to the min(ROW,COLS) as we have a one-to-one mapping, in this case min(3,4) as we have only 3 rows
     next smallest is 2.3 at intersection a1xd1 but notice that we cannot connect either a1 or d1 because they are already connected, therefore
     find next smallest, which is 2.7 at intersection a4xd3. a4 connects to d3
     We have connected 3 and we cannot map any more leaving us with 'a3' disconnected,
     therefore, it might need initialization of new customer
    */
    for (int next_min = 0; next_min < min(SIZECurrent, SIZEAnchor); next_min++)
    {  /** finds all possible connecting objects from customer list to newly detected objects */
        /** finds the first minimum value in 2D vector. The position
         of this value (a,d) are our connecting object positions in the lists
         a for position in Customer list to d in current detected objects */
        n_mins = 1000, _a_ = -1, _d_ = -1;
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
                        _a_ = a;
                        _d_ = d;
                    } /** end if(delta[d] == true) */
                }
            }
        }
        /** assign connections when proper i.e. within distance tolerance */
        if (_a_ != -1 && _d_ != -1 && (n_mins < INSTANT_DISPLACEMENT_TOLERANCE))
        {  /** set connection two ways */
            AconnectsD[_a_] = _d_;
            DconnectsA[_d_] = _a_;
            /** need to find next minimun not in this row or column i.e. not in a and d
             Therefore, set flags to false to skip this row and column next iteration */
            alpha[_a_] = false;
            delta[_d_] = false;
        }
    } /** end next_min controled loop */
    
    
    /** Push new position of newly detected into Customer list
     ignore when not found new connection for a given customer i.e. keeping previous last position
     @note that if the new detected displaces too much without detecting it might be difficult to relate this objects together */
    for (int a = 0; a < AconnectsD.size()/*min(SIZECurrent,SIZEAnchor)*//*connected.size()*/; a++)
    {
        Point TMP, TMPminus1;
        if (AconnectsD[a] != -1 && distance_obj_to_obj[a][AconnectsD[a]] < INSTANT_DISPLACEMENT_TOLERANCE)
        {
            /** push all data that needs to be updated in Customer list */
            anchor_customer->at(a).bounding.push_back(current_detected->at(AconnectsD[a]).bounding.back());
            anchor_customer->at(a).position.push_back(current_detected->at(AconnectsD[a]).position.back());
        }
        if (SHOW_P2POINT_CONNECTS) {
            Customer TEMP = anchor_customer->at(a);
            TMP = TEMP.position.back();
            TMPminus1 = TEMP.position[TEMP.position.size()-2];
            cout << "[" << TMPminus1.x/10 << ", " << TMPminus1.y/10 << "]\n";
            cout << "[" << TMP.x/10 << ", " << TMP.y/10 << "]\n";
            printf("connections %d to %d\n\n", a, AconnectsD[a]);
        }
    }
    
    /**  Create new customers when there is a detected object that is introduced in the current frame
     i.e. a newly detected object which wasnt connected to a respective object in Customer list */
    for (int i = 0; i < SIZECurrent/*(DconnectsA.size())*/; i++)
    {
        /** create new Customer only if started behind thresh */
        /** @note objects are created as they are detected behind threshold line so if it detects one cart but it goes away and another cart comes in then it wont create a new object for that new cart but it will fill the previously created
         Consequently, the data beyong this threshold point is unreliable */
        if ( (DconnectsA[i] == -1) && ///(DconnectsA[i] == -1) &&
            (current_detected->at(i).position[0].x/10 > CART_DETECTED_AT_START_OF_LINE))
        {
            customerList_add(current_detected->at(i));
        }
    }
    
    
    /**  @brief CSIZE: number of customer positions to push into arrayList of Customers */
    const unsigned int CSIZE = (int)min(SIZECurrent, SIZEAnchor);
    /**  @brief push back updated position of customers into array of position in customer */
    for (int linker_index = 0; linker_index < CSIZE; linker_index++)
    {
        if (distance_obj_to_obj[linker_index][AconnectsD[linker_index]] > INSTANT_DISPLACEMENT_TOLERANCE ||
            (AconnectsD[linker_index] == -1))
            continue;
        
        /**  @brief set track to TRUE when threshold is crossed */
        if (!anchor_customer->at(linker_index).track)
            if ((int)(anchor_customer->at(linker_index).position[0].x/10) > CART_DETECTED_AT_START_OF_LINE &&
                (int)(anchor_customer->at(linker_index).position.back().x/10) < OBJ_CREATION_LINE)
                anchor_customer->at(linker_index).track = true;
        
        /**  @brief stop tracking object when customer has checked out */
    }
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
int mergeOverlappingBoxes(vector<Rect> *inputBoxes, Mat &image, vector<Rect> *outputBoxes, int MOCI, vector<Point2f>center/*(method object customer/item)*/)
{
    Mat mask = Mat::zeros(image.size(), CV_8UC1); // Mask of original image
    Size scaleFactor(-10,-10); // To expand rectangles, i.e. increase sensitivity to nearby rectangles --can be anything
    for (int i = 0; i < inputBoxes->size(); i++)
    {

        Rect r = Rect(inputBoxes->at(i));
        Point2d centroid = Point2d(r.x + r.width / 2, r.y + r.height / 2);
        double x = centroid.x/10;
        double y = 0.0158* pow(x,2) - 1.13*x + 220.71;
        
//        Div_zone ZONE;
//        if(center[i].x > Z1_CROSSOVER_LINE*10)
//            ZONE = DIV1;
//        else if(center[i].x > Z2_CROSSOVER_LINE*10)
//            ZONE = DIV2;
//        else if(center[i].x > Z3_CROSSOVER_LINE*10)
//            ZONE = DIV3;
//        else
//            ZONE = DIV4;
        
        //// double euclianPointDistance = norm(inputBoxes->at(i).tl() - inputBoxes->at(i).br());
        /**  @brief filter boxes, ignore too small or big boxes when detecting customers */
        switch (MOCI) {
            case OBJECT_CUSTOMER:
                if((inputBoxes->at(i).width < y) || (inputBoxes->at(i).width > 500))
                    continue;
//                if((inputBoxes->at(i).height * inputBoxes->at(i).width) < MIN_AREA)
//                    continue;
//                if(DIV1 == ZONE)
//                {
//                    if(inputBoxes->at(i).width < Z1_CART_SIZE_RANGES[0] || inputBoxes->at(i).width > Z1_CART_SIZE_RANGES[1])
//                        continue;
//                } else if(DIV2 == ZONE)
//                {
//                    if(inputBoxes->at(i).width < Z2_CART_SIZE_RANGES[0] || inputBoxes->at(i).width > Z2_CART_SIZE_RANGES[1])
//                        continue;
//                } else if(DIV3 == ZONE)
//                {
//                    if(inputBoxes->at(i).width < Z3_CART_SIZE_RANGES[0] || inputBoxes->at(i).width > Z3_CART_SIZE_RANGES[1])
//                        continue;
//                } else
//                {
//                    if(inputBoxes->at(i).width < Z4_CART_SIZE_RANGES[0] || inputBoxes->at(i).width > Z4_CART_SIZE_RANGES[1])
//                        continue;
//                }
                break;
            case OBJECT_ITEM:
                break;
        } /**  end switch */

        Rect box = inputBoxes->at(i) + scaleFactor;
        box.height = image.rows;
        /**  Draw filled bounding boxes on mask */
        rectangle(mask, box, Scalar(255), CV_FILLED);
    }
    
    
    if (SHOW_OVERLAPPING_BOXES && OBJECT_CUSTOMER == MOCI)
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


/**
 Draws line as the object is moving
 @function CustomerOpticalFlow
 @param noObjects_TDOF number of objects to be tracked in mask. TDOF(ToDisplayOpticalFlow)
 @see where is called, in function encapsulateObjects
 @see OPTFLOW_ON switch at top
 */
void CustomerOpticalFlow(int noObjects_TDOF)
{
    vector<Point2d> a;
    vector<Point2d> b;
    int tmpb = 0;
    
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
}
