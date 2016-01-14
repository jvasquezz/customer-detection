# customer-detection

## functions

/**
 @discussion checks given ROI from frame if there is an object or item. Uses basic substraction.
 @function isObjectPresent
 @param arearoi the current frame to be evaluated
 @param baseroi the base frame region of interest to compare arearoi to
 @param passes a title to show window of the changes done to the image
 @param illumination of the background
 @see isObjectPresent overloaded
 @return true if there is an object false otherwise
 */
bool isObjectPresent(Mat* arearoi, Mat* baseroi, char* header, Background illumination, bool drawrect);

/**
 @discussion overloaded of isObjectPresent
 @function isObjectPresent
 @param arearoi the current frame to be evaluated
 @param baseroi the base frame region of interest to compare arearoi to
 @param lighting of the background
 @see isObjectPresent overloaded
 @return true if there is an object false otherwise
 */
bool isObjectPresent(Mat* arearoi, Mat* baseroi, Background lighting);


/**
 Draws line as the object is moving
 @function CustomerOpticalFlow
 @param noObjects_TDOF number of objects to be tracked in mask. TDOF(ToDisplayOpticalFlow)
 @see where is called, in function encapsulateObjects
 @see OPTFLOW_ON switch at top
 */
void CustomerOpticalFlow(int noObjects_TDOF);


/**
 @discussion counts the number of swipes the cashier performs
 @function countSwipes
 @param foundObj if there is an object in current frame
 @param disp matrix where we display count
 @see isObjectPresent gets foundObj value
 */
void countSwipes(bool foundObj, Mat* disp);

/**  @function MAIN */
int main();

/**
 Instantiates newly detected objects
 @function customerList_add
 @param ttcustomer an instance of Customer
 @see overloaded customerList_add(deque<Customer> customers)
 */
template <typename T>
inline void customerList_add( T ttcustomer );

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
encapsulateObjects( Mat* instanceROI, Mat* baseROI, Pick_object METHOD, int KSIZE, int SIGMA, int THRESH, Smooth_tier SMOOTHTYPE );

/** @function template to initialize vector to a predetermined value
    @param value the value to initialize the whole vector list 
    @param allElements the number of elements to be initialized 
    @param vectorList the list to initialize */
template <typename T>
inline void initializeVector(vector<T>* vectorList, T value, unsigned int allElements);

/**
 Will push new customer into its respective already IDed customer. If new, then create new customer
 @function linkCustomers will push new customer into its respective already IDed customer. If new, then create new customer
 @param current_detected the objects detected in current frame
 @param anchor_customer deque List of customers already in list
 @see customerList_add(Customer custom)
 */
void linkCustomers(deque<Customer>* current_detected, deque<Customer>* anchor_customer);

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
int mergeOverlappingBoxes(vector<Rect> *inputBoxes, Mat &image, vector<Rect> *outputBoxes, int MOCI, vector<Point2f>center/*(method object customer/item)*/);

/**
 Draws line as the object is moving
 @function CustomerOpticalFlow
 @param noObjects_TDOF number of objects to be tracked in mask. TDOF(ToDisplayOpticalFlow)
 @see where is called, in function encapsulateObjects
 @see OPTFLOW_ON switch at top
 */
void CustomerOpticalFlow(int noObjects_TDOF);








## end functions