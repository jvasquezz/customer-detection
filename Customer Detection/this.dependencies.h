/* Global variables */
Mat src, erosion_dst, dilation_dst;
Mat cframe, dst;
Mat img;
Mat templ;
Mat result;

Size size(720,720);//the dst image size,e.g.100x100

VideoCapture capture;

String VIDEOPATH = "/Users/drifter/Desktop/capstone/ver.mp4";
const String BASEFRAME_DIR = "/Users/drifter/Dropbox/Feloh/Customer Detection/baseframe.png";

Scalar WHITE(255,255,255);

//int erosion_elem = 0;
//int erosion_size = 4;
//int dilation_elem = 0;
//int dilation_size = 12;
//int const max_elem = 2;
//int const max_kernel_size = 21;
//RNG rng(12345);

int erosion_type = MORPH_RECT;
int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;
RNG rng(12345);

/* Morphology EX global variables */
int morph_elem = 0;
int morph_size = 0;
int morph_operator = 0;
int const max_operator = 4;
//int const max_elem = 2;
//int const max_kernel_size = 21;

Mat aframe, baseframe;
Mat cameraFeed;

String morph = "Morphology Transformations";

Mat e_element =
getStructuringElement( erosion_type,
                      Size( 2*erosion_size+1, 2*erosion_size+1 ),
                      Point( erosion_size, erosion_size ) );
int dilation_type = MORPH_RECT;
Mat d_element =
getStructuringElement( dilation_type,
                      Size( 2*dilation_size +1, 2*dilation_size+1),
                      Point( dilation_size, dilation_size) );

/** Function Headers (prototypes) */
void Erosion( int, void* );
void Dilation( int, void* );
void Morphology_Operations( int, void* );
void on_trackbar( int, void* );
void CannyThreshold(int, void*);
void MatchingMethod( int, void* );

vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

/* For slider */
const int alpha_slider_max = 100;
int alpha_slider;
double alpha;
double beta;
Mat src1;
Mat src2;
Mat dstslider;


/* Canny vars */
Mat srcCanny, src_gray;
Mat dstCanny, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratioCanny = 3;
int kernel_size = 3;
char window_name[20] = "Edge Map";




int match_method;
int max_Trackbar = 5;
char image_window[30] = "Source Image";
char result_window[30] = "Result window";
