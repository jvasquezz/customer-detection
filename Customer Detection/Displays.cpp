#ifndef KNOBS_HPP
#define KNOBS_HPP

/** This class broadcasts different stages of the frame processing */

/** Variable flags */
int SHOW_OVERLAPPING_BOXES = 0;
int SHOW_P2POINT_CONNECTIONS = 0;
int BISECT_F2FRAME = 0;
int SHOW_DISPLAY = 1;
int SHOW_EDGES = 1;
int OPTFLOW_ON = 0;
int SHOW_DIFF = 0;
int SHOW_ISOBJPRESENT = 0;
bool verbose = 0;
bool verbose2 = 0;


namespace displays
{
    
    void imshow2(int _SHOW_, string winheader, Mat* whattoshow)
    {
        if (_SHOW_)
            imshow(winheader, *whattoshow);
        else
            destroyWindow(winheader);
    }
  
    
    void castBars()
    {
        static char casttitle[30] = "Cast options:";
        namedWindow(casttitle);
        createTrackbar("Difference", casttitle, &SHOW_DIFF, 1, 0);
        createTrackbar("Overlapping boxes", casttitle, &SHOW_OVERLAPPING_BOXES, 1, 0);
        createTrackbar("Edges", casttitle, &SHOW_EDGES, 1, 0);
        createTrackbar("Optical flow", casttitle, &OPTFLOW_ON, 1, 0);
        createTrackbar("Obj present?", casttitle, &SHOW_ISOBJPRESENT, 1, 0);
        createTrackbar("Difference", casttitle, &SHOW_DIFF, 1, 0);
    }

}

#endif