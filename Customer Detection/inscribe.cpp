#ifndef INSCRIBE_HPP
#define INSCRIBE_HPP

#include <stdio.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
//
//  inscribe.cpp
//  Customer Detection
//
//  Created by Rurouni on 1/6/16.
//  Copyright © 2016 Rurouni. All rights reserved.
//
//  inscribe provides printing and lageling functionalities i.e. text over a Mat file
//  also provides Scalar2 which reorders input values from Scalar for ease of use in color


namespace inscribe
{
    /**
     Makes Scalar to RGB instead of BGR
     @function Scalar2
     @param R RED
     @param G GREEN
     @param B BLUE
     @return Scalar(RGB)
     */
    Scalar Scalar2(int R, int G, int B)
    {
        return Scalar(B,G,R);
    }
    
    /**
     Sets a tag/label over a Matrix (img)
     @function putLabel
     @param img where we put our text
     @param labelText the text to print on tag
     @param startPoint point at where we want to print our tag
     @param letters the number of characters to be in the labelText
     @param ground_color the background color of the label
     @see overloaded putTabel uses default ground color
     */
    void putLabel(Mat img, char* labelText, Point startPoint, double letters, Scalar ground_color)
    {
        int tag_width = (int)(letters * 11);
        rectangle(img, startPoint, Point(startPoint.x+tag_width,startPoint.y+20), ground_color, CV_FILLED);
        putText(img, labelText, Point(startPoint.x,startPoint.y+15), 3, .5, Scalar(255,255,255),1.5,40);
    }
    /**
     Sets a tag/label over a Matrix (img)
     @see overloaded putTabel takes ground color as param
     @code
     Mat image = imread("directory/to/image/img.png");
     char* texttoprint[100];
     sprintf(texttoprint, "image area = %d", (image.height * image.width));
     Point startingPoint(10,10);
     int numberofletters = 17;
     putLabel(image, texttoprint, startingPoint, numberofletters);
     */
    void putLabel(Mat img, char* labelText, Point startPoint, int letters)
    {
        putLabel(img, labelText, startPoint, letters, Scalar2(128, 0, 0));
    }
}
#endif