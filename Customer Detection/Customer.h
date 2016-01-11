//
//  Customer.h
//  Customer Detection
//
//  Created by Rurouni on 1/10/16.
//  Copyright © 2016 Rurouni. All rights reserved.
//

#ifndef Customer_h
#define Customer_h

/**
 @class Customer
 @discussion Customer class with attributes to track positions, bounding rectangles and times an object is in the line
 */
class Customer
{
public:
    int id;
    int idle;
    bool track;
    int type; ///type of object detected
    vector<MatND> histog;
    vector<Point2d> position;
    vector<Rect> bounding;
    vector<time_t> time_lapse;
    //    time_t time_introduced;
    //    time_t last_recorded_time;
};

#endif /* Customer_h */
