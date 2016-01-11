#ifndef ARCHIVE_HPP
#define ARCHIVE_HPP

#include <stdio.h>
#include <iostream>
#include <fstream>
//
//  archive.cpp
//  Customer Detection
//
//  Created by Rurouni on 1/10/16.
//  Copyright © 2016 Rurouni. All rights reserved.
//


namespace archive
{
    /**
     @class Customer
     @discussion Customer class with attributes to track positions, bounding rectangles and times an object is in the line
     */

    /*---------------------------------------------------------------------------*/
    /** write bounding starting points and dimensions in XML file */
    FileStorage fsbounding("bounding_rects.xml", FileStorage::APPEND);
    write( fsbounding, filename, anchor_customer->at(i).bounding);
    fsbounding.release();  /** release file */
    
    /*---------------------------------------------------------------------------*/
    /** write object to binary file */
    myFile.write((char*)&anchor_customer->at(i), sizeof(Customer));
    myFile.seekg(0);    /** read from beginning */
    myFile.close();     /** close file */
    
    /*---------------------------------------------------------------------------*/
    /** writing all positions of tracked customer to file */
    int sizep = (int)anchor_customer->at(i).position.size();
    char customerpoint[20];
    sprintf(customerpoint, "c%d.point", anchor_customer->at(i).id);
    fstream pStoring(customerpoint, ios::trunc | ios::out | ios::binary);
    pStoring.write((char*)&sizep, sizeof(sizep)); /** writes the size of vector */
    pStoring.write((char*)&anchor_customer->at(i).position[0], sizep * sizeof(anchor_customer->at(i).position));
    pStoring.seekg(0);
    pStoring.close();
    
    /*---------------------------------------------------------------------------*/
    /** writing all bounding Rects of tracked customer to file */
    int sizeb = (int)anchor_customer->at(i).bounding.size();
    char customerbound[20];
    sprintf(customerbound, "c%d.rect", anchor_customer->at(i).id);
    fstream rStoring(customerbound, ios::trunc | ios::out | ios::binary);
    rStoring.write((char*)&sizeb, sizeof(sizeb)); /** writes the size of vector */
    rStoring.write((char*)&anchor_customer->at(i).bounding[0], sizeb * sizeof(anchor_customer->at(i).bounding));
    rStoring.seekg(0);
    rStoring.close();
}
#endif