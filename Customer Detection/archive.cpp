#ifndef ARCHIVE_HPP
#define ARCHIVE_HPP

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "Customer.h"
//
//  archive.cpp
//  Customer Detection
//
//  Created by Rurouni on 1/10/16.
//  Copyright © 2016 Rurouni. All rights reserved.
//

namespace archive
{
    /** @abstract writes a customer object to file */
    void write2(Customer* acustomer, char* destination)
    {
        /*---------------------------------------------------------------------------*/
        char customerfile[20];
        strcat(customerfile, destination);
        strcat(customerfile, ".bin");
        fstream myFile(customerfile, ios::trunc | ios::out | ios::binary);
        /** check if file was opened correctly */
        if (!myFile.is_open()) { return; }
        /** write object to binary file */
        myFile.write((char*)&acustomer, sizeof(Customer));
        myFile.seekg(0);    /** read from beginning */
        myFile.close();     /** close file */
    }
    
    /** @abstract writes a vector of Rectangles to binary file and to XML file, allows unique id for naming */
    void write2(vector<Rect>* rcustomer, char* destfile)
    {
        /*---------------------------------------------------------------------------*/
        /** writing all bounding Rects of tracked customer to file */
        int sizeb = (int)rcustomer->size();
        char customerbound[20];
        strcat(customerbound, destfile);
        strcat(customerbound, ".rects");
        fstream rStoring(destfile, ios::trunc | ios::out | ios::binary);
        rStoring.write((char*)&sizeb, sizeof(sizeb)); /** writes the size of vector */
        rStoring.write((char*)&rcustomer[0], sizeb * sizeof(rcustomer));
        rStoring.seekg(0);
        rStoring.close();
        
        /*---------------------------------------------------------------------------*/
        /** write bounding starting points and dimensions in XML file */
        FileStorage fsbounding("Rect.xml", FileStorage::APPEND);
        write( fsbounding, destfile, *rcustomer);
        fsbounding.release();  /** release file */
    }
  
    /** @abstract writes a vector of points to a .points file */
    void write2(vector<Point2d>* pcustomer, char* pointfile)
    {
        /*---------------------------------------------------------------------------*/
        /** writing all positions of tracked customer to file */
        int sizep = (int)pcustomer->size();
        char customerpoint[20];
        strcat(customerpoint, pointfile);
        strcat(customerpoint, ".points");
        fstream pStoring(customerpoint, ios::trunc | ios::out | ios::binary);
        pStoring.write((char*)&sizep, sizeof(sizep)); /** writes the size of vector */
        pStoring.write((char*)&pcustomer, sizep * sizeof(pcustomer));
        pStoring.seekg(0);
        pStoring.close();
    }
}
#endif