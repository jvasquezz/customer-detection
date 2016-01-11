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
        strcpy(customerbound, destfile);
        strcat(customerbound, ".rects");
        fstream rStoring(customerbound, ios::trunc | ios::out | ios::binary);
        rStoring.write((char*)&sizeb, sizeof(sizeb)); /** writes the size of vector */
        rStoring.write((char*)&rcustomer[0], sizeb * sizeof(rcustomer));
        rStoring.seekg(0);
        rStoring.close();
        
        /*---------------------------------------------------------------------------*/
        /** write bounding starting points and dimensions in XML file */
        FileStorage fsbounding("rects.xml", FileStorage::APPEND);
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
        strcpy(customerpoint, pointfile);
        strcat(customerpoint, ".points");
        fstream pStoring(customerpoint, ios::trunc | ios::out | ios::binary);
        pStoring.write((char*)&sizep, sizeof(sizep)); /** writes the size of vector */
        pStoring.write((char*)&pcustomer[0], sizep * sizeof(pcustomer));
        pStoring.seekg(0);
        pStoring.close();
        
        /*---------------------------------------------------------------------------*/
        /** write instance points in XML file */
        FileStorage fspoints("points.xml", FileStorage::APPEND);
        write( fspoints, pointfile, *pcustomer);
        fspoints.release();  /** release file */
    }
    
    /** @abstract writes a vector of timestamps to a .times file */
    void write2(vector<struct timeval>* tcustomer, char* ttimefile)
    {
        /*---------------------------------------------------------------------------*/
        /** writing all positions of tracked customer to file */
        int sizet = (int)tcustomer->size();
        char customertimes[20];
        strcpy(customertimes, ttimefile);
        strcat(customertimes, ".times");
        fstream tStoring(customertimes, ios::trunc | ios::out | ios::binary);
        tStoring.write((char*)&sizet, sizeof(sizet)); /** writes the size of vector */
        tStoring.write((char*)&tcustomer[0], sizet * sizeof(tcustomer));
        tStoring.seekg(0);
        tStoring.close();
        
        /*---------------------------------------------------------------------------*/
        /** write instance points in XML file */
//        FileStorage fstimes("points.xml", FileStorage::APPEND);
//        write( fstimes, ttimefile, *tcustomer);
//        fstimes.release();  /** release file */
    }
}
#endif