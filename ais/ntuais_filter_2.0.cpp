/*
 * Author: Tim TingYuan Chien
 * Version 2
 * In Version 2, the speed has been replaced by average speed in the searching range.
 */ 

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "ntuais_filter_2.0.h"

using namespace std;

//-------------------------------------------------------------
// Constructor

NTUAIS_filter::NTUAIS_filter()
{
    m_search_date ="";
    m_search_range="";
    station_Latitude = 24.000825;
    station_Longitude= 120.263155;
}

//------------------------------------------------------------
// Destructor

NTUAIS_filter::~NTUAIS_filter()
{
}

//------------------------------------------------------------
// OnNewLine

bool NTUAIS_filter::GetSearchDate(string input)
{
    m_search_date = input;
    return 0;
}

bool NTUAIS_filter::GetSearchRange(string input)
{
    m_search_range = input;
    return 0;
}

string NTUAIS_filter::ReturnDate()
{
    return m_search_date;
}

string NTUAIS_filter::ReturnRange()
{
    return m_search_range;
}

double toRad(double degree)
{
    return degree/180*3.1415926;
}

double calculateDistance(double lat1, double long1, double lat2, double long2)
{
    double dist;
    dist =  sin(toRad(lat1)) * sin(toRad(lat2)) +
            cos(toRad(lat1)) * cos(toRad(lat2)) * cos(toRad(long1 - long2));
    dist = acos(dist);
    dist = 6371 * dist;
    return dist;


}
