/*
 * Author: Tim TingYuan Chien 錢定遠 r07525118@ntu.edu.tw
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
// GetSearchDate

bool NTUAIS_filter::GetSearchDate(string input)
{
    m_search_date = input;
    return 0;
}

//------------------------------------------------------------
// GetSearchRange
bool NTUAIS_filter::GetSearchRange(string input)
{
    m_search_range = input;
    return 0;
}

//-------------------------------------------------------------
// ReturnDate
string NTUAIS_filter::ReturnDate()
{
    return m_search_date;
}

//-------------------------------------------------------------
// ReturnRange

string NTUAIS_filter::ReturnRange()
{
    return m_search_range;
}


//-------------------------------------------------------------
// ReadFile on Certain Date then store them into ship_list

vector<string> NTUAIS_filter::ReadFile(string csvfile)
{
    vector<string> ship_list;
    fstream file;
    file.open(csvfile);
    string line;
    while(getline(file,line)){
        bool same_ship = false;
        size_t pos;
        if((pos =line.find(ReturnDate())) != string::npos)
            //cout << line << endl;
            ship_list.push_back(line); 
            //cout << "hey" << endl;
    }
    return ship_list;
}

//-------------------------------------------------------------
// GetPosition for each line 

string NTUAIS_filter::GetString(string line, int m)
{
    for(int n=1; n<=m; n++){
        size_t pos_comma = line.find(",");
        line.erase(0,pos_comma+1); 
    } 
    string get_line = line.substr(0,line.find(",")); 
    return get_line;
}

//-------------------------------------------------------------
// BuildShip

struct Ship NTUAIS_filter::BuildShip(string s)
{
    struct Ship ship;
    ship.mmsi   = this->GetString(s,3);
    ship.sog    = this->GetString(s,6);
    ship.lon    = this->GetString(s,8);
    ship.lat    = this->GetString(s,9);
    ship.cog    = this->GetString(s,10);
    ship.ship_type  = this->GetString(s,14);
    ship.ref_pA = this->GetString(s,15);
    ship.ref_pB = this->GetString(s,16);
    ship.ref_pC = this->GetString(s,17);
    ship.ref_pD = this->GetString(s,18);
    ship.recordtime = this->GetString(s,25); 
    return ship;
};


//-------------------------------------------------------------
// toRad 

double NTUAIS_filter::toRad(double degree)
{
    return degree/180*3.1415926;
}

//-------------------------------------------------------------
// calculatedistance

double NTUAIS_filter::CalculateDistance(double lat1, double long1, double lat2, double long2)
{
    double dist;
    dist =  sin(this->toRad(lat1)) * sin(this->toRad(lat2)) +
            cos(this->toRad(lat1)) * cos(this->toRad(lat2)) * cos(this->toRad(long1 - long2));
    dist = acos(dist);
    dist = 6371 * dist;
    return dist;

}


//---------------------------------------------------------------
// TimeCalculate
int NTUAIS_filter::TimeCalculate(string t0, string t1)
{
    int time;
    //example t0 = 2019-03-05 08:17:03.953000000, t1 = 2019-03-05 09:17:02.243000000000
    string t0_HH, t0_MM, t0_SS;
    string t1_HH, t1_MM, t1_SS;
    size_t pos_t0;
    size_t pos_t1;
    if((pos_t0 = t0.find(" ")) != string::npos){
        t0_HH = t0.substr(pos_t0+1, 2);
        t0_MM = t0.substr(pos_t0+4, 2);
        t0_SS = t0.substr(pos_t0+7, 2);
    }
    if((pos_t1 = t1.find(" ")) != string::npos){
        t1_HH = t1.substr(pos_t1+1, 2);
        t1_MM = t1.substr(pos_t1+4, 2);
        t1_SS = t1.substr(pos_t1+7, 2);  
    }
    int t0_secs = stoi(t0_HH)*3600+stoi(t0_MM)*60+stoi(t0_SS);
    int t1_secs = stoi(t1_HH)*3600+stoi(t1_MM)*60+stoi(t1_SS);  
    int delta = t1_secs - t0_secs;

    time = delta;
    return time;
}
