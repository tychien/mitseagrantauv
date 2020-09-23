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
    lat_now = 0;
    lon_now = 0;
    lat_previous = 0;
    lon_previous = 0;
    time_now = "";
    time_previous = "";
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
        string recordtime = GetString(line,25);
        string date = recordtime.substr(0,10);
        cout << "date=" << date << endl;
        if(date==ReturnDate())
            ship_list.push_back(line);
        //if((pos =line.find(ReturnDate())) != string::npos)
        //    ship_list.push_back(line); 
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
    if(ship.recordtime =="-1")
       ship.recordtime = this->GetString(s,26); 
    return ship;
};

//------------------------------------------------------------
// Buildup mmsi_list

void NTUAIS_filter::BuildupMMSIList(vector<struct Ship> vector1, vector<string> vector2)
{
    for(vector<struct Ship>::const_iterator i= vector1.begin(); i!=vector1.end(); i++){
        Ship ship       = *i;
        bool same_ship  = false;
        for(vector<string>::const_iterator k = vector2.begin(); k!= vector2.end(); k++){
            if(ship.mmsi == *k)
                same_ship = true;
        }
        if(!same_ship)
            vector2.push_back(ship.mmsi);

        cout << ship.recordtime << endl;
    }
}


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
    return fabs(dist);

}


//---------------------------------------------------------------
// TimeCalculate
double NTUAIS_filter::TimeCalculate(string t0, string t1)
{
    double time;
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

    time = double(delta);
    
    return time;
}

//-------------------------------------------------------------------
// Calculate Ave Speed

double NTUAIS_filter::AvgSpeedCalculate(vector<struct Ship> ship)
{
    double avgspeed;
    bool initial = true;
    bool timeinitial = true;
    double distcal = 0;
    double timecal = 0;
    for(vector<struct Ship>::const_iterator i=ship.begin(); i!=ship.end(); i++){
        lon_now = stod(i->lon);
        lat_now = stod(i->lat);
        cout << "lon_now:" << lon_now;
        cout << " lat_now:" << lat_now;
        time_now= i->recordtime;
        cout << " time_now:" << time_now << endl;
        if(initial == true){
            lon_previous = lon_now;
            lat_previous = lat_now;
            initial = false;
        }
       
        if(timeinitial == true){
            time_previous = time_now; 
            timeinitial = false;
        } 
        distcal += fabs( CalculateDistance(lat_now, lon_now, lat_previous, lon_previous) );
        cout << "distcal+=" << distcal << endl;
        timecal += fabs( TimeCalculate(time_previous,time_now)); 
        
        lon_previous = lon_now;
        lat_previous = lat_now;
        time_previous= time_now;
    }
     
    avgspeed = distcal/timecal;
    cout << distcal <<" "<<timecal << " "<< avgspeed << endl;
    
    return avgspeed;
}
