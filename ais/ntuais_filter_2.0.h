/****************************************/ 
/*  NAME: tychien                       */
/*  FILE: ntuais_filter_2.0.h           */ 
/*  DATE: September 18th, 2020          */ 
/****************************************/
#ifndef ntuais_filter_HEADER
#define ntuais_filter_HEADER 

#include <string>

using namespace std;

struct Ship{
            string mmsi;
            string sog;
            string lon;
            string lat;
            string cog;
            string ship_type;
            string ref_pA;
            string ref_pB;
            string ref_pC;
            string ref_pD;
            string recordtime;
            double ship_length;
            double ship_width;    
    };


class NTUAIS_filter 
{
    public:
        NTUAIS_filter();
        ~NTUAIS_filter();
        
        bool    GetSearchDate(string input);
        bool    GetSearchRange(string input);
        string  ReturnDate();
        string  ReturnRange();
        vector<string>  ReadFile(string csvfile);
        string  GetString(string line, int m);

        double  toRad(double degree);
        double  CalculateDistance(double lat1, double long1, double lat2, double long2);
        
        
        vector<struct Ship> ship_array;
        struct Ship BuildShip(string s);
        
        double ReturnStationLat(){return station_Latitude;};
        double ReturnStationLon(){return station_Longitude;};
    protected:
        
        string  m_search_date;
        string  m_search_range;
        double  station_Latitude;
        double  station_Longitude; 


    private:


};

#endif
