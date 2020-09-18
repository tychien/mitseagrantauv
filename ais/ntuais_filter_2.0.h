/****************************************/ 
/*  NAME: tychien                       */
/*  FILE: ntuais_filter_2.0.h           */ 
/*  DATE: September 18th, 2020          */ 
/****************************************/
#ifndef ntuais_filter_HEADER
#define ntuais_filter_HEADER 

#include <string>

using namespace std;

class NTUAIS_filter 
{
    public:
        NTUAIS_filter();
        ~NTUAIS_filter();
        
        bool    GetSearchDate(string input);
        bool    GetSearchRange(string input);
        string  ReturnDate();
        string  ReturnRange();
        double  toRad(double degree);
        double  calculateDistance(double lat1, double long1, double lat2, double long2);



    protected:
        
        string  m_search_date;
        string  m_search_range;
        double  station_Latitude;
        double  station_Longitude; 


    private:


};

#endif
