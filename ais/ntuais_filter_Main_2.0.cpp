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
#include <list>
#include "ntuais_filter_2.0.h"

using namespace std;


int main(int argc, char **argv)
{
    NTUAIS_filter ntuais_filter;
    for(int i=1; i<argc; i++){
        string argi = argv[i];
        if(argi.find("--date=")==0)
            ntuais_filter.GetSearchDate(argi.substr(7));
        if(argi.find("--range=")==0)
            ntuais_filter.GetSearchRange(argi.substr(8));
    } 
    
    if(ntuais_filter.ReturnDate()  == "" || (ntuais_filter.ReturnDate().size()!=10)){
        cout << "Usage: ./ntuais_filter --date=yyyy-mm-dd --range=km" << endl;
        exit(1);
    }
    
    if(ntuais_filter.ReturnRange() == ""){
        cout << "Usage: ./ntuais_filter --date=yyyy-mm-dd --range=km" << endl;
        exit(1);
    }

    string csvfile  = ntuais_filter.ReturnDate().substr(0,4)
                    + ntuais_filter.ReturnDate().substr(5,2)
                    + ".csv";
        /*******************//*DATE*//***************************/
    vector<string> ship_list = ntuais_filter.ReadFile(csvfile);//RAW Messages of that date
/*******************************************************************************/
    //Chop the String and Build the Ships and store them into a list;
    
    for(vector<string>::const_iterator i = ship_list.begin(); i!=ship_list.end(); i++){
        struct Ship ships   = ntuais_filter.BuildShip(*i);
        double ship_length  = stod(ships.ref_pA)+stod(ships.ref_pB);
        double ship_width   = stod(ships.ref_pC)+stod(ships.ref_pD);
        
        double ship_lat     = stod(ships.lat);
        double ship_lon     = stod(ships.lon);
        
        ships.ship_length   = ship_length;
        ships.ship_width    = ship_width;
        double distant = ntuais_filter.CalculateDistance(ntuais_filter.ReturnStationLat(),ntuais_filter.ReturnStationLon(), ship_lat, ship_lon);
        
        /******************//*RANGE*//*****************/
        if(distant< stod(ntuais_filter.ReturnRange())) 
            ntuais_filter.ship_array.push_back(ships); 
    }
/******************************************************************************/ 
    int j =1;

    for(vector<struct Ship>::const_iterator i = ntuais_filter.ship_array.begin(); i!=ntuais_filter.ship_array.end(); i++){
        Ship ship = *i;
        bool same_ship = false; 
        for(vector<string>::const_iterator k = ntuais_filter.mmsi_list.begin(); k!=ntuais_filter.mmsi_list.end(); k++){
            if(ship.mmsi == *k)
               same_ship = true;
        }
        if(!same_ship)
            ntuais_filter.mmsi_list.push_back(ship.mmsi);
        
        cout << ship.recordtime<< endl;
        j++;
    }
/********************************************************************************/
    vector<struct Ship> ship_sameMMSI;//這裏要建object 
    vector<vector<struct Ship> > ship_sameMMSIs; //分群的各MMSI們for the big circle under RANGE
    for(vector<string>::const_iterator i= ntuais_filter.mmsi_list.begin(); i!=ntuais_filter.mmsi_list.end(); i++){
        string shipi = *i;
        for(vector<struct Ship>::const_iterator k = ntuais_filter.ship_array.begin(); k!=ntuais_filter.ship_array.end(); k++){
            if(shipi== k->mmsi)
                ship_sameMMSI.push_back(*k); 
        }
        double avgspeed = ntuais_filter.AvgSpeedCalculate(ship_sameMMSI);
        cout << "AvgSpeed=" << avgspeed << endl;
        //計算Avg.Speed 
        ship_sameMMSIs.push_back(ship_sameMMSI);
    
    //calculateAvg.Speed
    }




/***********************************************************************************/
    for(vector<string>::const_iterator l = ntuais_filter.mmsi_list.begin(); l!=ntuais_filter.mmsi_list.end(); l++){
        
        cout << *l << endl;
        
    }

    

    cout << j << endl;
    cout << ntuais_filter.TimeCalculate("2019-03-05 22:23:23.123456789","2019-03-05 22:24:25.123456789") << endl;
    return 0;
}
