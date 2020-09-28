/*************************************************/
/* Author: Tim TingYuan Chien                    */
/* Version 2                                     */
/* In Version 2, the speed has been replaced     */
/*      by average speed in the searching range. */ 

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>
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
        /*******************//*DATE*//******從時間篩資料************/
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
        
        /******************//*RANGE*//******從範圍篩資料**存到ship_array*********/
        if(distant< stod(ntuais_filter.ReturnRange())) 
            ntuais_filter.ship_array.push_back(ships); 
    }
/*******從ship_array找相同mmsi的msg存成string到mmsi_list中***********************/ 
    
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
    }


/****************************從mmsi_list找相同mmsi的船存到ship_sameMMSI*******************/
    vector<struct Ship> ship_sameMMSI;//這裏要建object 
    vector<vector<struct Ship> > ship_sameMMSIs; //分群的各MMSI們for the big circle under RANGE
    for(vector<string>::const_iterator i= ntuais_filter.mmsi_list.begin(); i!=ntuais_filter.mmsi_list.end(); i++){
        string shipi = *i;
        cout << "-----------------------------------------------------------"<< endl;
        cout << "shipi:" << shipi << endl;
        for(vector<struct Ship>::const_iterator k = ntuais_filter.ship_array.begin(); 
                k!=ntuais_filter.ship_array.end(); k++){
            if(shipi== k->mmsi){
                ship_sameMMSI.push_back(*k); 
            //    cout << "push_back_sameMMSI"<<" " << shipi << endl;
            }
        }
        /*******************將ship_sameMMSI裡重疊的時間內的資料刪掉*****************/ 
        //ntuais_filter.CleanUpOverlapTime(ship_sameMMSI);
        cout << "here "<<ship_sameMMSI.size() << endl;
        bool has_previous_time = false;
        int previous_time = 0;
        for(vector<struct Ship>::const_iterator j = ship_sameMMSI.begin(); j!=ship_sameMMSI.end();){
            cout << &j << endl;
            cout << j->recordtime << endl;
            string now_recoredtime = j->recordtime;
            int t1_secs = ntuais_filter.ConvertTimeToSeconds(now_recoredtime);
            cout <<"previous_time:" << previous_time << " ,t1_secs:"<< t1_secs << endl;
            while(has_previous_time == false){
                previous_time = t1_secs;
                has_previous_time =true;
            }

            if((has_previous_time == true )&& (previous_time > t1_secs)){
                j=ship_sameMMSI.erase(j);
                cout << &j << endl;
                cout << "erase("<<&j<<")"<< endl;
            }

            else
                j++; 
        }
        ntuais_filter.ShowSTL(ship_sameMMSI); 
        cout << ship_sameMMSI.size() << endl; 
       
        /***************************************************************************/

        //計算平均速度
        double avgspeed = ntuais_filter.AvgSpeedCalculate(ship_sameMMSI);
        //檢查算出來的平均速度是否為NaN
        if(isnan(avgspeed)){
            cout <<"Single AIS Msg"<< endl;
            vector<struct Ship>::const_iterator a=ship_sameMMSI.begin();
            Ship ship = *a;
            avgspeed = stod(a->sog);
            cout << avgspeed << endl;
        }
        cout << "AvgSpeed=" << avgspeed <<" knots "<< endl;
        cout << "-----------------------------------------------------------"<< endl;
        ship_sameMMSIs.push_back(ship_sameMMSI);
        ship_sameMMSI.clear(); 
    }

/***********************************************************************************/
    for(vector<string>::const_iterator l = ntuais_filter.mmsi_list.begin(); l!=ntuais_filter.mmsi_list.end(); l++){
        cout << *l << endl;
    }
    return 0;
}