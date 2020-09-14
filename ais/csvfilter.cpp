#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>


using namespace std;

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

int main()
{

    //readfile
    fstream file;
    file.open("201903.csv");
    string date = "2019-03-04";
    string line;
    size_t pos=0;
    size_t pos_comma=0;
    size_t pos_shipname=0;
    size_t pos_longitude=0;
    size_t pos_latitude=0;
    string token;
    string shipname;
    string longitude;
    string latitude;
    double station_position_longitude = 120.273155;
    double station_position_latitude = 24.000825; 
    double range = 5;
    vector<string> shipname_array;
    int j = 1;  //數第幾行
    
    

    //如果有找到該日期，算一筆
    
    while(getline(file,line)){
        bool same_ship = false;
        pos = line.find(date);
        if(pos != string::npos){
            token = line.substr(0,pos+29);
        
            //找到船名
            for(int n=1; n<=2; n++){
                pos_comma = line.find(",");
                line.erase(0,pos_comma+1); 
            }
            pos_shipname = line.find(",");
            shipname = line.substr(0,pos_shipname);
            cout << "Ship Name: "<< shipname << endl;            

                //檢查有沒有相同船名，若無就加入新船----------------- 
            for(vector<string>::iterator it=shipname_array.begin(); it != shipname_array.end(); ++it){
                if( *it == shipname) {
                    same_ship = true;
                    cout << same_ship << endl;
                }
            }
           
            //if(!same_ship){
            //    shipname_array.push_back(shipname);
            //    same_ship = false;
            //}
                //~檢查有沒有相同船名，若無就加入新船-----------------
                
                
            //~找到船名
        
            cout << j << "/ " << token << endl; 
            j++; 
            
            for(int n=1; n<=6; n++){
                pos_comma = line.find(",");
                line.erase(0,pos_comma+1);     
            }
            pos_longitude = line.find(",");
            longitude = line.substr(0,pos_longitude);
            double longitude_d = stod(longitude);
            
            cout << "Lon: " << to_string(longitude_d) << endl;
            
            for(int n=1; n<=1; n++){
                pos_comma = line.find(",");
                line.erase(0,pos_comma+1);
            }
            pos_latitude = line.find(",");
            latitude = line.substr(0,pos_latitude);
            double latitude_d = stod(latitude);
            cout << "Lat: "<< to_string(latitude_d) << endl;
            
            double distance_in_KM = calculateDistance(latitude_d,longitude_d,
                    station_position_latitude, station_position_longitude);

            cout <<"Distance: "<< distance_in_KM <<" km. "<< "\n" << endl; 
            
            if(!same_ship && (distance_in_KM <range)){
                //shipname_array.push_back(shipname);
                shipname_array.push_back(token);
                same_ship = false;
            } 
        }
    }
    
    
    //印出所有船---------------
    int shipcount = 1; 
    for(vector<string>::iterator it = shipname_array.begin(); it !=shipname_array.end(); ++it){
        cout << shipcount << "/ " << *it <<"\n"<< endl;
        shipcount ++;
    }
    //~印出所有船--------------

    //輸出到.txt檔案
        std::ofstream output_file("output.txt");
        for(const auto &e : shipname_array) output_file << e << "\n\n";
    //~輸出到.txt檔案
    return 0;
}


