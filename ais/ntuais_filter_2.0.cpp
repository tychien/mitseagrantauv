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

using namespace std;


int main(int argc, char **argv)
{
    string search_date;
    string search_range;
    for(int i=1; i<argc; i++){
        string argi = argv[i];
        if(argi.find("--date=")==0)
            search_date  = argi.substr(7);
        if(argi.find("--range=")==0)
            search_range = argi.substr(8);
    } 
    if(search_date  == "" || (search_date.size()!=10)){
        cout << "Usage: ./ntuais_filter --date=yyyy-mm-dd --range=km" << endl;
        exit(1);
    }
    if(search_range == ""){
        cout << "Usage: ./ntuais_filter --date=yyyy-mm-dd --range=km" << endl;
        exit(1);
    }

    string csvfile  = search_date.substr(0,4)+search_date.substr(5,2)+".csv";
   




    return 0;
}
