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

    string csvfile  = ntuais_filter.ReturnDate().substr(0,4)+ntuais_filter.ReturnDate().substr(5,2)+".csv";
   
    cout << ntuais_filter.ReturnDate() << endl;
    cout << ntuais_filter.ReturnRange() <<endl;



    return 0;
}
