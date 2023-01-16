#include "Wirebot.h"


// basic file operations
#include <iostream>
#include <fstream>
using namespace std;

BLA::Matrix<3> circleTraj(BLA::Matrix<1> t){
    return {sin(3*t(0)), 2*cos(3*t(0)), 3*sin(t(0) * 12) + 5};
}

int main(){
    Wirebot Bot({-5.774,0,10}, {2.887,5,10}, {-2.887,-5,10});

    float t = 0;

    ofstream myfile;

    myfile.open("log.csv");

    while(t < 10){
        Bot = circleTraj({t});
        BLA::Matrix<3> p = Bot.run(0.05);
        myfile<< p(0)<< ", "<<p(1)<<", "<<p(2) <<", "<< t<<'\n';
        t+=0.05;
    }
    myfile.close();



}