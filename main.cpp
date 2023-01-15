#include "Wirebot.h"


// basic file operations
#include <iostream>
#include <fstream>
using namespace std;

BLA::Matrix<3> circleTraj(BLA::Matrix<1> t){
    return {sin(3*t(0)), 2*cos(3*t(0)), 3*sin(t(0)) + 5};
}

int main(){
    Wirebot Bot({-5.774,0,10}, {2.887,5,10}, {-2.887,-5,10});

    BLA::Matrix<3> c =  Bot.getTraverser({10,10,10});

    BLA::Matrix<3> q = Bot.getKinematics()->IK(c);
    float t = 0;

    ofstream myfile;

    myfile.open("log.csv");

    while(t < 10){
        BLA::Matrix<3> setpoint = circleTraj({t});

        BLA::Matrix<3> disp = Bot.getKinematics()->InverseJacobian(c) * (setpoint - c);

        q = q + disp * 0.3;
        
        c =  Bot.getTraverser(q); 

        t = t + 0.1;

        myfile<< c(0)<< ", "<<c(1)<<", "<<c(2) <<", "<< t<<'\n';

    }
    myfile.close();



}