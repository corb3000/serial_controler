
// C++ program to display "Hello World"
  
// Header file for input output functions
#include <iostream>
#include <string>
#include "hover_comms.h"
using namespace std;
  
// Main() function: where the execution of program begins
int main()
{
    // prints hello world
    double s;
    while (true)
    {
    cin >> s ;
    HoverComms hover_comms;
    
    hover_comms.setup("/dev/ttyUSB0", 115200, 1000);
    double jnt[2] = {s,s};
    hover_comms.setMotorValues(jnt);    /* code */
    SerialFeedback read_msg = hover_comms.readValues();
    cout << read_msg.start;
    cout << read_msg.cmd1;
    cout << read_msg.cmd2;
    cout << read_msg.speedR_meas;
    cout << read_msg.speedL_meas;
    cout << read_msg.wheelR_cnt;
    cout << read_msg.wheelL_cnt;
    cout << read_msg.batVoltage;
    cout << read_msg.boardTemp;
    cout << read_msg.cmdLed;
    cout << read_msg.checksum;

    }
    

    
    cout << "Hello World" << s;
  
    return 0;
}