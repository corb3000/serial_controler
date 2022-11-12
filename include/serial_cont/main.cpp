
  
// Header file for input output functions
#include <iostream>
#include <string>
#include "hover_comms.h"
#include <unistd.h>
using namespace std;
  
// Main() function: where the execution of program begins
int main()
{
    // prints 
    double s;
    cin >> s ;    
    int16_t loop = 0;
    while (true)
    {

    HoverComms hover_comms;
    
    hover_comms.setup("/dev/ttyUSB0");
    double jnt[2] = {s,s};
    hover_comms.setMotorValues(jnt);    /* code */
    SerialFeedback read_msg = hover_comms.readValues();
    cout << read_msg.start << " Start   \t";
    cout << read_msg.cmd1 << " CMD1   \t";
    cout << read_msg.cmd2 << " CMD2   \t";
    cout << read_msg.speedR_meas << " speed R   \t";
    cout << read_msg.speedL_meas << " Speed L   \t";
    cout << read_msg.wheelR_cnt << " Cnt R   \t";
    cout << read_msg.wheelL_cnt << " Cnt L   \t";
    cout << read_msg.wheelR_multR << " Mult R   \t";
    cout << read_msg.wheelL_multL << " Mult L   \t";
    cout << read_msg.batVoltage << " Bat V   \t";
    cout << read_msg.boardTemp << " Tmp   \t";
    cout << loop << " Loop  \r";

    loop ++;

    if (read_msg.start != START_FRAME){
        cout << "\n";
   
    }

    sleep(0.02);//sleeps for 0.1 second
    }
    
 
    return 0;
}