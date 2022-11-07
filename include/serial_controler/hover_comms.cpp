
#include <sstream>
#include <cstdlib>
#include <vector>
#include <stdio.h>
#include <stdint.h>
#include "hover_comms.h"


void HoverComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial_conn_.setTimeout(10,timeout_ms,0,timeout_ms,0); 
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}


SerialFeedback HoverComms::readValues()
{
    uint8_t len = serial_conn_.read(read_buffer, read_lenth);
    if (len < read_lenth){
        if (len == 0)
        {
            return read_msg; //error;
        }
        else{
            if (serial_conn_.read(read_buffer, read_lenth) < read_lenth)
            {
                // Failed second attempt
                return read_msg; //error;
                }
            }
        }
    else{
        //check and decode
        uint16_t start = (read_buffer[1] << 8) | read_buffer[0];
        if (start != START_FRAME)
        {
                return read_msg; //error!
        }

        read_msg.start =  start;
        read_msg.cmd1=  (read_buffer[3] << 8) | read_buffer[2]; 
        read_msg.cmd2=  (read_buffer[5] << 8) | read_buffer[4]; 
        read_msg.speedR_meas=  (read_buffer[7] << 8) | read_buffer[6]; 
        read_msg.speedL_meas=  (read_buffer[9] << 8) | read_buffer[8]; 
        read_msg.wheelR_cnt=  (read_buffer[11] << 8) | read_buffer[10]; 
        read_msg.wheelL_cnt=  (read_buffer[13] << 8) | read_buffer[12];  
        read_msg.batVoltage=  (read_buffer[15] << 8) | read_buffer[14]; 
        read_msg.boardTemp=  (read_buffer[17] << 8) | read_buffer[16]; 
        read_msg.cmdLed=  (read_buffer[19] << 8) | read_buffer[18]; 
        read_msg.checksum=  (read_buffer[21] << 8) | read_buffer[20]; 

        if (read_msg.checksum != (uint16_t)(
        read_msg.start ^
        read_msg.cmd1 ^
        read_msg.cmd2 ^
        read_msg.speedR_meas ^
        read_msg.speedL_meas ^
        read_msg.wheelR_cnt ^
        read_msg.wheelL_cnt ^
        read_msg.batVoltage ^
        read_msg.boardTemp ^
        read_msg.cmdLed))
        {
            return read_msg; //error!
        }

        return read_msg; //susess

    }
}
 


void HoverComms::setMotorValues(double joints [2])
{
    
        // Convert outputs in RAD/S to RPM
    double set_speed[2] = {
        joints[0] / 0.10472,
        joints[1] / 0.10472
    };

  
    write_msg.start = START_FRAME;

        // Calculate steering from difference of left and right
    write_msg.speed = (int16_t)(set_speed[0] + set_speed[1])/2.0;
    write_msg.steer = (int16_t)(set_speed[0] - write_msg.speed)*2.0;
    write_msg.checksum = (uint16_t)(
        write_msg.start ^ 
        write_msg.steer ^ 
        write_msg.speed);

    // 16 bit system  
   
    uint8_t bytes [sizeof(write_msg) * 2] = 
        {
        (write_msg.start >> 8) & 0xFF,  
        (write_msg.start >> 0) & 0xFF,
        (write_msg.steer >> 8) & 0xFF,  
        (write_msg.steer >> 0) & 0xFF,
        (write_msg.speed >> 8) & 0xFF,  
        (write_msg.speed >> 0) & 0xFF,
        (write_msg.checksum >> 8) & 0xFF,  
        (write_msg.checksum >> 0) & 0xFF,
        };
serial_conn_.write(bytes, sizeof(write_msg) );
}


