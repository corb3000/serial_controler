
#include <sstream>
#include <cstdlib>
#include <vector>
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "hover_comms.h"
#include "config.h"

using namespace LibSerial;


void HoverComms::setup(const std::string &serial_device)
{  
    serial_conn.Open(serial_device);
    serial_conn.SetBaudRate(BaudRate::BAUD_115200);
    // serial_conn.SetCharacterSize( CHAR_SIZE_8 ); 
    // serial_conn.SetParity( PARITY_NONE );
    
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}


SerialFeedback HoverComms::readValues()
{
    uint16_t start = 0;
    char bite0= 0;
    char bite1= 0;
    DataBuffer read_buffer;

    do{
        serial_conn.ReadByte(bite0, 5);
        if  (bite0 == (START_FRAME & 0xFF)) {
          serial_conn.ReadByte(bite1, 1);  
        }
        start = bite1 | bite0;
        }
    while (start != START_FRAME );

        serial_conn.Read(read_buffer, 20, 5);

        read_msg.start =  start;
        read_msg.cmd1=  (read_buffer[1] << 8) | read_buffer[0]; 
        read_msg.cmd2=  (read_buffer[3] << 8) | read_buffer[2]; 
        read_msg.speedR_meas=  (read_buffer[5] << 8) | read_buffer[4]; 
        read_msg.speedL_meas=  (read_buffer[7] << 8) | read_buffer[6]; 
        read_msg.wheelR_cnt=  (read_buffer[9] << 8) | read_buffer[8]; 
        read_msg.wheelL_cnt=  (read_buffer[11] << 8) | read_buffer[10];  
        read_msg.batVoltage=  (read_buffer[13] << 8) | read_buffer[12]; 
        read_msg.boardTemp=  (read_buffer[15] << 8) | read_buffer[14]; 
        read_msg.cmdLed=  (read_buffer[17] << 8) | read_buffer[16]; 
        read_msg.checksum=  (read_buffer[19] << 8) | read_buffer[18]; 


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
        encoder_update(read_msg.wheelR_cnt, read_msg.wheelL_cnt);
        return read_msg; //susess

    
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
   
    DataBuffer bytes  = 
        {
        u_int8_t((write_msg.start >> 0) & 0xFF),  
        u_int8_t((write_msg.start >> 8) & 0xFF),
        u_int8_t((write_msg.steer >> 0) & 0xFF),  
        u_int8_t((write_msg.steer >> 8) & 0xFF),
        u_int8_t((write_msg.speed >> 0) & 0xFF),  
        u_int8_t((write_msg.speed >> 8) & 0xFF),
        u_int8_t((write_msg.checksum >> 0) & 0xFF),  
        u_int8_t((write_msg.checksum >> 8) & 0xFF),
        };
serial_conn.Write(bytes);
}

void HoverComms::encoder_update (int16_t right, int16_t left) {
    static int16_t last_wheelcountR = 0;
    static int16_t last_wheelcountL = 0; 
    static int16_t multR = 0;
    static int16_t multL = 0;
    // Calculate wheel position in ticks, factoring in encoder wraps
    if (right < ENCODER_LOW_WRAP && last_wheelcountR > ENCODER_HIGH_WRAP)
        multR++;
    else if (right > ENCODER_HIGH_WRAP && last_wheelcountR < ENCODER_LOW_WRAP)
        multR--;

    last_wheelcountR = right;
    read_msg.wheelR_multR = multR;

    if (left < ENCODER_LOW_WRAP && last_wheelcountL > ENCODER_HIGH_WRAP)
        multL++;
    else if (left > ENCODER_HIGH_WRAP && last_wheelcountL < ENCODER_LOW_WRAP)
        multL--;

    last_wheelcountL = left;
    read_msg.wheelL_multL = multL;
}

