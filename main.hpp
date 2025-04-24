#ifndef __MAIN_HPP__
#define __MAIN_HPP__

#include "mbed.h"
#include "robomaster.hpp"

robomaster robomas(PA_11,PA_12);
RawCAN can(PB_5,PB_6,1000000);
DigitalIn button(BUTTON1);
DigitalIn interrupter[4] = {
    PB_9,
    PA_6,
    PA_7,
    PA_10
};
Ticker pid_ticker;
Thread can_callback_thread;
CircularBuffer<CANMessage,32> queue;
UnbufferedSerial pc(USBTX,USBRX,115200);

void calculate_pid();
void reset();
void can_receive();
void can_callback_loop();
float optimize_angle(float target,float last);

bool is_resetting[4] = {false};
float initial_position[4] = {
    -4.808,
     4.675,
    -4.127,
     4.297
};
bool reverse_rpm[4] = {
    false,
    false,
    false,
    false
};
float angle[4] = {0,0,0,0};
float last_angle[4] = {0,0,0,0};
float speed[4] = {0,0,0,0};
int reset_rpm = 1000;

constexpr int delta_t = 20;
constexpr float M_PI = 3.141592653589793;
constexpr float max_rpm = 450 * 36.;
constexpr float wheel_radius = 0.03;
constexpr float wheel_position = 0.2;
const float wheel_positions[4][2] = {
    {+ wheel_position, + wheel_position},
    {- wheel_position, + wheel_position},
    {- wheel_position, - wheel_position},
    {+ wheel_position, - wheel_position}
};

struct canmsg_s {
    struct {
        uint8_t emg : 1;
        uint8_t reset : 1;
        uint8_t reserved : 6;
    } data;
    float x;
    float y;
    float z;
} canmsg;

union float_bytes {
    uint8_t bytes[4];
    float value;
};

#endif//__MAIN_HPP__