#ifndef __ROBOMAS_H__
#define __ROBOMAS_H__

#include "mbed.h"
#include "RawCAN.h"

class robomas {
public:
    robomas(PinName RD, PinName TD);

    typedef enum {
        current,
        speed,
        position,
    } robomas_control_type;
    
    typedef struct pid_gain_s {
        float p = 1;
        float i = 0;
        float d = 0;
    } pid_gain;

    typedef struct robomas_data_s {
        // recieve
        int counts_now = 0;
        int rpm = 0;
        int current = 0;

        int offset = 0;
        int counts_last = 0;
        int position_counts = 0;
        float position = 0;
        
        // control
        robomas_control_type control_type = robomas_control_type::current;
        float target = 0;
        int send_current = 0;
        int max_current = 0;
        pid_gain gain;
        float last_error = 0;
        float integral = 0;
        bool stop = true;
    } robomas_data;

    int get_counts(int id);     // -> 0~8192 <=> 0~360deg
    int get_rpm(int id);        // -> rotate / minuite
    int get_current(int id);    // -> milliampere
    float get_position(int id); // -> 1 rotate = 1.0
    // after setting control type, we have to set new target
    void set_control_type(int id, robomas_control_type control_type);
    void set_max_current(int id, int current);  // current : milliampere
    void reset_offset(int id);  // reset offset of position
    void send_data();           // send all data to canbus
    void set_gain(int id, pid_gain gain);   // after setting gain, we have to set new target
    void set_target(int id, float target);  // pid target
    void calculate_pid(int dt); // dt : millisecond
    void reset_pid(int id);     // reset pid integral and last_error


private:
    robomas_data bus[4];
    const int ppr = 8192;
    RawCAN canbus;
    CircularBuffer<CANMessage, 32> queue;
    Thread recieve_th;
    void recieve_loop();
    void recieve_irq();
};


#endif//__ROBOMAS_H__