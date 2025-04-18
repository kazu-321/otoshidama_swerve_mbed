#include "mbed.h"
#include "robomaster.hpp"

robomaster robomas(PA_11,PA_12);
Ticker pid_ticker;
void calculate_pid();
DigitalIn button(BUTTON1);
DigitalIn interrupter[4] = {
    PB_9,
    PA_6,
    PA_7,
    PA_10
};
float initial_position[4] = {
    4.843872,
    0,
    0,
    0
};
void reset();
bool is_resetting = false;
int reset_rpm = 5000;
const int delta_t = 10;

int main() {
    for(int i = 0; i < 4 ; i++) {
        robomas.set_control_type(i  , robomaster::position);
        robomas.set_control_type(i+4, robomaster::speed   );
        robomas.set_max_current<robomaster::c610>(i  , 10000);
        robomas.set_max_current<robomaster::c610>(i+4, 10000);
        robomas.set_gain<robomaster::speed>   (i  ,{2.0, 1.8, 0.1});
        robomas.set_gain<robomaster::position>(i  ,{1000, 0, 8000});
        robomas.set_gain<robomaster::speed>   (i+4,{2.0, 1.8, 0.1});
    }
    pid_ticker.attach(&calculate_pid,chrono::milliseconds(delta_t));

    reset();

    while(true) {
        robomas.send_current();
    }
}

void calculate_pid() {
    if(is_resetting) {
        for(int id = 0; id < 8; id++) {
            robomas.calculate_pid(id, delta_t);
        }
    } else {
        for(int id = 0; id < 4; id++){
            robomas.calculate_rpm_pid(id, delta_t);
            robomas.calculate_pid(id, delta_t);
            robomas.set_target<robomaster::speed>(id+4,
                robomas.get_robomaster_data(id).target_rpm +
                robomas.get_robomaster_data(id+4).target_rpm
            );
            robomas.calculate_pid(id+4,delta_t);
        }
    }
}

int get_sum(int ptr[], int n) {
    int sum = 0;
    for(int i = 0; i < n; i++) {
        sum += ptr[i];
    }
    return sum;
}

void reset() {
    is_resetting = true;
    for(int id = 0; id < 4; id++) {
        robomas.set_control_type(id, robomaster::speed);
    }
    int flag[4] = {0};
    bool prev_interrupter[4];
    for(int i = 0; i < 4; i++) {
        prev_interrupter[i] = interrupter[i];
    }
    while(get_sum(flag,4) != 4) {
        for(int i = 0; i < 4; i++){
            if(prev_interrupter[i] == true) {
                robomas.set_target<robomaster::speed>(i,   reset_rpm);
                robomas.set_target<robomaster::speed>(i+4, reset_rpm);
                prev_interrupter[i] = interrupter[i];
            } else {
                robomas.set_target<robomaster::speed>(i,   -reset_rpm);
                robomas.set_target<robomaster::speed>(i+4, -reset_rpm);
                if(interrupter[i] == true) {
                    flag[i] = 1;
                    robomas.set_neutral(i,
                        robomas.get_position(i)+initial_position[i]);
                }
            }
        }
        robomas.send_current();
    }
    is_resetting = false;
}