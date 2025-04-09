#include "mbed.h"
#include "robomas.h"

robomas bus0(PB_8,PB_9);
robomas bus1(PB_5,PB_6);
robomas bus2(PA_8,PA_15);
Ticker robomas_pid;
void calculate_pid();
const int dt = 10; // milliseconds


int main() {
    robomas_pid.attach(&calculate_pid,chrono::milliseconds(dt));
    while(true) {
        
    }
}

void calculate_pid() {
    bus0.calculate_pid(dt);
    bus1.calculate_pid(dt);
    bus2.calculate_pid(dt);
}