#include "robomas.h"

robomas::robomas(PinName RD, PinName TD) : canbus(RD, TD, 1000000) {
    receive_th.start(callback(this,&robomas::receive_loop));
    canbus.attach(callback(this,&robomas::receive_irq), CAN::RxIrq);
}

void robomas::receive_loop() {
    while(true) {
        while(!queue.empty()) {
            CANMessage receive_message;
            queue.pop(receive_message);
            int id = receive_message.id - 0x201;
            if(0 <= id && id < 4) {
                bus[id].counts_now  = uint16_t((receive_message.data[0] << 8 ) | 
                                                receive_message.data[1] );
                bus[id].rpm         =  int16_t((receive_message.data[2] << 8 ) | 
                                                receive_message.data[3] );
                bus[id].current     =  int16_t((receive_message.data[4] << 8 ) | 
                                                receive_message.data[5] );
                if(bus[id].counts_last == -1) {
                    bus[id].counts_last = bus[id].counts_now;
                }
                int delta_counts = bus[id].counts_last - bus[id].counts_now;
                if(delta_counts >  ppr/2) {
                    bus[id].position_counts++;
                }
                if(delta_counts < -ppr/2) {
                    bus[id].position_counts--;
                }
                bus[id].counts_last =  bus[id].counts_now;
            } 
        }
    }
}

void robomas::receive_irq() {
    CANMessage receive_message;
    if(canbus.read(receive_message)) {
        queue.push(receive_message);
    }
}

int robomas::get_counts(int id) {
    return bus[id].counts_now;
}

int robomas::get_rpm(int id) {
    return bus[id].rpm;
}

int robomas::get_current(int id) {
    return bus[id].current;
}

float robomas::get_position(int id) {
    return bus[id].position_counts + float(bus[id].counts_now - bus[id].offset) / ppr;
}

void robomas::set_control_type(int id, robomas_control_type control_type) {
    bus[id].control_type = control_type;
    bus[id].stop = true;
}

void robomas::set_max_current(int id, int current) {
    bus[id].max_current = current;
}

void robomas::reset_offset(int id) {
    bus[id].offset = bus[id].counts_now;
}

void robomas::send_data() {
    CANMessage send_message;
    send_message.id = 0x200;
    for(int id=0; id<4; id++) {
        uint16_t current = min(max( bus[id].send_current,
                                   -bus[id].max_current),
                                    bus[id].max_current);
        if(bus[id].stop) {
            current = 0;
        }
        send_message.data[id*2] = (current >> 8) & 0xFF;
        send_message.data[id*2+1] = current & 0xFF;
    }
    canbus.write(send_message);
}

void robomas::set_gain(int id, pid_gain gain) {
    bus[id].gain = gain;
    bus[id].stop = true;
}

void robomas::set_target(int id, float target) {
    bus[id].target = target;
    bus[id].stop = false;
}

void robomas::calculate_pid(int dt) {
    if(dt <= 0.0f) {
        return;
    }
    float error = 0;
    for(int id = 0; id < 4; id++) {
        if(bus[id].control_type == robomas_control_type::current) {
            error = bus[id].target - bus[id].current;
        } else if(bus[id].control_type == robomas_control_type::speed) {
            error = bus[id].target - bus[id].rpm;
        } else if(bus[id].control_type == robomas_control_type::position) {
            error = bus[id].target - bus[id].position;
        } else {
            return;
        }
        bus[id].integral += error * dt;
        float derivative = (error - bus[id].last_error) / dt;
        bus[id].last_error = error;
        bus[id].send_current =  bus[id].gain.p * error +
                                bus[id].gain.i * bus[id].integral +
                                bus[id].gain.d * derivative;
    }
}

void robomas::reset_pid(int id) {
    bus[id].last_error = 0;
    bus[id].integral = 0;
}