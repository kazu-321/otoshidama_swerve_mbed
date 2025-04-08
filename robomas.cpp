#include "robomas.h"

robomas::robomas(PinName RD, PinName TD) : canbus(RD, TD, 1000000) {
    recieve_th.start(callback(this,&robomas::recieve_loop));
    canbus.attach(callback(this,&robomas::recieve_irq), CAN::RxIrq);
}

void robomas::recieve_loop() {
    while(true) {
        while(!queue.empty()) {
            CANMessage recieve_message;
            queue.pop(recieve_message);
            int id = recieve_message.id - 0x201;
            unsigned char data[8];
            if(0 <= id && id < 4) {
                bus[id].counts_now  = uint16_t( (data[0] << 8 ) | data[1] );
                bus[id].rpm         =  int16_t( (data[2] << 8 ) | data[3] );
                bus[id].current     =  int16_t( (data[4] << 8 ) | data[5] );
                if(bus[id].counts_last == -1) {
                    bus[id].counts_last = bus[id].counts_now;
                }
                int delta_counts = bus[id].counts_last - bus[id].counts_now;
                while(delta_counts >   ppr/2) {
                    bus[id].position_counts++;
                }
                while(delta_counts < - ppr/2) {
                    bus[id].position_counts--;
                }
                bus[id].counts_last =  bus[id].counts_now;
            } 
        }
    }
}

void robomas::recieve_irq() {
    CANMessage recieve_message;
    if(canbus.read(recieve_message)) {
        queue.push(recieve_message);
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
    return bus[id].position_counts + float( (bus[id].counts_now - bus[id].offset ) / ppr );
}

void robomas::set_control_type(int id, robomas_control_type control_type) {
    bus[id].control_type = control_type;
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

        send_message.data[id*2] = (current >> 8) & 0xFF;
        send_message.data[id*2+1] = current & 0xFF;
    }
    canbus.write(send_message);
}

void robomas::set_gain(int id, pid_gain gain) {
    bus[id].gain = gain;
}

void robomas::set_target(int id, float target) {
    bus[id].target = target;
}

void robomas::calculate_pid() {
    
}