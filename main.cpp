#include "main.hpp"


int main() {
    printf("start\n");
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
    can.attach(&can_receive, CAN::RxIrq);

    for(int id = 0; id < 4; id++) {
        robomas.set_target<robomaster::position>(id,0);
    }


    while(true) {
        if(!queue.empty()) {
            CANMessage msg;
            queue.pop(msg);
            printf("%d\n",msg.id);
            switch(msg.id) {
                case 0x000:
                    if(msg.len == 1) {
                        memcpy(&canmsg.data, &msg.data, sizeof(canmsg.data));
                    }
                    break;
                case 0x001:
                    if(msg.len == 8) {
                        memcpy(&canmsg.x, &msg.data[0], sizeof(float));
                        memcpy(&canmsg.y, &msg.data[4], sizeof(float));
                    }
                    break;
                case 0x002:
                    if(msg.len == 4) {
                        memcpy(&canmsg.z, &msg.data, sizeof(float));
                    }
                    break;
            }
        }

        // for(int i = 0; i < 4; i++) {
        //     float angle_pos,rpm;
        //     angle_pos = 0;
        //     rpm = 0;
        //     robomas.set_target<robomaster::position>(i  , angle_pos);
        //     robomas.set_target<robomaster::speed>   (i+4, rpm);
        //     printf("%3d ",(int)angle_pos*10);
        // }
        // printf("\n");

        // if(!button) {
        //     for(int id = 0; id < 8; id++) {
        //         robomas.set_emg(id, true);
        //     }
        //     printf("%1.5f  %1.5f  %1.5f  %1.5f\n",
        //         robomas.get_position(0),
        //         robomas.get_position(1),
        //         robomas.get_position(2),
        //         robomas.get_position(3));
        // } else {
        //     for(int id = 0; id < 8; id++) {
        //         robomas.set_emg(id, false);
        //     }
        // }
        printf("emg: %d, reset: %d, x: %2.5f, y: %2.5f, z: %2.5f\n",
            canmsg.data.emg,
            canmsg.data.reset,
            canmsg.x,
            canmsg.y,
            canmsg.z);
        robomas.send_current();
    }
}

void calculate_pid() {
    for(int i = 0; i < 4; i++){
        if(is_resetting[i]) {
            robomas.calculate_pid(i  , delta_t);
            robomas.calculate_pid(i+4, delta_t);
        } else {
            robomas.calculate_rpm_pid(i, delta_t);
            robomas.calculate_pid(i, delta_t);
            robomas.set_target<robomaster::speed>(i+4,
                robomas.get_rpm(i) +
                robomas.get_robomaster_data(i+4).target_rpm
            );
            robomas.calculate_pid(i+4,delta_t);
        }
    }
}

void reset() {
    for(int i = 0; i < 4; i++) {
        is_resetting[i] = true;
        robomas.set_control_type(i, robomaster::speed);
    }
    bool prev_interrupter[4];
    for(int i = 0; i < 4; i++) {
        prev_interrupter[i] = interrupter[i];
    }
    while( is_resetting[0] || is_resetting[1] || is_resetting[2] || is_resetting[3] ) { // all flag is true
        for(int i = 0; i < 4; i++) {
            if(is_resetting[i]){
                if(prev_interrupter[i] == true) { // 1 -> 0
                    robomas.set_target<robomaster::speed>(i,    reset_rpm);
                    robomas.set_target<robomaster::speed>(i+4,  reset_rpm);
                    prev_interrupter[i] = interrupter[i];
                } else {    // 0
                    robomas.set_target<robomaster::speed>(i,   -reset_rpm);
                    robomas.set_target<robomaster::speed>(i+4, -reset_rpm);
                    if(interrupter[i] == true) { // 0 -> 1
                        robomas.set_neutral(i,
                            robomas.get_position(i)+initial_position[i]);
                        is_resetting[i] = false;
                        robomas.set_control_type(i, robomaster::position);
                    }
                }
            } else {
                robomas.set_target<robomaster::position>(i,   0);
                robomas.set_target<robomaster::speed>   (i+4, 0);
            }
        }
        robomas.send_current();
    }
}

void can_receive() {
    CANMessage msg;
    if(can.read(msg)) {
        queue.push(msg);
    }
}