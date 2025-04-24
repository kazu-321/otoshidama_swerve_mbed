#include "main.hpp"


int main() {
    printf("otoshidama swerve start\n");
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

    can_callback_thread.start(&can_callback_loop);

    while(true) {
        if(!queue.empty()) {
            CANMessage msg;
            queue.pop(msg);
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

        if(canmsg.data.reset) {
            reset();
            canmsg.data.reset = false;
        }


        for(int i = 0; i < 4; i++) {
            float x = canmsg.x;
            float y = canmsg.y;
            float z = canmsg.z;

            float pos_x = wheel_positions[i][0];
            float pos_y = wheel_positions[i][1];

            float vx, vy;
            if(x==0 && y==0 && z==0) {
                vx = - pos_y;
                vy = + pos_x;
                speed[i] = 0;
            } else {
                vx = x - z * pos_y;
                vy = y + z * pos_x;
                float v = sqrt(vx * vx + vy * vy);
                float omega = v / (2.0f * M_PI * wheel_radius); // 回転数 [rps]
                speed[i] = omega * 60.0f; // RPM
            }

            float target_angle = atan2(vy, vx); // [rad]
            angle[i] = optimize_angle(target_angle, last_angle[i]);

            robomas.set_target<robomaster::position>(i, angle[i] / M_PI / 2. *36.);
            robomas.set_target<robomaster::speed>(i+4,  speed[i] * 36.);

            last_angle[i] = angle[i];
        }


        for(int id = 0; id < 8; id++) {
            robomas.set_emg(id, canmsg.data.emg);
        }


        robomas.send_current();
        ThisThread::sleep_for(10ms);
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
                        printf("reset %d\n",i);
                    }
                }
            } else {
                robomas.set_target<robomaster::position>(i,   0);
                robomas.set_target<robomaster::speed>   (i+4, 0);
            }
        }
        robomas.send_current();
    }
    printf("reset end \n");
}

void can_receive() {
    CANMessage msg;
    if(can.read(msg)) {
        queue.push(msg);
    }
}

void can_callback_loop(){
    bool do_setup = false;
    bool button_flag = false;
    while(true){
        if(!button) {
            button_flag = true;
        } else {
            if(button_flag) {
                do_setup = !do_setup;
                button_flag = false;
                for(int id = 0; id < 8; id++) {
                    robomas.set_emg(id, do_setup);
                }
            }
        }
        if(do_setup) {
            printf("%1.5f, %1.5f, %1.5f, %1.5f\n",
                robomas.get_position(0),
                robomas.get_position(1),
                robomas.get_position(2),
                robomas.get_position(3));
        }

        for(int i = 0; i < 4; i++) {
            CANMessage msg;
            msg.id = 0x101 + i;
            msg.len = 8;
            memcpy(&msg.data[0],&angle[i],sizeof(float));
            memcpy(&msg.data[4],&speed[i],sizeof(float));
            can.write(msg);
            ThisThread::sleep_for(1ms);
        }
        ThisThread::sleep_for(10ms);
    }
}

float optimize_angle(float target, float last) {
    while(target - last > +M_PI) target -= 2*M_PI;
    while(target - last < -M_PI) target += 2*M_PI;
    return target;
}