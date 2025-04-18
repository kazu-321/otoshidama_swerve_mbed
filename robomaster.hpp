#ifndef __ROBOMAS_HPP__
#define __ROBOMAS_HPP__

#include "mbed.h"
#include "math.h"

class robomaster {
public:
    typedef enum {
        current,
        speed,
        position
    } control_type;

    typedef enum {
        c610,
        c620,
        gm6020
    } esc_type;

    typedef struct gain_s {
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;
    } gain;

    typedef struct data_s {
        int counts = 0;
        int rpm = 0;
        int current = 0;

        int prev_counts = -1;
        int position_counts = 0;
        float position = 0;

        control_type control_type = control_type::current;
        esc_type esc_type = esc_type::c610;
        
        float target_rpm;
        float target_position;

        int send_current = 0;
        int max_current = 0;
        gain gain_speed;
        gain gain_position;
        bool stop = true;
        bool emg = false;

        float prev_error_speed;

        float neutral = 0;
        float last_position = -1;
        float prev_error_position;
        float error_integral_position;
        float prev_input_input_delay;
        float prev_output_input_delay;


        float prev_ampare;
        float prev_output;
        float prev_error_mec;
        float Kp_mec = 1;
        float Ki_mec = 0;
        float Kd_mec = 0;
    } robomaster_data;

    robomaster(PinName rx, PinName tx) : bus(rx,tx,hz) {
        receive_thread.start(callback(this,&robomaster::receive_loop));
        bus.attach(callback(this,&robomaster::receive_irq), CAN::RxIrq);
    }

    int get_counts(int id) {
        return data[id].counts;
    }

    int get_rpm(int id) {
        return data[id].rpm;
    }

    int get_current(int id) {
        return data[id].current;
    }

    float get_position(int id) {
        return data[id].position;
    }

    robomaster_data get_robomaster_data(int id) {
        return data[id];
    }

    void set_neutral(int id, float neutral) {
        data[id].neutral = neutral;
    }

    void set_emg(int id, bool emg) {
        data[id].emg = emg;
    }

    void send_current() {
        CANMessage message;
        message.id = 0x200;
        for(int id = 0; id < 4; id++) {
            int current = fix_current(id);
            message.data[id*2] = (current >> 8) & 0xFF;
            message.data[id*2+1] = current & 0xFF;
        }
        bus.write(message);
        message.id = 0x1FF;
        for(int id = 0; id < 4; id++) {
            int current = fix_current(id+4);
            message.data[id*2] = (current >> 8) & 0xFF;
            message.data[id*2+1] = current & 0xFF;
        }
        bus.write(message);
    }

    void set_control_type(int id, control_type control_type) {
        data[id].control_type = control_type;
        data[id].stop = true;
    }

    template<esc_type esc_type>
    void set_max_current(int id, int current);

    template<>
    void set_max_current<c610>(int id, int current) {
        if(current < -max_current_c610) current = -max_current_c610;
        if(max_current_c610 < current)  current =  max_current_c610;
        data[id].max_current = current;
    }

    template<>
    void set_max_current<c620>(int id, int current) {
        if(current < -max_current_c620) current = -max_current_c620;
        if(max_current_c620 < current)  current =  max_current_c620;
        data[id].max_current = current;
    }

    template<>
    void set_max_current<gm6020>(int id, int current) {
        if(current < -max_current_gm6020) current = -max_current_gm6020;
        if(max_current_gm6020 < current)  current =  max_current_gm6020;
        data[id].max_current = current;
    }

    template<control_type control_type>
    void set_gain(int id, gain gain);

    template<>
    void set_gain<speed>(int id, gain gain) {
        data[id].gain_speed = gain;
    }

    template<>
    void set_gain<position>(int id, gain gain) {
        data[id].gain_position = gain;
    }

    template<control_type control_type>
    void set_target(int id, float target) {
        if(data[id].control_type == current) {
            set_target<current>(id,target);
        } else if(data[id].control_type == speed) {
            set_target<speed>(id, target);
        } else if(data[id].control_type == position) {
            set_target<position>(id, target);
        }
    }

    template<>
    void set_target<current>(int id, float target) {
        data[id].send_current = static_cast<int>(target);
    }

    template<>
    void set_target<speed>(int id, float target) {
        data[id].target_rpm = target;
        data[id].stop = false;
    }

    template<>
    void set_target<position>(int id, float target) {
        data[id].target_position = target;
        data[id].stop = false;
    }

    void calculate_pid(int id, int delta_t) {
        float filtered_rpm = input_delay(id, data[id].target_rpm);
        float need_rpm = filtered_rpm + pid_control(id, filtered_rpm - data[id].rpm, (delta_t/1000.));
        float target_ampare = need_rpm * rpm_to_ampare;
        float model = calculate_plant_model(id, target_ampare);
        float input_ampare = target_ampare + compensator(id, model - data[id].rpm);
        data[id].send_current = input_ampare;
    }

    void calculate_rpm_pid(int id, int delta_t) {
        float error_position = data[id].target_position - data[id].position;
        data[id].error_integral_position += (error_position + data[id].prev_error_position) / 2.0 * (delta_t/1000.);
        float target_rpm = error_position * data[id].gain_position.Kp +
            data[id].error_integral_position * data[id].gain_position.Ki +
            (error_position - data[id].prev_error_position) * data[id].gain_position.Kd;
        data[id].prev_error_position = error_position;
        data[id].target_rpm = target_rpm;
    }


private:
    const int hz = 1000000;
    const int ppr = 8192;
    const int max_current_c610 = 10000;
    const int max_current_c620 = 10000;
    const int max_current_gm6020 = 16000;
    const float rpm_to_ampare = 0.29411764705882354;
    robomaster_data data[8];
    RawCAN bus;
    CircularBuffer<CANMessage, 32> queue;
    Thread receive_thread;
    
    void receive_loop() {
        while(true) {
            while(!queue.empty()) {
                CANMessage message;
                queue.pop(message);
                int id = message.id - 0x201;
                if(0 <= id && id < 8) {
                    
                    data[id].counts  =uint16_t((message.data[0] << 8) |
                                                message.data[1]);
                    data[id].rpm     = int16_t((message.data[2] << 8) |
                                                message.data[3]);
                    data[id].current = int16_t((message.data[4] << 8) |
                                                message.data[5]);

                    if(data[id].prev_counts == -1) {
                        data[id].prev_counts = data[id].counts;
                    }
                    int delta_counts = data[id].prev_counts - data[id].counts;
                    if(delta_counts > ppr/2) {
                        data[id].position_counts++;
                    }
                    if(delta_counts < -ppr/2) {
                        data[id].position_counts--;
                    } 
                    data[id].prev_counts = data [id].counts;
                    data[id].position = data[id].position_counts + (float)data[id].counts/ ppr - data[id].neutral;
                } 
            }
        }
    }

    void receive_irq() {
        CANMessage message;
        if(bus.read(message)) {
            queue.push(message);
        }
    }

    int fix_current(int id) {
        int current = data[id].send_current;
        if(current < -data[id].max_current) {
            current = -data[id].max_current;
        }
        if(data[id].max_current < current)  {
            current =  data[id].max_current;
        }
        if(data[id].stop || data[id].emg) {
            current = 0;
            data[id].error_integral_position = 0;
            data[id].prev_output = 0;
            data[id].prev_error_speed = 0;
            data[id].prev_input_input_delay = 0;
            data[id].prev_output_input_delay = 0;
        }
        return current;
    }

    float input_delay(int id, float input) {
        float Kdel=0.04761904761904762;        //遅延計算の重み　変えるな
        float output = Kdel*input + Kdel* data[id].prev_input_input_delay + (0.9+Kdel/10)* data[id].prev_output_input_delay;
        data[id].prev_input_input_delay = input;
        data[id].prev_output_input_delay = output;
        return output;
    }

    float pid_control(int id, float error, int delta_t) {
        float error_integral_speed = (error + data[id].prev_error_speed)/2 * (delta_t/1000.);
        float output =  error * data[id].gain_speed.Kp +
            error_integral_speed * data[id].gain_speed.Ki +
            (error - data[id].prev_error_speed) * data[id].gain_speed.Kd;
        data[id].prev_error_speed = error;
        return output;
    }

    float calculate_plant_model(int id, float target_ampare) {
        float plant_model = 0.085 * target_ampare + 
            0.085 * data[id].prev_ampare + 
            0.95 * data[id].prev_output;
        data[id].prev_ampare = target_ampare;
        data[id].prev_output = plant_model;
        return plant_model;
    }

    float compensator(int id, float error) {
        float output = data[id].Kp_mec * error +
            data[id].Kd_mec*(error - data[id].prev_error_mec);
        data[id].prev_error_mec = error;
        return output;
    }

};

#endif//__ROBOMAS_HPP__