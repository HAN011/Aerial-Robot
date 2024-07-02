#include "power_calc.h"
#include "arm_math.h"
#include "user_lib.h"
#include <stdlib.h>
#include <math.h>

float k1                   = 1.8231377168e-07;
float k2                   = 1.89e-07f;
float constant             = 1.0f;
float torque_coefficient   = 2.94974577124e-06f; // (20/16384) * (0.3) * (1/13) / (9.55)
float machine_power        = 0;
float current_power        = 0;
float speed_power          = 0;
float motor_current_output = 0;

float reduction_ratio,
    total_power;
uint16_t max_power;

void PowerControlInit(uint16_t max_power_init, float reduction_ratio_init)
{
    int cnt   = 0;
    max_power = max_power_init;
    if (reduction_ratio_init != 0) {
        reduction_ratio = reduction_ratio_init;
    } else {
        reduction_ratio = (187.0f / 3591.0f);
    }
    // if (cnt == 0) {
    //     toque_coefficient *= reduction_ratio;
    //     cnt++;
    // }
}

float PowerInputCalc(float motor_speed, float motor_current)
{
    motor_speed = motor_speed / 6.0f;
    // P_input = I_cmd * C_t * w + k_1* w * w +k_2 * I_cmd * I_cmd
    float power_input = motor_current * torque_coefficient * motor_speed +
                        k1 * motor_speed * motor_speed +
                        k2 * motor_current * motor_current + constant;
    machine_power = motor_current * torque_coefficient * motor_speed;
    current_power = k2 * motor_current * motor_current + constant;
    speed_power   = k1 * motor_speed * motor_speed;
    return power_input;
}

float TotalPowerCalc(float input_power[])
{
    total_power = 0;
    for (int i = 0; i < 4; i++) {
        if (input_power[i] < 0) {
            continue;
        } else {
            total_power += input_power[i];
        }
        // total_power += input_power[i];
    }
    return total_power;
}

float give_power;
float power_scale;
float torque_output;
int8_t power_flag;
float CurrentOutputCalc(float motor_power, float motor_speed, float motor_current)
{
    motor_speed = motor_speed / 6.0f;
    if (total_power > max_power) {
        power_scale = max_power / total_power;
        give_power  = motor_power * power_scale;
        power_flag  = 1;
        if (motor_power < 0) {
            if (motor_current > 15000) {
                motor_current = 15000;
            }
            if (motor_current < -15000) {
                motor_current = -15000;
            }
            return motor_current;
        }
        float a = k2;
        float b = motor_speed * torque_coefficient;
        float c = k1 * motor_speed * motor_speed - give_power + constant;
        if (motor_current > 0) {
            float temp           = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);
            motor_current_output = temp;
        } else {
            float temp           = (-b - sqrtf(b * b - 4 * a * c)) / (2 * a);
            motor_current_output = temp;
        }
        // motor_current = torque_output / 0.3f * (13 / 1) * (16384 / 20);
        // motor_current_output = motor_current;
        if (motor_current_output > 15000) {
            motor_current_output = 15000;
        } else if (motor_current_output < -15000) {
            motor_current_output = -15000;
        }
        return motor_current_output;
    }
    if (motor_current > 15000) {
        motor_current = 15000;
    } else if (motor_current < -15000) {
        motor_current = -15000;
    }
    return motor_current;
}