#pragma once

#include <stdint.h>

typedef struct
{
    float input_power[4];
    float wheel_speed[4];
    float total_power;
    float predict_output[4];
    uint8_t count;
} Power_Data_s;

void PowerControlInit(uint16_t max_power_init, float reduction_ratio_init);

float PowerInputCalc(float motor_speed, float motor_current);

float TotalPowerCalc(float input_power[]);

float CurrentOutputCalc(float motor_power, float motor_speed, float motor_current);