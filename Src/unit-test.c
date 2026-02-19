
#define STM32H753xx

#include "HX711.h"
#include <stm32h7xx_hal.h>
#include <stdio.h>

bool compare_floats(float a, float b) {
    return ((a-b) < 0.0001 && (a-b) > -0.0001);
}


#define DEVICE_INIT                                           \
    struct hx711_device device;                               \
    hx711_init(&device, HX711_Data_GPIO_Port, HX711_Data_Pin, \
        HX711_Clock_GPIO_Port, HX711_Clock_Pin,               \
        true);

#define KNOWN_OBJECT_WEIGHT 100

bool hx711_calibrate() {
    DEVICE_INIT
    hx711_wait_for_ready(&device);
    hx711_tare(&device , 10);
    hx711_calibrate_scale(&device , KNOWN_OBJECT_WEIGHT , 10);
    return true;
}


// bool hx711_test_scale() {
//     DEVICE_INIT
//     if (hx711_set_scale(&device, 0)) {
//         return false;
//     }
//     hx711_set_scale(&device, 0.5);
//     if (!compare_floats(hx711_get_scale(&device), 2 )) {
//         return false;
//     }
//     hx711_set_scale(&device, -0.5);
//     if (!compare_floats(hx711_get_scale(&device), -2)) {
//         return false;
//     }
//     return true;
    
// }

bool hx711_test_scale(struct hx711_device *device) {
    if (hx711_set_scale(device, 0)) {
        return false;
    }
    hx711_set_scale(device, 0.5);
    if (!compare_floats(hx711_get_scale(device), 2 )) {
        return false;
    }
    hx711_set_scale(device, -0.5);
    if (!compare_floats(hx711_get_scale(device), -2)) {
        return false;
    }
    return true;
}


#define TEST_GAIN(gain_val)                                                      \
    if(!(hx711_get_gain(&device) == gain_val)) {                                 \
        printf("Gain test failed (hx711_get_gain(&device) == %d)\n" , gain_val); \
        return false;                                                            \
    }

bool hx711_test_gain() {
    DEVICE_INIT
    
    TEST_GAIN(128)
    TEST_GAIN(64)
    TEST_GAIN(32)
    return true;
}

bool hx711_test_offset() {
    DEVICE_INIT
    if (hx711_get_offset(&device) != 0) {
        return false;
    }
    hx711_set_offset(&device, 100);
    if (hx711_get_offset(&device) != 100) {
        return false;
    }
    return true;
}