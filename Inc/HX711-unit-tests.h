#ifndef _HX711_UNIT_TESTS_H_
#define _HX711_UNIT_TESTS_H_
#include "HX711.h"
/********************** UNIT TESTING CODES **********************/

bool hx711_calibrate();
bool hx711_test();
bool hx711_test_read();
bool hx711_test_gain();
// bool hx711_test_scale();
bool hx711_test_scale(struct hx711_device *device);
bool compare_floats(float a, float b);
#endif