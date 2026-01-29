#ifndef __HX711_H
#define __HX711_H

#include "main.h"

typedef char bool;

#define HX711_LIB_VERSION               (F("0.6.3"))

#define HX711_AVERAGE_MODE  0x00
//  in median mode only between 3 and 15 samples are allowed.
#define HX711_MEDIAN_MODE   0x01
//  medavg = average of the middle "half" of sorted elements
//  in medavg mode only between 3 and 15 samples are allowed.
#define HX711_MEDAVG_MODE   0x02
//  runavg = running average
#define HX711_RUNAVG_MODE   0x03
//  causes read() to be called only once!
#define HX711_RAW_MODE      0x04

//  supported values for set_gain()
#define HX711_CHANNEL_A_GAIN_128  128  //  default
#define HX711_CHANNEL_A_GAIN_64  64
#define HX711_CHANNEL_B_GAIN_32  32

struct hx711_device {
    GPIO_TypeDef *_data_GPIO_port;
    uint16_t _data_GPIO_pin;
    GPIO_TypeDef *_clock_GPIO_port;
    uint16_t _clock_GPIO_pin;


    int32_t  _offset;
    float    _scale;
    uint8_t  _gain;
    uint32_t _lastTimeRead;
    uint8_t  _mode;
    bool     _fastProcessor;
    uint8_t  _ratePin;
    uint8_t  _rate;
    float    _price;
};

void hx711_init(struct hx711_device *device , 
    GPIO_TypeDef *data_port , uint16_t data_pin , 
    GPIO_TypeDef *clock_port , uint16_t clock_pin , 
    bool fast_processor , 
    bool do_reset);
void hx711_reset(struct hx711_device *device);
  // Check whether the device is ready. True if it's ready, false if not
bool hx711_is_ready(struct hx711_device *device);
  // Wait for the device to be ready for given ms
void hx711_wait_for_ready(struct hx711_device *device , uint32_t ms);

#define HX711_DEFAULT_WAIT_RETRY_NUM 3
  //  max # retries
bool wait_ready_retry(struct hx711_device *device , uint8_t retries, uint32_t ms);
#define HX711_DEFAULT_WAIT_TIMEOUT  1000
  //  max timeout
bool wait_ready_timeout(struct hx711_device *device , uint32_t timeout, uint32_t ms);





///////////////////////////////////////////////////////////////
//
//  READ
//
//  raw read, is blocking until device is ready to read().
//  this blocking period can be long up to 400 ms in first read() call.
float    hx711_read(struct hx711_device *device);  

//  get average of multiple raw reads
//  times = 1 or more
float    hx711_read_average(struct hx711_device *device , uint8_t times/* = 10*/);
//  get median of multiple raw reads
//  times = 3..15 - odd numbers preferred
float    hx711_read_median(struct hx711_device *device , uint8_t times/* = 7*/);
//  get average of "middle half" of multiple raw reads.
//  times = 3..15 - odd numbers preferred
float    hx711_read_medavg(struct hx711_device *device , uint8_t times/* = 7*/);
//  get running average over times measurements.
//  the weight alpha can be set to any value between 0 and 1
//  times = 1 or more.
float    hx711_read_runavg(struct hx711_device *device , uint8_t times/* = 7*/, float alpha/* = 0.5*/);

///////////////////////////////////////////////////////////////
//
//  MODE
//
//  get set mode for get_value() and indirect get_units().
//  in median and medavg mode only 3..15 samples are allowed.
void     hx711_set_raw_mode(struct hx711_device *device);
void     hx711_set_average_mode(struct hx711_device *device);
void     hx711_set_median_mode(struct hx711_device *device);
void     hx711_set_medavg_mode(struct hx711_device *device);
//  set_run_avg will use a default alpha of 0.5.
void     hx711_set_runavg_mode(struct hx711_device *device);
uint8_t  hx711_get_mode(struct hx711_device *device);

//  corrected for offset.
//  in HX711_RAW_MODE the parameter times will be ignored.
float    hx711_get_value(struct hx711_device *device , uint8_t times/* = 1*/);
//  converted to proper units, corrected for scale.
//  in HX711_RAW_MODE the parameter times will be ignored.
float    hx711_get_units(struct hx711_device *device , uint8_t times/* = 1*/);


///////////////////////////////////////////////////////////////
//
//  GAIN
//
//  CORE "CONSTANTS" -> read datasheet
//  CHANNEL      GAIN   notes
//  -------------------------------------
//     A         128    default, tested
//     A          64
//     B          32

//  returns true  ==>  parameter gain is valid
//  returns false ==>  parameter gain is invalid ==> no change.
//  note that changing gain/channel takes up to 400 ms (page 3)
//  if forced == true, the gain will be forced set
//  even it is already the right value
bool     hx711_set_gain(struct hx711_device *device , uint8_t gain/* = HX711_CHANNEL_A_GAIN_128*/, bool forced/* = false*/);
uint8_t  hx711_get_gain(struct hx711_device *device);


///////////////////////////////////////////////////////////////
//
//  TARE
//  call tare to calibrate zero
void     hx711_tare(struct hx711_device *device , uint8_t times/* = 10*/);
float    hx711_get_tare(struct hx711_device *device);
bool     hx711_tare_set(struct hx711_device *device);


///////////////////////////////////////////////////////////////
//
//  CALIBRATION
//
//  SCALE > 0
//  returns false if scale == 0;
bool     hx711_set_scale(struct hx711_device *device , float scale/* = 1.0*/);
float    hx711_get_scale(struct hx711_device *device);

//  OFFSET > 0
void     hx711_set_offset(struct hx711_device *device , int32_t offset/* = 0*/);
int32_t  hx711_get_offset(struct hx711_device *device);

//  clear the scale
//  call tare() to set the zero offset
//  put a known weight on the scale
//  call calibrate_scale(weight)
//  scale is calculated.
void     hx711_calibrate_scale(struct hx711_device *device , float weight, uint8_t times/* = 10*/);


///////////////////////////////////////////////////////////////
//
//  POWER MANAGEMENT
//
void     hx711_power_down(struct hx711_device *device);
void     hx711_power_up(struct hx711_device *device);


///////////////////////////////////////////////////////////////
//
//  EXPERIMENTAL
//  RATE PIN - works only if rate pin is exposed.
//
void     hx711_set_rate_pin(struct hx711_device *device , uint8_t pin);
void     hx711_set_rate_10SPS(struct hx711_device *device);
void     hx711_set_rate_80SPS(struct hx711_device *device);
uint8_t  hx711_get_rate(struct hx711_device *device);


//  TIME OF LAST READ
uint32_t hx711_last_time_read(struct hx711_device *device);


//  PRICING
float    hx711_get_price(struct hx711_device *device , uint8_t times/* = 1*/)  { return hx711_get_units(device , times) * device->_price; };
void     hx711_set_unit_price(struct hx711_device *device , float price/* = 1.0*/) { device->_price = price; };
float    hx711_get_unit_price(struct hx711_device *device) { return device->_price; };


#endif // __HX711_H