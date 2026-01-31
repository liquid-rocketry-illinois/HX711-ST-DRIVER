#define STM32H753xx

#include <stm32h7xx_hal.h>
#include "HX711.h"

//  MSB_FIRST optimized shiftIn
//  see datasheet page 5 for timing
static uint8_t hx711_shiftIn(struct hx711_device *device)
{
    //  local variables are faster. (if you insist..)
    uint8_t clk_pin   = device->_clock_GPIO_pin;
    uint8_t clk_port  = device->_clock_GPIO_port;

    uint8_t data_pin  = device->_data_GPIO_pin;
    uint8_t data_port = device->_data_GPIO_port;
    
    uint8_t value = 0;
    uint8_t mask  = 0x80;
    while (mask > 0)
    {
        HAL_GPIO_WritePin(clk_port , clk_pin , GPIO_PIN_SET);
        //  T2  >= 0.2 us
        if(device->_fastProcessor) delayMicroseconds(1); // To-do : implement delayMicroseconds
        if (HAL_GPIO_ReadPin(data_port , data_pin) == GPIO_PIN_SET)
        {
            value |= mask;
        }

        HAL_GPIO_WritePin(clk_port , clk_pin , GPIO_PIN_RESET);
        //  keep duty cycle ~50%
        if(device->_fastProcessor) delayMicroseconds(1);
        mask >>= 1;
    }
    return value;
}

static insertion_sort(float *array , uint8_t size) {
    uint8_t t, z;
    float temp;
    for (t = 1; t < size; t++)
    {
        z = t;
        temp = array[z];
        while( (z > 0) && (temp < array[z - 1] ))
        {
            array[z] = array[z - 1];
            z--;
        }
        array[z] = temp;
        // To-do : Implement scheduler and the yield() function
        // yield();
    }
}

static void init_device_struct(struct hx711_device *device) {
    device->_offset   = 0;
    device->_scale    = 1;
    device->_gain     = HX711_CHANNEL_A_GAIN_128;
    device->_lastTimeRead = 0;
    device->_mode     = HX711_AVERAGE_MODE;
    device->_price    = 0;
}

/// @brief Initialize the device struct and the HX711 device itself
/// @param device The HX711 device struct
/// @param data_port GPIO port of the data input, the data port should be configured as INPUT
/// @param data_pin  GPIO pin of the data input
/// @param clock_port GPIO port of the clock output, the clock port should be configured as OUTPUT
/// @param clock_pin  GPIO pin of the clock output
/// @param fast_processor 
/// @param do_reset If true, the device will be reset during the initialization
void hx711_init(struct hx711_device *device , 
                GPIO_TypeDef *data_port , uint16_t data_pin , 
                GPIO_TypeDef *clock_port , uint16_t clock_pin , 
                bool fast_processor , 
                bool do_reset) {
    init_device_struct(device);
    device->_fastProcessor = fast_processor;
    
    device->_data_GPIO_port = data_port;
    device->_data_GPIO_pin  = data_pin;
    
    device->_clock_GPIO_port = clock_port;
    device->_clock_GPIO_pin  = clock_pin;

    // Enable the pullup register for the data port
    device->_data_GPIO_port->ODR |= (1 << device->_data_GPIO_pin);
    HAL_GPIO_WritePin(device->_clock_GPIO_port , device->_clock_GPIO_pin , GPIO_PIN_RESET);
    if(do_reset) hx711_reset(device);
}

void hx711_reset(struct hx711_device *device) {
    hx711_power_down(device);
    hx711_power_up(device);
    init_device_struct(device);
}
  // Check whether the device is ready
bool hx711_is_ready(struct hx711_device *device) {
    return (HAL_GPIO_ReadPin(device->_data_GPIO_port , device->_data_GPIO_pin) == GPIO_PIN_RESET);
}
  // Wait for the device to be ready for given ms
  // To-do : Yield to other processor when reading + Add MutEx
void hx711_wait_for_ready(struct hx711_device *device , uint32_t ms) {
    // mutex_lock(device->mutex);
    while(!hx711_is_ready(device)) {
        HAL_Delay(ms);  // delay_schedule(ms); something like that
    }
    // mutex_unlock(device->mutex);
}

#define HX711_DEFAULT_WAIT_RETRY_NUM 3
  //  max # retries
bool wait_ready_retry(struct hx711_device *device , uint8_t retries, uint32_t ms) {
    while(retries--) {
        if(hx711_is_ready(device)) return true;
        HAL_Delay(ms);
    }
    return false;
}

#define HX711_DEFAULT_WAIT_TIMEOUT  1000
  //  max timeout
bool wait_ready_timeout(struct hx711_device *device , uint32_t timeout, uint32_t ms) {
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout)
    {
        if (hx711_is_ready(device)) return true;
        HAL_Delay(ms);
    }
    return false;
}


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
