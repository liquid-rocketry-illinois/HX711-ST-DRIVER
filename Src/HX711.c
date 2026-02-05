/**
 * HX711 Driver for STM32H753
 * 
 * Original source from Rob Tillaart's HX711 Driver for Arduino
 * (https://github.com/RobTillaart/HX711)
 */

#define STM32H753xx

#include "HX711.h"

extern TIM_HandleTypeDef htim1;

void TIM_delay_us(TIM_HandleTypeDef *htim , uint16_t us) {
    htim->Instance->CNT = 0;
    while(htim->Instance->CNT < us) { }
}

//  MSB_FIRST optimized shiftIn
//  see datasheet page 5 for timing
static uint8_t hx711_shiftIn(struct hx711_device *device)
{
    //  local variables are faster. (if you insist..)
    uint16_t clk_pin   = device->_clock_GPIO_pin;
    GPIO_TypeDef* clk_port  = device->_clock_GPIO_port;

    uint16_t data_pin  = device->_data_GPIO_pin;
    GPIO_TypeDef* data_port = device->_data_GPIO_port;
    
    uint8_t value = 0;
    uint8_t mask  = 0x80;
    while (mask > 0)
    {
        HAL_GPIO_WritePin(clk_port , clk_pin , GPIO_PIN_SET);
        //  T2  >= 0.2 us
        if(device->_fastProcessor) TIM_delay_us(&htim1 , 1); // To-do : implement delayMicroseconds
        if (HAL_GPIO_ReadPin(data_port , data_pin) == GPIO_PIN_SET)
        {
            value |= mask;
        }

        HAL_GPIO_WritePin(clk_port , clk_pin , GPIO_PIN_RESET);
        //  keep duty cycle ~50%
        if(device->_fastProcessor) TIM_delay_us(&htim1 , 1);
        mask >>= 1;
    }
    return value;
}

static void insertion_sort(float *array , uint8_t size) {
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
// float    hx711_read(struct hx711_device *device) {
//     hx711_wait_for_ready(device , 0);
//     uint32_t value = 0;
//     //  read 24 bits
//     value |= ((uint32_t)hx711_shiftIn(device)) << 16;
//     value |= ((uint32_t)hx711_shiftIn(device)) << 8;
//     value |= ((uint32_t)hx711_shiftIn(device)) << 0;
//     //  set the channel and the gain factor for the next reading
//     for (uint8_t i = 0; i < device->_gain; i++)
//     {
//         HAL_GPIO_WritePin(device->_clock_GPIO_port , device->_clock_GPIO_pin , GPIO_PIN_SET);
//         //  T4  >= 0.2 us
//         if(device->_fastProcessor) delay_us(1);
//         HAL_GPIO_WritePin(device->_clock_GPIO_port , device->_clock_GPIO_pin , GPIO_PIN_RESET);
//         //  keep duty cycle ~50%
//         if(device->_fastProcessor) delay_us(1);
//     }
//     //  convert from unsigned to signed
//     if (value & 0x800000)
//     {
//         value |= 0xFF000000;
//     }
//     device->_lastTimeRead = HAL_GetTick();
//     return (float)( (int32_t)value );
// }
float    hx711_read(struct hx711_device *device) {
    while (HAL_GPIO_ReadPin(device->_data_GPIO_port , device->_data_GPIO_pin) == GPIO_PIN_SET) {
        // to-do yield();
    }
    
    union {
        uint8_t bytes[4];
        uint32_t value;
    } data;

    data.bytes[3] = 0;

    ////// CRITICAL SESSION START //////
    __disable_irq(); // disable interrupts

    //  Pulse the clock pin 24 times to read the data.
    data.bytes[2] = hx711_shiftIn(device);
    data.bytes[1] = hx711_shiftIn(device);
    data.bytes[0] = hx711_shiftIn(device);
   
    //  TABLE 3 page 4 datasheet
    //
    //  CLOCK      CHANNEL      GAIN      m
    //  ------------------------------------
    //   25           A         128       1    //  default
    //   26           B          32       2
    //   27           A          64       3
    //
    //  only default 128 verified,
    //  selection goes through the set_gain(gain)
    //
    uint8_t m = 1;

    if (device->_gain == HX711_CHANNEL_A_GAIN_128) {
        m = 1;
    } else if (device->_gain == HX711_CHANNEL_A_GAIN_64) {
        m = 3;
    } else if (device->_gain == HX711_CHANNEL_B_GAIN_32) {
        m = 2;
    }

    while (m > 0) {
        HAL_GPIO_WritePin(device->_clock_GPIO_port , device->_clock_GPIO_pin , GPIO_PIN_SET);
        //  T4  >= 0.2 us
        if(device->_fastProcessor) TIM_delay_us(&htim1 , 1);
        HAL_GPIO_WritePin(device->_clock_GPIO_port , device->_clock_GPIO_pin , GPIO_PIN_RESET);
        //  keep duty cycle ~50%
        if(device->_fastProcessor) TIM_delay_us(&htim1 , 1);
        m--;
    }

    __enable_irq();
    ////// CRITICAL SESSION END //////
    
    // yield();

    // SIGN extend
    if (data.bytes[2] & 0x80) {
        data.bytes[3] = 0xFF;
    }
    
    device->_lastTimeRead = HAL_GetTick();
    return 1.0 * (int32_t)(data.value);
}

//  get average of multiple raw reads
//  times = 1 or more
float    hx711_read_average(struct hx711_device *device , uint8_t times/* = 10*/) {
    if (times < 1) times = 1;
    
    float sum = 0;
    for (uint8_t i = 0; i < times; i++) {
      sum += hx711_read(device);
      // yield();
    }
    return sum / times;
}
//  get median of multiple raw reads
//  times = 3..15 - odd numbers preferred
float    hx711_read_median(struct hx711_device *device , uint8_t times/* = 7*/) {
    if (times > 15) times = 15;
    if (times < 3)  times = 3;
    float samples[15];
    for (uint8_t i = 0; i < times; i++) {
        samples[i] = hx711_read(device);
        
        // yield();
    }
    insertion_sort(samples, times);
    if (times & 0x01) return samples[times/2];
    return (samples[times/2] + samples[times/2 + 1]) / 2;
}
//  get average of "middle half" of multiple raw reads.
//  times = 3..15 - odd numbers preferred
float    hx711_read_medavg(struct hx711_device *device , uint8_t times/* = 7*/) {
    if (times > 15) times = 15;
    if (times < 3)  times = 3;
    float samples[15];
    for (uint8_t i = 0; i < times; i++)
    {
        samples[i] = hx711_read(device);
        // yield();
    }
    insertion_sort(samples, times);
    float sum = 0;
    //  iterate over 1/4 to 3/4 of the array
    uint8_t count = 0;
    uint8_t first = (times + 2) / 4;
    uint8_t last  = times - first - 1;
    for (uint8_t i = first; i <= last; i++)  //  !! include last one too
    {
        sum += samples[i];
        count++;
    }
    return sum / count;
}
//  get running average over times measurements.
//  the weight alpha can be set to any value between 0 and 1
//  times = 1 or more.
float    hx711_read_runavg(struct hx711_device *device , uint8_t times/* = 7*/, float alpha/* = 0.5*/) {
    if (times < 1)  times = 1;
    if (alpha < 0)  alpha = 0;
    if (alpha > 1)  alpha = 1;

    float val = hx711_read(device);
    for (uint8_t i = 1; i < times; i++) {
        val += alpha * (hx711_read(device) - val);
        // yield();
    }
    return val;
}

///////////////////////////////////////////////////////////////
//
//  MODE
//
//  get set mode for get_value() and indirect get_units().
//  in median and medavg mode only 3..15 samples are allowed.
void     hx711_set_raw_mode(struct hx711_device *device)     { device->_mode = HX711_RAW_MODE;     }
void     hx711_set_average_mode(struct hx711_device *device) { device->_mode = HX711_AVERAGE_MODE; }
void     hx711_set_median_mode(struct hx711_device *device)  { device->_mode = HX711_MEDIAN_MODE;  }
void     hx711_set_medavg_mode(struct hx711_device *device)  { device->_mode = HX711_MEDAVG_MODE;  }
//  set_run_avg will use a default alpha of 0.5.
void     hx711_set_runavg_mode(struct hx711_device *device)  { device->_mode = HX711_RUNAVG_MODE;  }
uint8_t  hx711_get_mode(struct hx711_device *device)         { return device->_mode; }

//  corrected for offset.
//  in HX711_RAW_MODE the parameter times will be ignored.
float    hx711_get_value(struct hx711_device *device , uint8_t times/* = 1*/) {
    float raw;
    switch (device->_mode)
    {
        case HX711_RAW_MODE:
            raw = hx711_read(device);
            break;
        case HX711_AVERAGE_MODE:
            raw = hx711_read_average(device , times);
            break;
        case HX711_MEDIAN_MODE:
            raw = hx711_read_median(device , times);
            break;
        case HX711_MEDAVG_MODE:
            raw = hx711_read_medavg(device , times);
            break;
        case HX711_RUNAVG_MODE:
            raw = hx711_read_runavg(device , times , 0.5);
            break;
        default:
            raw = hx711_read_average(device , times);
            break;
    }
    return raw - device->_offset;
}
//  converted to proper units, corrected for scale.
//  in HX711_RAW_MODE the parameter times will be ignored.
float    hx711_get_units(struct hx711_device *device , uint8_t times/* = 1*/) {
    float units = hx711_get_value(device , times) * device->_scale;
    return units;
}


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
bool     hx711_set_gain(struct hx711_device *device , uint8_t gain/* = HX711_CHANNEL_A_GAIN_128*/, bool forced/* = false*/) {
    if ( (! forced) && (device->_gain == gain) ) return true;
    switch(gain) {
        case HX711_CHANNEL_A_GAIN_128:
        case HX711_CHANNEL_A_GAIN_64:
        case HX711_CHANNEL_B_GAIN_32:
            device->_gain = gain;
            //  set the gain by doing a read
            hx711_read(device);
            return true;
    }
    return false;
}
uint8_t  hx711_get_gain(struct hx711_device *device) { return device->_gain; }


///////////////////////////////////////////////////////////////
//
//  TARE
//  call tare to calibrate zero
void     hx711_tare(struct hx711_device *device , uint8_t times/* = 10*/) { device->_offset = hx711_read_average(device , times); }
float    hx711_get_tare(struct hx711_device *device) { return -device->_offset*device->_scale; }
bool     hx711_is_tare_set(struct hx711_device *device) { return device->_offset != 0; }


///////////////////////////////////////////////////////////////
//
//  CALIBRATION
//
//  SCALE > 0
//  returns false if scale == 0;
bool     hx711_set_scale(struct hx711_device *device , float scale/* = 1.0*/) {
    if (scale == 0) return false;
    device->_scale = 1.0 / scale;
    return true;
}
float    hx711_get_scale(struct hx711_device *device) {
    return 1.0 / device->_scale;
}

//  OFFSET > 0
void     hx711_set_offset(struct hx711_device *device , int32_t offset/* = 0*/) {
    device->_offset = offset;
}
int32_t  hx711_get_offset(struct hx711_device *device) {
    return device->_offset;
}

//  clear the scale
//  call tare() to set the zero offset
//  put a known weight on the scale
//  call calibrate_scale(weight)
//  scale is calculated.
void     hx711_calibrate_scale(struct hx711_device *device , float weight, uint8_t times/* = 10*/) {
    float reading = hx711_read_average(device , times) - device->_offset;
    if (reading != 0) {
        device->_scale = weight / reading;
    }
}


///////////////////////////////////////////////////////////////
//
//  POWER MANAGEMENT
//
void     hx711_power_down(struct hx711_device *device) {
    HAL_GPIO_WritePin(device->_clock_GPIO_port , device->_clock_GPIO_pin , GPIO_PIN_SET);
    TIM_delay_us(&htim1 , 69);
}
void     hx711_power_up(struct hx711_device *device) {
    HAL_GPIO_WritePin(device->_clock_GPIO_port , device->_clock_GPIO_pin , GPIO_PIN_RESET);
}


///////////////////////////////////////////////////////////////
//
//  EXPERIMENTAL
//  RATE PIN - works only if rate pin is exposed.
//
// 
void     hx711_set_rate_pin(struct hx711_device *device , GPIO_TypeDef *rate_port , uint16_t rate_pin) {
    device->_rate_GPIO_port = rate_port;
    device->_rate_GPIO_pin  = rate_pin;
    
    hx711_set_rate_10SPS(device);
}
void     hx711_set_rate_10SPS(struct hx711_device *device) {
    device->_rate = 10;
    HAL_GPIO_WritePin(device->_data_GPIO_port,  device->_clock_GPIO_pin, GPIO_PIN_RESET);
}
void     hx711_set_rate_80SPS(struct hx711_device *device) {
    device->_rate = 80;
    HAL_GPIO_WritePin(device->_data_GPIO_port,  device->_clock_GPIO_pin, GPIO_PIN_SET);
}
uint8_t  hx711_get_rate(struct hx711_device *device) {
    return device->_rate;
}


//  TIME OF LAST READ
uint32_t hx711_last_time_read(struct hx711_device *device) { return device->_lastTimeRead; }


//  PRICING
float    hx711_get_price(struct hx711_device *device , uint8_t times/* = 1*/)      { return hx711_get_units(device , times) * device->_price; }
void     hx711_set_unit_price(struct hx711_device *device , float price/* = 1.0*/) { device->_price = price; }
float    hx711_get_unit_price(struct hx711_device *device)                         { return device->_price; }