#include "board.h"

// BMA180, Alternative address mode 0x80
#if !defined(BMA180_ADDRESS) 
  #define BMA180_ADDRESS 0x80
  //#define BMA180_ADDRESS 0x82
#endif

#define BMA180_BW_RATE     0x2C
#define BMA180_POWER_CTL   0x2D
#define BMA180_INT_ENABLE  0x2E
#define BMA180_DATA_FORMAT 0x31
#define BMA180_DATA_OUT    0x32
#define BMA180_FIFO_CTL    0x38

#define BMA180_BW_RATE_200 0x0B
#define BMA180_POWER_MEAS  0x08
#define BMA180_FULL_RANGE  0x08
#define BMA180_RANGE_16G   0x03

static void bma180Init(void);
static void bma180Read(int16_t *accelData);
static void bma180Align(int16_t *accelData);

bool bma180Detect(sensor_t *acc)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(BMA180_ADDRESS, 0x00, 1, &sig);
    if (!ack || sig != 0xE5)
        return false;

    acc->init = bma180Init;
    acc->read = bma180Read;
    acc->align = bma180Align;
    return true;
}

#define ADXL_RATE_100      0x0A
#define ADXL_RATE_200      0x0B
#define ADXL_RATE_400      0x0C
#define ADXL_RATE_800      0x0D
#define ADXL_RATE_1600     0x0E
#define ADXL_RATE_3200     0x0F
#define ADXL_FULL_RES      0x08
#define ADXL_RANGE_2G      0x00
#define ADXL_RANGE_4G      0x01
#define ADXL_RANGE_8G      0x02
#define ADXL_RANGE_16G     0x03

static void bma180Init(void)
{
#ifdef FREEFLIGHT
    i2cWrite(BMA180_ADDRESS, BMA180_BW_RATE, BMA180_BW_RATE_200);
    i2cWrite(BMA180_ADDRESS, BMA180_POWER_CTL, BMA180_POWER_MEAS);
    i2cWrite(BMA180_ADDRESS, BMA180_INT_ENABLE, 0);
    i2cWrite(BMA180_ADDRESS, BMA180_DATA_FORMAT, BMA180_FULL_RANGE | BMA180_RANGE_16G);
    i2cWrite(BMA180_ADDRESS, BMA180_FIFO_CTL, 0);
#else
    // MWC defaults
    i2cWrite(BMA180_ADDRESS, BMA180_POWER_CTL, 1 << 3);        //  register: Power CTRL  -- value: Set measure bit 3 on
    i2cWrite(BMA180_ADDRESS, BMA180_DATA_FORMAT, 0x0B);  //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
    i2cWrite(BMA180_ADDRESS, BMA180_BW_RATE, 0x09);  //  register: BW_RATE     -- value: rate=50hz, bw=20hz
#endif /* FreeFlight */
}

static void bma180Read(int16_t *accelData)
{
    static uint8_t buf[6];

	#ifndef modify_it
	if(i2cRead(BMA180_ADDRESS, BMA180_DATA_OUT, 6, buf) == true)
	{
        accelData[0] = buf[0] + (buf[1] << 8);
        accelData[1] = buf[2] + (buf[3] << 8);
        accelData[2] = buf[4] + (buf[5] << 8);
	}
	else
	{
	    LED1_TOGGLE
	}
	#else
    i2cRead(BMA180_ADDRESS, BMA180_DATA_OUT, 6, buf);
    accelData[0] = buf[0] + (buf[1] << 8);
    accelData[1] = buf[2] + (buf[3] << 8);
    accelData[2] = buf[4] + (buf[5] << 8);
	#endif
}

static void bma180Align(int16_t *accData)
{
    // official direction is RPY, nothing to change here.
}
