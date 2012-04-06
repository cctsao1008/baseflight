#include "board.h"

// ITG3200, Standard address 0x69
#if !defined(ITG3200_ADDRESS) 
  //#define ITG3200_ADDRESS 0x68
  #define ITG3200_ADDRESS 0x69
#endif

// Registers
#define ITG3200_SMPLRT_DIV      0x15
#define ITG3200_DLPF_FS_SYNC    0x16
#define ITG3200_INT_CFG         0x17
#define ITG3200_TEMP_OUT        0x1B
#define ITG3200_GYRO_OUT        0x1D
#define ITG3200_USER_CTRL       0x3D
#define ITG3200_PWR_MGM         0x3E

// Bits
#define ITG3200_FS_SEL_2000DPS  0x18
#define ITG3200_DLPF_10HZ       0x05
#define ITG3200_DLPF_20HZ       0x04
#define ITG3200_DLPF_42HZ       0x03
#define ITG3200_DLPF_98HZ       0x02
#define ITG3200_DLPF_188HZ      0x01
#define ITG3200_DLPF_256HZ      0x00

#define ITG3200_USER_RESET      0x01
#define ITG3200_CLK_SEL_PLL_GX  0x01

static uint8_t mpuLowPassFilter = ITG3200_DLPF_42HZ;

static void ig3200Init(void);
static void ig3200Read(int16_t *gyroData);
static void ig3200Align(int16_t *gyroData);

bool ig3200Detect(sensor_t *gyro)
{
    bool ack;

    delay(25); // datasheet page 13 says 20ms. other stuff could have been running meanwhile. but we'll be safe

    ack = i2cWrite(ITG3200_ADDRESS, ITG3200_SMPLRT_DIV, 0);
    if (!ack)
        return false;

    gyro->init = ig3200Init;
    gyro->read = ig3200Read;
    gyro->align = ig3200Align;

    return true;
}

void ig3200Config(uint16_t lpf)
{
    switch (lpf) {
        case 256:
            mpuLowPassFilter = ITG3200_DLPF_256HZ;
            break;
        case 188:
            mpuLowPassFilter = ITG3200_DLPF_188HZ;
            break;
        case 98:
            mpuLowPassFilter = ITG3200_DLPF_98HZ;
            break;
        case 42:
            mpuLowPassFilter = ITG3200_DLPF_42HZ;
            break;
        case 20:
            mpuLowPassFilter = ITG3200_DLPF_20HZ;
            break;
        case 10:
            mpuLowPassFilter = ITG3200_DLPF_10HZ;
            break;
    }

    i2cWrite(ITG3200_ADDRESS, ITG3200_DLPF_FS_SYNC, ITG3200_FS_SEL_2000DPS | mpuLowPassFilter);
}

static void ig3200Init(void)
{
    bool ack;

    delay(25); // datasheet page 13 says 20ms. other stuff could have been running meanwhile. but we'll be safe

    ack = i2cWrite(ITG3200_ADDRESS, ITG3200_SMPLRT_DIV, 0);
    if (!ack)
        failureMode(3);

    i2cWrite(ITG3200_ADDRESS, ITG3200_DLPF_FS_SYNC, ITG3200_FS_SEL_2000DPS | mpuLowPassFilter);
    i2cWrite(ITG3200_ADDRESS, ITG3200_INT_CFG, 0);
    i2cWrite(ITG3200_ADDRESS, ITG3200_USER_CTRL, ITG3200_USER_RESET);
    i2cWrite(ITG3200_ADDRESS, ITG3200_PWR_MGM, ITG3200_CLK_SEL_PLL_GX);
}

static void ig3200Align(int16_t *gyroData)
{
    // official direction is RPY
    gyroData[0] = gyroData[0] / 4;
    gyroData[1] = gyroData[1] / 4;
    gyroData[2] = -gyroData[2] / 4;
}

// Read 3 gyro values into user-provided buffer. No overrun checking is done.
static void ig3200Read(int16_t *gyroData)
{
    uint8_t buf[6];

	#ifndef modify_it
	if(i2cRead(ITG3200_ADDRESS, ITG3200_GYRO_OUT, 6, buf) == true)
	{
        gyroData[0] = (buf[0] << 8) | buf[1];
        gyroData[1] = (buf[2] << 8) | buf[3];
        gyroData[2] = (buf[4] << 8) | buf[5];
	}
	else
	{
	    LED1_TOGGLE
	}
	#else
    i2cRead(ITG3200_ADDRESS, ITG3200_GYRO_OUT, 6, buf);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
	#endif
}

#ifndef modify_it

#else
static int16_t ig3200ReadTemp(void)
{
    uint8_t buf[2];
    i2cRead(ITG3200_ADDRESS, ITG3200_TEMP_OUT, 2, buf);
    
    return 35 + ((int32_t)(buf[0] << 8 | buf[1]) + 13200) / 280;
}
#endif
