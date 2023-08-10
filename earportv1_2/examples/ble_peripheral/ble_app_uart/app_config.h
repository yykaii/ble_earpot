#ifndef USER_DEFINE_H__
#define USER_DEFINE_H__

#define USE_BMI160               // 使用九轴数据，需要同时打开USE_BMI160和USE_EXT_BMM150

#if defined(USE_BMI160)
//#define ACC_ONLY
//#define GYRO_ONLY
#define ACC_GYRO
#define USE_EXT_BMM150
//#define DATA_POLL
//#define TAP
//#define NO_MOTION
//#define STEP_COUNTER
#define FIFO_POLL                  // FIFO采集模式
//#define FIFO_WM_INT
#define SUPPORT_LOWPOWER         //enable BMX160 support lowpower
#endif

#define MAX_BYTES_PER_PACKET  20   //刚开始调试，按照协议最大包定义的，暂时不用

/*----------------硬件GPIO定义---------------------*/
/*spi interface*/
#define SPI_SS_PIN          0
#define SPI_MISO_PIN        12
#define SPI_MOSI_PIN        14
#define SPI_SCK_PIN         1

#define GPIO_CHARGE         21 // control LED RED   
#define GPIO_VBAT_SENSE     5 
#define GPIO_VBUS_SENSE     4 //csy_0308  second pcb version add;
#define GPIO_LED_BLUE       18
#define GPIO_MEMS_PWR_EN    16
#define GPIO_BMX160_INT     20//csy_0308  second pcb version add;

/* BMX160
P0.4:TWI(I2C) SDA
P0.1:TWI(I2C) SCL
*/

#define TWI_SCL_M           1  // I2C SCL
#define TWI_SDA_M           4  // I2C SDA

/*----------------设备SN定义，修改生产不同设备固件------------------------*/
#define DEVICE_SN                       "AXOL2201000011" //csy: modify device SN number
//#define DEVICE_SN                       "AXOL2201000022"
//#define DEVICE_SN                       "AXOL2201000033"

/*----------------设备名称定义------------------------*/
#define DEVICE_NAME                     "EarPot"             /**< Name of device. Will be included in the advertising data. */


/*----------------传感器操作接口定义------------------*/
/* local macro definitions */
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_I2C     0

/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_SPI     1

#if (!((BMI160_INTERFACE_I2C == 1) && (BMI160_INTERFACE_SPI == 0)) && \
	 (!((BMI160_INTERFACE_I2C == 0) && (BMI160_INTERFACE_SPI == 1))))
#error "Invalid value given for the macros BMI160_INTERFACE_I2C / BMI160_INTERFACE_SPI"
#endif

#define FIFO_SIZE	   1024

#endif
