/*
 generated hardware definitions from hwdef.dat - DO NOT EDIT
*/

#pragma once

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define MHZ (1000U*1000U)
#define KHZ (1000U)


#define AP_SIGNED_FIRMWARE 0

#define HAL_ENABLE_DFU_BOOT FALSE
#define CHIBIOS_BOARD_NAME "Pixhawk6C"
// MCU type (ChibiOS define)
#define STM32H7xx_MCUCONF
#define STM32H743_MCUCONF

#define STM32H743xx

// crystal frequency
#define STM32_HSECLK 16000000U

// UART used for stdout (printf)
// SDMMC2 available, enable POSIX filesystem support
#define USE_POSIX

#define HAL_USE_SDC TRUE
#define STM32_SDC_USE_SDMMC2 TRUE
#define HAL_USE_SDMMC 1
#define STM32_USB_USE_OTG1                  TRUE
#define STM32_OTG2_IS_OTG1                  FALSE
#define HAL_USE_USB TRUE
#define HAL_USE_SERIAL_USB TRUE
#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 1
#endif
#ifndef HAL_WITH_MCU_MONITORING
#define HAL_WITH_MCU_MONITORING 1
#endif
#ifndef STM32H7
#define STM32H7 1
#endif
#define HAL_USE_HW_RNG TRUE
#define HAL_PROCESS_STACK_SIZE 0x1C00
#define STM32_ST_USE_TIMER 2
#define HAL_BATT_VOLT_PIN 8
#define HAL_BATT_CURR_PIN 4
#define HAL_BATT_VOLT_SCALE 18.18
#define HAL_BATT_CURR_SCALE 36.36
#define HAL_I2C_INTERNAL_MASK 1
#define HAL_HEATER_GPIO_PIN 80
#define HAL_HAVE_IMU_HEATER 1
#define HAL_IMU_TEMP_DEFAULT 45
#define HAL_IMUHEAT_P_DEFAULT 50
#define HAL_IMUHEAT_I_DEFAULT 0.07
#define HAL_GPIO_SPEKTRUM_PWR 73
#define HAL_SPEKTRUM_PWR_ENABLED 1
#define FATFS_HAL_DEVICE SDCD2
#define AP_NOTIFY_GPIO_LED_2_ENABLED 1
#define HAL_GPIO_A_LED_PIN 90
#define HAL_GPIO_B_LED_PIN 92
#define HAL_BARO_ALLOW_INIT_NO_BARO 1
#define HAL_PROBE_EXTERNAL_I2C_COMPASSES
#define HAL_HEATER_MAG_OFFSET {AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_I2C,0,0xc,0xa),Vector3f(17,14,0)}
#define HAL_COMPASS_DISABLE_IST8310_INTERNAL_PROBE
#define HAL_DEFAULT_INS_FAST_SAMPLE 3
#define HAL_STORAGE_SIZE 32768
#define HAL_WITH_RAMTRON 1
#define HAL_HAVE_SAFETY_SWITCH 1
#define HAL_OS_FATFS_IO 1
#define HAL_WITH_IO_MCU_BIDIR_DSHOT 1
#define HAL_CAN_IFACE1_ENABLE
#define HAL_CAN_IFACE2_ENABLE
#define HAL_CAN_INTERFACE_LIST 0,1
#define HAL_CAN_INTERFACE_REV_LIST 0,1,-1
#define HAL_CAN_BASE_LIST reinterpret_cast<CanType*>(uintptr_t(FDCAN1_BASE)),reinterpret_cast<CanType*>(uintptr_t(FDCAN2_BASE))
#define HAL_NUM_CAN_IFACES 2
#define HAL_CANFD_SUPPORTED 4
#define BOARD_FLASH_SIZE 2048

// location of loaded firmware
#define FLASH_LOAD_ADDRESS 0x08020000
#define EXT_FLASH_SIZE_MB 0
#define EXT_FLASH_RESERVE_START_KB 0
#define EXT_FLASH_RESERVE_END_KB 0
#define CRT0_AREAS_NUMBER 1
#define __EXTFLASHFUNC__
#ifndef AP_CRASHDUMP_ENABLED
#define AP_CRASHDUMP_ENABLED 1
#endif

#define HAL_RAM0_START 0x20000000
// memory regions
#define HAL_MEMORY_REGIONS {(void*)0x24000000, 0x00080000, 0x04 }, {(void*)0x30000000, 0x00040000, 0x08 }, {(void*)0x20000000, 0x00020000, 0x02 }, {(void*)0x00000400, 0x0000fc00, 0x02 }, {(void*)0x30040000, 0x00008000, 0x08 }, {(void*)0x38000000, 0x00010000, 0x01 }
#define HAL_CC_MEMORY_REGIONS {0x24000000, 0x24080000, CRASH_CATCHER_BYTE }, {0x30000000, 0x30040000, CRASH_CATCHER_BYTE }, {0x20000000, 0x20020000, CRASH_CATCHER_BYTE }, {0x00000400, 0x00010000, CRASH_CATCHER_BYTE }, {0x30040000, 0x30048000, CRASH_CATCHER_BYTE }, {0x38000000, 0x38010000, CRASH_CATCHER_BYTE }
#define HAL_MEMORY_TOTAL_KB 1055

// CPU serial number (12 bytes)
#define UDID_START UID_BASE


// APJ board ID (for bootloaders)
#define APJ_BOARD_ID 56

#ifndef HAL_ENABLE_THREAD_STATISTICS
#define HAL_ENABLE_THREAD_STATISTICS FALSE
#endif
    
#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 0
#endif
#define HAL_EXPECTED_SYSCLOCK 400000000
#define HAL_EXPECTED_STM32_SYS_CK 400000000
#define HAL_EXPECTED_STM32_QSPICLK 200000000
#define HAL_EXPECTED_STM32_SDMMC1CLK 80000000
#define HAL_EXPECTED_STM32_SPI45CLK 100000000
#define HAL_EXPECTED_STM32_FDCANCLK 80000000
#define STM32_DMA_REQUIRED TRUE


#ifndef HAL_FLASH_PROTECTION
#define HAL_FLASH_PROTECTION 0
#endif
#define HAL_SPI1_CONFIG { &SPID1, 1, STM32_SPI_SPI1_DMA_STREAMS, PAL_LINE(GPIOA,5U) }
#define HAL_SPI2_CONFIG { &SPID2, 2, STM32_SPI_SPI2_DMA_STREAMS, PAL_LINE(GPIOD,3U) }
#define HAL_SPI_BUS_LIST HAL_SPI1_CONFIG,HAL_SPI2_CONFIG


// SPI device table
#define HAL_SPI_DEVICE0  SPIDesc("bmi055_g"       ,  0,  1, PAL_LINE(GPIOC,14U), SPIDEV_MODE3,  10*MHZ,  10*MHZ)
#define HAL_SPI_DEVICE1  SPIDesc("bmi055_a"       ,  0,  2, PAL_LINE(GPIOC,15U), SPIDEV_MODE3,  10*MHZ,  10*MHZ)
#define HAL_SPI_DEVICE2  SPIDesc("icm42688"       ,  0,  3, PAL_LINE(GPIOC,13U), SPIDEV_MODE3,   2*MHZ,   8*MHZ)
#define HAL_SPI_DEVICE3  SPIDesc("ramtron"        ,  1,  1, PAL_LINE(GPIOD,4U) , SPIDEV_MODE3,   8*MHZ,   8*MHZ)
#define HAL_SPI_DEVICE_LIST HAL_SPI_DEVICE0,HAL_SPI_DEVICE1,HAL_SPI_DEVICE2,HAL_SPI_DEVICE3

#define HAL_WITH_SPI_BMI055_G 1
#define HAL_WITH_SPI_BMI055_A 1
#define HAL_WITH_SPI_ICM42688 1
#define HAL_WITH_SPI_RAMTRON 1

// ADC config
#define ANALOG_VCC_5V_PIN 18
#define HAL_HAVE_BOARD_VOLTAGE 1
#define HAL_ANALOG_PINS \
{  4,  4,    3.30/4096 }, /* PC4 BATT_CURRENT_SENS */ \
{  5,  5,    3.30/4096 }, /* PB1 BATT2_VOLTAGE_SENS */ \
{  8,  8,    3.30/4096 }, /* PC5 BATT_VOLTAGE_SENS */ \
{ 14, 14,    3.30/4096 }, /* PA2 BATT2_CURRENT_SENS */ \
{ 18, 18,  2*3.30/4096 }, /* PA4 VDD_5V_SENS */ \


// GPIO config
#define HAL_GPIO_LINE_GPIO50 PAL_LINE(GPIOA,8U)
#define HAL_GPIO_LINE_GPIO51 PAL_LINE(GPIOE,11U)
#define HAL_GPIO_LINE_GPIO52 PAL_LINE(GPIOE,13U)
#define HAL_GPIO_LINE_GPIO53 PAL_LINE(GPIOE,14U)
#define HAL_GPIO_LINE_GPIO54 PAL_LINE(GPIOD,14U)
#define HAL_GPIO_LINE_GPIO55 PAL_LINE(GPIOD,15U)
#define HAL_GPIO_LINE_GPIO56 PAL_LINE(GPIOA,0U)
#define HAL_GPIO_LINE_GPIO57 PAL_LINE(GPIOA,1U)
#define HAL_GPIO_LINE_GPIO73 PAL_LINE(GPIOH,2U)
#define HAL_GPIO_LINE_GPIO77 PAL_LINE(GPIOB,0U)
#define HAL_GPIO_LINE_GPIO80 PAL_LINE(GPIOB,9U)
#define HAL_GPIO_LINE_GPIO90 PAL_LINE(GPIOD,10U)
#define HAL_GPIO_LINE_GPIO92 PAL_LINE(GPIOD,11U)
#define HAL_GPIO_PINS { \
{  50, true,  1, PAL_LINE(GPIOA,8U)}, /* PA8 TIM1_CH1 TIM1 AF1 PWM1 */ \
{  51, true,  2, PAL_LINE(GPIOE,11U)}, /* PE11 TIM1_CH2 TIM1 AF1 PWM2 */ \
{  52, true,  3, PAL_LINE(GPIOE,13U)}, /* PE13 TIM1_CH3 TIM1 AF1 PWM3 */ \
{  53, true,  4, PAL_LINE(GPIOE,14U)}, /* PE14 TIM1_CH4 TIM1 AF1 PWM4 */ \
{  54, true,  5, PAL_LINE(GPIOD,14U)}, /* PD14 TIM4_CH3 TIM4 AF2 PWM5 */ \
{  55, true,  6, PAL_LINE(GPIOD,15U)}, /* PD15 TIM4_CH4 TIM4 AF2 PWM6 */ \
{  56, true,  7, PAL_LINE(GPIOA,0U)}, /* PA0 TIM5_CH1 TIM5 AF2 PWM7 */ \
{  57, true,  8, PAL_LINE(GPIOA,1U)}, /* PA1 TIM5_CH2 TIM5 AF2 PWM8 */ \
{  73, true,  0, PAL_LINE(GPIOH,2U)}, /* PH2 SPEKTRUM_PWR OUTPUT */ \
{  77, true,  0, PAL_LINE(GPIOB,0U)}, /* PB0 TIM3_CH3 TIM3 AF2 */ \
{  80, true,  0, PAL_LINE(GPIOB,9U)}, /* PB9 HEATER_EN OUTPUT */ \
{  90, true,  0, PAL_LINE(GPIOD,10U)}, /* PD10 LED_RED OUTPUT */ \
{  92, true,  0, PAL_LINE(GPIOD,11U)}, /* PD11 LED_BLUE OUTPUT */ \
}

// full pin define list
#define HAL_GPIO_PIN_BATT2_CURRENT_SENS   PAL_LINE(GPIOA,2U)
#define HAL_GPIO_PIN_BATT2_VOLTAGE_SENS   PAL_LINE(GPIOB,1U)
#define HAL_GPIO_PIN_BATT_CURRENT_SENS    PAL_LINE(GPIOC,4U)
#define HAL_GPIO_PIN_BATT_VOLTAGE_SENS    PAL_LINE(GPIOC,5U)
#define HAL_GPIO_PIN_BMI055_A_CS          PAL_LINE(GPIOC,15U)
#define HAL_GPIO_PIN_BMI055_DRDY_A        PAL_LINE(GPIOE,4U)
#define HAL_GPIO_PIN_BMI055_DRDY_G        PAL_LINE(GPIOE,5U)
#define HAL_GPIO_PIN_BMI055_G_CS          PAL_LINE(GPIOC,14U)
#define HAL_GPIO_PIN_CAN1_RX              PAL_LINE(GPIOD,0U)
#define HAL_GPIO_PIN_CAN1_TX              PAL_LINE(GPIOD,1U)
#define HAL_GPIO_PIN_CAN2_RX              PAL_LINE(GPIOB,5U)
#define HAL_GPIO_PIN_CAN2_TX              PAL_LINE(GPIOB,13U)
#define HAL_GPIO_PIN_FRAM_CS              PAL_LINE(GPIOD,4U)
#define HAL_GPIO_PIN_HEATER_EN            PAL_LINE(GPIOB,9U)
#define HAL_GPIO_PIN_HW_VER_REV_DRIVE     PAL_LINE(GPIOE,12U)
#define HAL_GPIO_PIN_I2C1_SDA             PAL_LINE(GPIOB,7U)
#define HAL_GPIO_PIN_I2C1_SCL             PAL_LINE(GPIOB,8U)
#define HAL_GPIO_PIN_I2C1_SDA             PAL_LINE(GPIOB,7U)
#define HAL_GPIO_PIN_I2C2_SCL             PAL_LINE(GPIOB,10U)
#define HAL_GPIO_PIN_I2C2_SDA             PAL_LINE(GPIOB,11U)
#define HAL_GPIO_PIN_I2C2_SCL             PAL_LINE(GPIOB,10U)
#define HAL_GPIO_PIN_I2C4_SCL             PAL_LINE(GPIOD,12U)
#define HAL_GPIO_PIN_I2C4_SDA             PAL_LINE(GPIOD,13U)
#define HAL_GPIO_PIN_I2C4_SCL             PAL_LINE(GPIOD,12U)
#define HAL_GPIO_PIN_ICM42688_CS          PAL_LINE(GPIOC,13U)
#define HAL_GPIO_PIN_ICM42688_DRDY        PAL_LINE(GPIOE,6U)
#define HAL_GPIO_PIN_JTCK_SWCLK           PAL_LINE(GPIOA,14U)
#define HAL_GPIO_PIN_JTMS_SWDIO           PAL_LINE(GPIOA,13U)
#define HAL_GPIO_PIN_LED_BLUE             PAL_LINE(GPIOD,11U)
#define HAL_GPIO_PIN_LED_RED              PAL_LINE(GPIOD,10U)
#define HAL_GPIO_PIN_OTG_FS_DM            PAL_LINE(GPIOA,11U)
#define HAL_GPIO_PIN_OTG_FS_DP            PAL_LINE(GPIOA,12U)
#define HAL_GPIO_PIN_SDMMC2_CK            PAL_LINE(GPIOD,6U)
#define HAL_GPIO_PIN_SDMMC2_CMD           PAL_LINE(GPIOD,7U)
#define HAL_GPIO_PIN_SDMMC2_D0            PAL_LINE(GPIOB,14U)
#define HAL_GPIO_PIN_SDMMC2_D1            PAL_LINE(GPIOB,15U)
#define HAL_GPIO_PIN_SDMMC2_D2            PAL_LINE(GPIOB,3U)
#define HAL_GPIO_PIN_SDMMC2_D3            PAL_LINE(GPIOB,4U)
#define HAL_GPIO_PIN_SPEKTRUM_PWR         PAL_LINE(GPIOH,2U)
#define HAL_GPIO_PIN_SPI1_MISO            PAL_LINE(GPIOA,6U)
#define HAL_GPIO_PIN_SPI1_MOSI            PAL_LINE(GPIOA,7U)
#define HAL_GPIO_PIN_SPI1_SCK             PAL_LINE(GPIOA,5U)
#define HAL_GPIO_PIN_SPI2_MISO            PAL_LINE(GPIOC,2U)
#define HAL_GPIO_PIN_SPI2_MOSI            PAL_LINE(GPIOC,3U)
#define HAL_GPIO_PIN_SPI2_SCK             PAL_LINE(GPIOD,3U)
#define HAL_GPIO_PIN_TIM1_CH1             PAL_LINE(GPIOA,8U)
#define HAL_GPIO_PIN_TIM1_CH2             PAL_LINE(GPIOE,11U)
#define HAL_GPIO_PIN_TIM1_CH3             PAL_LINE(GPIOE,13U)
#define HAL_GPIO_PIN_TIM1_CH4             PAL_LINE(GPIOE,14U)
#define HAL_GPIO_PIN_TIM3_CH3             PAL_LINE(GPIOB,0U)
#define HAL_GPIO_PIN_TIM4_CH3             PAL_LINE(GPIOD,14U)
#define HAL_GPIO_PIN_TIM4_CH4             PAL_LINE(GPIOD,15U)
#define HAL_GPIO_PIN_TIM5_CH1             PAL_LINE(GPIOA,0U)
#define HAL_GPIO_PIN_TIM5_CH2             PAL_LINE(GPIOA,1U)
#define HAL_GPIO_PIN_UART5_CTS            PAL_LINE(GPIOC,9U)
#define HAL_GPIO_PIN_UART5_RTS            PAL_LINE(GPIOC,8U)
#define HAL_GPIO_PIN_UART5_RX             PAL_LINE(GPIOD,2U)
#define HAL_GPIO_PIN_UART5_TX             PAL_LINE(GPIOC,12U)
#define HAL_GPIO_PIN_UART7_CTS            PAL_LINE(GPIOE,10U)
#define HAL_GPIO_PIN_UART7_RTS            PAL_LINE(GPIOE,9U)
#define HAL_GPIO_PIN_UART7_RX             PAL_LINE(GPIOE,7U)
#define HAL_GPIO_PIN_UART7_TX             PAL_LINE(GPIOE,8U)
#define HAL_GPIO_PIN_UART8_RX             PAL_LINE(GPIOE,0U)
#define HAL_GPIO_PIN_UART8_TX             PAL_LINE(GPIOE,1U)
#define HAL_GPIO_PIN_USART1_RX            PAL_LINE(GPIOA,10U)
#define HAL_GPIO_PIN_USART1_TX            PAL_LINE(GPIOB,6U)
#define HAL_GPIO_PIN_USART2_RX            PAL_LINE(GPIOA,3U)
#define HAL_GPIO_PIN_USART2_TX            PAL_LINE(GPIOD,5U)
#define HAL_GPIO_PIN_USART3_RX            PAL_LINE(GPIOD,9U)
#define HAL_GPIO_PIN_USART3_TX            PAL_LINE(GPIOD,8U)
#define HAL_GPIO_PIN_USART6_RX            PAL_LINE(GPIOC,7U)
#define HAL_GPIO_PIN_USART6_TX            PAL_LINE(GPIOC,6U)
#define HAL_GPIO_PIN_VBUS                 PAL_LINE(GPIOA,9U)
#define HAL_GPIO_PIN_VDD_3V3_SENSORS1_EN  PAL_LINE(GPIOB,2U)
#define HAL_GPIO_PIN_VDD_5V_HIPOWER_nOC   PAL_LINE(GPIOC,11U)
#define HAL_GPIO_PIN_VDD_5V_PERIPH_nOC    PAL_LINE(GPIOE,3U)
#define HAL_GPIO_PIN_VDD_5V_SENS          PAL_LINE(GPIOA,4U)
#define HAL_GPIO_PIN_VDD_BRICK2_nVALID    PAL_LINE(GPIOB,12U)
#define HAL_GPIO_PIN_VDD_BRICK_nVALID     PAL_LINE(GPIOA,15U)
#define HAL_GPIO_PIN_nVDD_5V_HIPOWER_EN   PAL_LINE(GPIOC,10U)
#define HAL_GPIO_PIN_nVDD_5V_PERIPH_EN    PAL_LINE(GPIOE,2U)

#define HAL_INS_PROBE1  ADD_BACKEND(AP_InertialSensor_Invensensev3::probe(*this,hal.spi->get_device("icm42688"),ROTATION_PITCH_180_YAW_90))
#define HAL_INS_PROBE2  ADD_BACKEND(AP_InertialSensor_BMI055::probe(*this,hal.spi->get_device("bmi055_a"),hal.spi->get_device("bmi055_g"),ROTATION_PITCH_180))
#define HAL_INS_PROBE3  ADD_BACKEND(AP_InertialSensor_BMI088::probe(*this,hal.spi->get_device("bmi055_a"),hal.spi->get_device("bmi055_g"),ROTATION_PITCH_180))
#define HAL_INS_PROBE_LIST HAL_INS_PROBE1;HAL_INS_PROBE2;HAL_INS_PROBE3

#define HAL_MAG_PROBE1  ADD_BACKEND(DRIVER_IST8310, AP_Compass_IST8310::probe(GET_I2C_DEVICE(0,0x0c),false,ROTATION_NONE))
#define HAL_MAG_PROBE_LIST HAL_MAG_PROBE1

#define HAL_BARO_PROBE1  ADD_BACKEND(AP_Baro_MS56XX::probe(*this,GET_I2C_DEVICE(0,0x76)))
#define HAL_BARO_PROBE2  ADD_BACKEND(AP_Baro_MS56XX::probe(*this,GET_I2C_DEVICE(0,0x77)))
#define HAL_BARO_PROBE3  ADD_BACKEND(AP_Baro_BMP388::probe(*this,GET_I2C_DEVICE(0,0x77)))
#define HAL_BARO_PROBE_LIST HAL_BARO_PROBE1;HAL_BARO_PROBE2;HAL_BARO_PROBE3


#ifndef AP_FILESYSTEM_ROMFS_ENABLED
#define AP_FILESYSTEM_ROMFS_ENABLED 1
#endif
// peripherals enabled
#define STM32_I2C_USE_I2C1                  TRUE
#define STM32_I2C_USE_I2C2                  TRUE
#define STM32_I2C_USE_I2C4                  TRUE
#define STM32_SPI_USE_SPI1                  TRUE
#define STM32_SPI_USE_SPI2                  TRUE
#ifndef STM32_SERIAL_USE_UART5 
#define STM32_SERIAL_USE_UART5  TRUE
#endif
#ifndef STM32_SERIAL_USE_UART7 
#define STM32_SERIAL_USE_UART7  TRUE
#endif
#ifndef STM32_SERIAL_USE_UART8 
#define STM32_SERIAL_USE_UART8  TRUE
#endif
#ifndef STM32_SERIAL_USE_USART1
#define STM32_SERIAL_USE_USART1 TRUE
#endif
#ifndef STM32_SERIAL_USE_USART2
#define STM32_SERIAL_USE_USART2 TRUE
#endif
#ifndef STM32_SERIAL_USE_USART3
#define STM32_SERIAL_USE_USART3 TRUE
#endif
#ifndef STM32_SERIAL_USE_USART6
#define STM32_SERIAL_USE_USART6 TRUE
#endif
#define AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED 1


// auto-generated DMA mapping from dma_resolver.py
#define STM32_ADC_ADC1_DMA_STREAM      STM32_DMA_STREAM_ID(1, 1)
#define STM32_ADC_ADC1_DMA_CHAN        STM32_DMAMUX1_ADC1
#define STM32_SPI_SPI1_RX_DMA_STREAM   STM32_DMA_STREAM_ID(2, 4) // shared SPI1_RX,SPI2_TX,TIM1_UP
#define STM32_SPI_SPI1_RX_DMA_CHAN     STM32_DMAMUX1_SPI1_RX
#define STM32_SPI_SPI1_TX_DMA_STREAM   STM32_DMA_STREAM_ID(2, 3) // shared USART1_TX,SPI1_TX,TIM4_UP
#define STM32_SPI_SPI1_TX_DMA_CHAN     STM32_DMAMUX1_SPI1_TX
#define STM32_SPI_SPI2_RX_DMA_STREAM   STM32_DMA_STREAM_ID(2, 1) // shared USART3_TX,SPI2_RX
#define STM32_SPI_SPI2_RX_DMA_CHAN     STM32_DMAMUX1_SPI2_RX
#define STM32_SPI_SPI2_TX_DMA_STREAM   STM32_DMA_STREAM_ID(2, 4) // shared SPI1_RX,SPI2_TX,TIM1_UP
#define STM32_SPI_SPI2_TX_DMA_CHAN     STM32_DMAMUX1_SPI2_TX
#define STM32_TIM_TIM1_UP_DMA_STREAM   STM32_DMA_STREAM_ID(2, 4) // shared SPI1_RX,SPI2_TX,TIM1_UP
#define STM32_TIM_TIM1_UP_DMA_CHAN     STM32_DMAMUX1_TIM1_UP
#define STM32_TIM_TIM4_UP_DMA_STREAM   STM32_DMA_STREAM_ID(2, 3) // shared USART1_TX,SPI1_TX,TIM4_UP
#define STM32_TIM_TIM4_UP_DMA_CHAN     STM32_DMAMUX1_TIM4_UP
#define STM32_TIM_TIM5_UP_DMA_STREAM   STM32_DMA_STREAM_ID(2, 2) // shared USART2_TX,TIM5_UP
#define STM32_TIM_TIM5_UP_DMA_CHAN     STM32_DMAMUX1_TIM5_UP
#define STM32_UART_UART5_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 3)
#define STM32_UART_UART5_RX_DMA_CHAN   STM32_DMAMUX1_UART5_RX
#define STM32_UART_UART5_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 0)
#define STM32_UART_UART5_TX_DMA_CHAN   STM32_DMAMUX1_UART5_TX
#define STM32_UART_UART7_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 2)
#define STM32_UART_UART7_RX_DMA_CHAN   STM32_DMAMUX1_UART7_RX
#define STM32_UART_UART7_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 6)
#define STM32_UART_UART7_TX_DMA_CHAN   STM32_DMAMUX1_UART7_TX
#define STM32_UART_UART8_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 4)
#define STM32_UART_UART8_RX_DMA_CHAN   STM32_DMAMUX1_UART8_RX
#define STM32_UART_UART8_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 5)
#define STM32_UART_UART8_TX_DMA_CHAN   STM32_DMAMUX1_UART8_TX
#define STM32_UART_USART1_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 5)
#define STM32_UART_USART1_RX_DMA_CHAN  STM32_DMAMUX1_USART1_RX
#define STM32_UART_USART1_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 3) // shared USART1_TX,SPI1_TX,TIM4_UP
#define STM32_UART_USART1_TX_DMA_CHAN  STM32_DMAMUX1_USART1_TX
#define STM32_UART_USART2_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 6)
#define STM32_UART_USART2_RX_DMA_CHAN  STM32_DMAMUX1_USART2_RX
#define STM32_UART_USART2_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 2) // shared USART2_TX,TIM5_UP
#define STM32_UART_USART2_TX_DMA_CHAN  STM32_DMAMUX1_USART2_TX
#define STM32_UART_USART3_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 7)
#define STM32_UART_USART3_RX_DMA_CHAN  STM32_DMAMUX1_USART3_RX
#define STM32_UART_USART3_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 1) // shared USART3_TX,SPI2_RX
#define STM32_UART_USART3_TX_DMA_CHAN  STM32_DMAMUX1_USART3_TX
#define STM32_UART_USART6_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 0)
#define STM32_UART_USART6_RX_DMA_CHAN  STM32_DMAMUX1_USART6_RX
#define STM32_UART_USART6_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 7)
#define STM32_UART_USART6_TX_DMA_CHAN  STM32_DMAMUX1_USART6_TX

// Mask of DMA streams which are shared
#define SHARED_DMA_MASK ((1U<<STM32_DMA_STREAM_ID(2,1))|(1U<<STM32_DMA_STREAM_ID(2,2))|(1U<<STM32_DMA_STREAM_ID(2,3))|(1U<<STM32_DMA_STREAM_ID(2,4)))


// generated UART DMA configuration lines
#define STM32_USART1_RX_DMA_CONFIG true, STM32_UART_USART1_RX_DMA_STREAM, STM32_DMAMUX1_USART1_RX
#define STM32_USART1_TX_DMA_CONFIG true, STM32_UART_USART1_TX_DMA_STREAM, STM32_DMAMUX1_USART1_TX
#define STM32_USART2_RX_DMA_CONFIG true, STM32_UART_USART2_RX_DMA_STREAM, STM32_DMAMUX1_USART2_RX
#define STM32_USART2_TX_DMA_CONFIG true, STM32_UART_USART2_TX_DMA_STREAM, STM32_DMAMUX1_USART2_TX
#define STM32_USART3_RX_DMA_CONFIG true, STM32_UART_USART3_RX_DMA_STREAM, STM32_DMAMUX1_USART3_RX
#define STM32_USART3_TX_DMA_CONFIG true, STM32_UART_USART3_TX_DMA_STREAM, STM32_DMAMUX1_USART3_TX
#define STM32_UART5_RX_DMA_CONFIG true, STM32_UART_UART5_RX_DMA_STREAM, STM32_DMAMUX1_UART5_RX
#define STM32_UART5_TX_DMA_CONFIG true, STM32_UART_UART5_TX_DMA_STREAM, STM32_DMAMUX1_UART5_TX
#define STM32_USART6_RX_DMA_CONFIG true, STM32_UART_USART6_RX_DMA_STREAM, STM32_DMAMUX1_USART6_RX
#define STM32_USART6_TX_DMA_CONFIG true, STM32_UART_USART6_TX_DMA_STREAM, STM32_DMAMUX1_USART6_TX
#define STM32_UART7_RX_DMA_CONFIG true, STM32_UART_UART7_RX_DMA_STREAM, STM32_DMAMUX1_UART7_RX
#define STM32_UART7_TX_DMA_CONFIG true, STM32_UART_UART7_TX_DMA_STREAM, STM32_DMAMUX1_UART7_TX
#define STM32_UART8_RX_DMA_CONFIG true, STM32_UART_UART8_RX_DMA_STREAM, STM32_DMAMUX1_UART8_RX
#define STM32_UART8_TX_DMA_CONFIG true, STM32_UART_UART8_TX_DMA_STREAM, STM32_DMAMUX1_UART8_TX


// generated SPI DMA configuration lines
#define STM32_SPI_SPI1_DMA_STREAMS STM32_SPI_SPI1_TX_DMA_STREAM, STM32_SPI_SPI1_RX_DMA_STREAM
#define STM32_SPI_SPI2_DMA_STREAMS STM32_SPI_SPI2_TX_DMA_STREAM, STM32_SPI_SPI2_RX_DMA_STREAM
#define HAL_PWM_COUNT 8

// Alarm PWM output config
#define STM32_PWM_USE_TIM3 TRUE
#define STM32_TIM3_SUPPRESS_ISR
#define HAL_PWM_ALARM \
        { /* pwmGroup */ \
          2,  /* Timer channel */ \
          { /* PWMConfig */ \
            1000000,    /* PWM clock frequency. */ \
            1000,    /* Initial PWM period 20ms. */ \
            NULL,  /* no callback */ \
            { /* Channel Config */ \
             {PWM_OUTPUT_DISABLED, NULL}, \
             {PWM_OUTPUT_DISABLED, NULL}, \
             {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
             {PWM_OUTPUT_DISABLED, NULL}  \
            }, \
            0, 0 \
          }, \
          &PWMD3 /* PWMDriver* */ \
        }

// PWM timer config
#define STM32_PWM_USE_TIM5 TRUE
#define STM32_TIM5_SUPPRESS_ISR
#define STM32_PWM_USE_TIM4 TRUE
#define STM32_TIM4_SUPPRESS_ISR
#define STM32_PWM_USE_TIM1 TRUE
#define STM32_TIM1_SUPPRESS_ISR

// PWM output config
#if defined(STM32_TIM_TIM5_UP_DMA_STREAM) && defined(STM32_TIM_TIM5_UP_DMA_CHAN)
# define HAL_PWM5_DMA_CONFIG true, STM32_TIM_TIM5_UP_DMA_STREAM, STM32_TIM_TIM5_UP_DMA_CHAN
#else
# define HAL_PWM5_DMA_CONFIG false, 0, 0
#endif
#if !defined(HAL_TIM5_UP_SHARED)
#define HAL_TIM5_UP_SHARED false
#endif
#define HAL_PWM_GROUP1 { false, \
        {6, 7, 255, 255}, \
        /* Group Initial Config */ \
        { \
          1000000,  /* PWM clock frequency. */ \
          20000,   /* Initial PWM period 20ms. */ \
          NULL,     /* no callback */ \
          { \
           /* Channel Config */ \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_DISABLED, NULL}, \
           {PWM_OUTPUT_DISABLED, NULL}  \
          }, 0, 0}, &PWMD5, 5, \
          HAL_PWM5_DMA_CONFIG, \
          { 2, 2, 0, 0 }, \
          { PAL_LINE(GPIOA,0U), PAL_LINE(GPIOA,1U), 0, 0 }}
#if defined(STM32_TIM_TIM4_UP_DMA_STREAM) && defined(STM32_TIM_TIM4_UP_DMA_CHAN)
# define HAL_PWM4_DMA_CONFIG true, STM32_TIM_TIM4_UP_DMA_STREAM, STM32_TIM_TIM4_UP_DMA_CHAN
#else
# define HAL_PWM4_DMA_CONFIG false, 0, 0
#endif
#if !defined(HAL_TIM4_UP_SHARED)
#define HAL_TIM4_UP_SHARED false
#endif
#define HAL_PWM_GROUP2 { false, \
        {255, 255, 4, 5}, \
        /* Group Initial Config */ \
        { \
          1000000,  /* PWM clock frequency. */ \
          20000,   /* Initial PWM period 20ms. */ \
          NULL,     /* no callback */ \
          { \
           /* Channel Config */ \
           {PWM_OUTPUT_DISABLED, NULL}, \
           {PWM_OUTPUT_DISABLED, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}  \
          }, 0, 0}, &PWMD4, 4, \
          HAL_PWM4_DMA_CONFIG, \
          { 0, 0, 2, 2 }, \
          { 0, 0, PAL_LINE(GPIOD,14U), PAL_LINE(GPIOD,15U) }}
#if defined(STM32_TIM_TIM1_UP_DMA_STREAM) && defined(STM32_TIM_TIM1_UP_DMA_CHAN)
# define HAL_PWM1_DMA_CONFIG true, STM32_TIM_TIM1_UP_DMA_STREAM, STM32_TIM_TIM1_UP_DMA_CHAN
#else
# define HAL_PWM1_DMA_CONFIG false, 0, 0
#endif
#if !defined(HAL_TIM1_UP_SHARED)
#define HAL_TIM1_UP_SHARED false
#endif
#define HAL_PWM_GROUP3 { true, \
        {0, 1, 2, 3}, \
        /* Group Initial Config */ \
        { \
          1000000,  /* PWM clock frequency. */ \
          20000,   /* Initial PWM period 20ms. */ \
          NULL,     /* no callback */ \
          { \
           /* Channel Config */ \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}  \
          }, 0, 0}, &PWMD1, 1, \
          HAL_PWM1_DMA_CONFIG, \
          { 1, 1, 1, 1 }, \
          { PAL_LINE(GPIOA,8U), PAL_LINE(GPIOE,11U), PAL_LINE(GPIOE,13U), PAL_LINE(GPIOE,14U) }}
#define HAL_PWM_GROUPS HAL_PWM_GROUP1,HAL_PWM_GROUP2,HAL_PWM_GROUP3

#define STM32_PWM_USE_ADVANCED TRUE
// I2C configuration

#if defined(STM32_I2C_I2C4_RX_DMA_STREAM) && defined(STM32_I2C_I2C4_TX_DMA_STREAM)
#define HAL_I2C4_CONFIG { &I2CD4, 4, STM32_I2C_I2C4_RX_DMA_STREAM, STM32_I2C_I2C4_TX_DMA_STREAM, PAL_LINE(GPIOD,12U), PAL_LINE(GPIOD,13U) }
#else
#define HAL_I2C4_CONFIG { &I2CD4, 4, SHARED_DMA_NONE, SHARED_DMA_NONE, PAL_LINE(GPIOD,12U), PAL_LINE(GPIOD,13U) }
#endif

#if defined(STM32_I2C_I2C1_RX_DMA_STREAM) && defined(STM32_I2C_I2C1_TX_DMA_STREAM)
#define HAL_I2C1_CONFIG { &I2CD1, 1, STM32_I2C_I2C1_RX_DMA_STREAM, STM32_I2C_I2C1_TX_DMA_STREAM, PAL_LINE(GPIOB,8U), PAL_LINE(GPIOB,7U) }
#else
#define HAL_I2C1_CONFIG { &I2CD1, 1, SHARED_DMA_NONE, SHARED_DMA_NONE, PAL_LINE(GPIOB,8U), PAL_LINE(GPIOB,7U) }
#endif

#if defined(STM32_I2C_I2C2_RX_DMA_STREAM) && defined(STM32_I2C_I2C2_TX_DMA_STREAM)
#define HAL_I2C2_CONFIG { &I2CD2, 2, STM32_I2C_I2C2_RX_DMA_STREAM, STM32_I2C_I2C2_TX_DMA_STREAM, PAL_LINE(GPIOB,10U), PAL_LINE(GPIOB,11U) }
#else
#define HAL_I2C2_CONFIG { &I2CD2, 2, SHARED_DMA_NONE, SHARED_DMA_NONE, PAL_LINE(GPIOB,10U), PAL_LINE(GPIOB,11U) }
#endif

#define HAL_I2C_DEVICE_LIST HAL_I2C4_CONFIG,HAL_I2C1_CONFIG,HAL_I2C2_CONFIG


// UART configuration
#define HAL_HAVE_SERIAL0 1
#define HAL_HAVE_SERIAL1 1
#define HAL_HAVE_SERIAL2 1
#define HAL_HAVE_SERIAL3 1
#define HAL_HAVE_SERIAL4 1
#define HAL_HAVE_SERIAL5 1
#define HAL_HAVE_SERIAL6 1
#define HAL_HAVE_SERIAL7 1
#define HAL_NUM_SERIAL_PORTS 8
#define HAL_SERIAL0_DRIVER ChibiOS::UARTDriver serial0Driver(0)
#define HAL_SERIAL1_DRIVER ChibiOS::UARTDriver serial1Driver(1)
#define HAL_SERIAL2_DRIVER ChibiOS::UARTDriver serial2Driver(2)
#define HAL_SERIAL3_DRIVER ChibiOS::UARTDriver serial3Driver(3)
#define HAL_SERIAL4_DRIVER ChibiOS::UARTDriver serial4Driver(4)
#define HAL_SERIAL5_DRIVER ChibiOS::UARTDriver serial5Driver(5)
#define HAL_SERIAL6_DRIVER ChibiOS::UARTDriver serial6Driver(6)
#define HAL_SERIAL7_DRIVER ChibiOS::UARTDriver serial7Driver(7)
#define HAL_SERIAL8_DRIVER Empty::UARTDriver serial8Driver
#define HAL_SERIAL9_DRIVER Empty::UARTDriver serial9Driver
#ifndef HAL_WITH_IO_MCU
#define HAL_WITH_IO_MCU 1
#endif
#define HAL_UART_IOMCU_IDX 8
#define HAL_UART_IO_DRIVER ChibiOS::UARTDriver uart_io(HAL_UART_IOMCU_IDX)
#define HAL_HAVE_SERVO_VOLTAGE 1
#define AP_FEATURE_SBUS_OUT 1

#define HAL_CRASH_SERIAL_PORT UART5
#define IRQ_DISABLE_HAL_CRASH_SERIAL_PORT() nvicDisableVector(STM32_UART5_NUMBER)
#define RCC_RESET_HAL_CRASH_SERIAL_PORT() rccResetUART5(); rccEnableUART5(true)
#define HAL_CRASH_SERIAL_PORT_CLOCK STM32_UART5CLK
#define HAL_OTG1_CONFIG {(BaseSequentialStream*) &SDU1, 1, true, false, 0, 0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, UINT8_MAX,}
#define HAL_HAVE_RTSCTS_SERIAL1
#define HAL_UART7_CONFIG { (BaseSequentialStream*) &SD7, 7, false, STM32_UART7_RX_DMA_CONFIG, STM32_UART7_TX_DMA_CONFIG, PAL_LINE(GPIOE,8U), PAL_LINE(GPIOE,7U), PAL_LINE(GPIOE,9U), PAL_LINE(GPIOE,10U), -1, 0, -1, 0, 0, 7}
#define HAL_HAVE_RTSCTS_SERIAL2
#define HAL_UART5_CONFIG { (BaseSequentialStream*) &SD5, 5, false, STM32_UART5_RX_DMA_CONFIG, STM32_UART5_TX_DMA_CONFIG, PAL_LINE(GPIOC,12U), PAL_LINE(GPIOD,2U), PAL_LINE(GPIOC,8U), PAL_LINE(GPIOC,9U), -1, 0, -1, 0, 0, 8}
#define HAL_USART1_CONFIG { (BaseSequentialStream*) &SD1, 1, false, STM32_USART1_RX_DMA_CONFIG, STM32_USART1_TX_DMA_CONFIG, PAL_LINE(GPIOB,6U), PAL_LINE(GPIOA,10U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define HAL_UART8_CONFIG { (BaseSequentialStream*) &SD8, 8, false, STM32_UART8_RX_DMA_CONFIG, STM32_UART8_TX_DMA_CONFIG, PAL_LINE(GPIOE,1U), PAL_LINE(GPIOE,0U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define HAL_USART2_CONFIG { (BaseSequentialStream*) &SD2, 2, false, STM32_USART2_RX_DMA_CONFIG, STM32_USART2_TX_DMA_CONFIG, PAL_LINE(GPIOD,5U), PAL_LINE(GPIOA,3U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define HAL_USART3_CONFIG { (BaseSequentialStream*) &SD3, 3, false, STM32_USART3_RX_DMA_CONFIG, STM32_USART3_TX_DMA_CONFIG, PAL_LINE(GPIOD,8U), PAL_LINE(GPIOD,9U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define HAL_OTG2_CONFIG {(BaseSequentialStream*) &SDU2, 2, true, false, 0, 0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, UINT8_MAX,}
#define HAL_USART6_CONFIG { (BaseSequentialStream*) &SD6, 6, false, STM32_USART6_RX_DMA_CONFIG, STM32_USART6_TX_DMA_CONFIG, PAL_LINE(GPIOC,6U), PAL_LINE(GPIOC,7U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define AP_FEATURE_RTSCTS 1
#define HAL_OTG2_UART_INDEX 7
#define HAL_HAVE_DUAL_USB_CDC 1

#if defined(HAL_NUM_CAN_IFACES) && HAL_NUM_CAN_IFACES
#ifndef HAL_OTG2_PROTOCOL
#define HAL_OTG2_PROTOCOL SerialProtocol_MAVLink2
#endif
#define DEFAULT_SERIAL7_PROTOCOL HAL_OTG2_PROTOCOL
#define DEFAULT_SERIAL7_BAUD 115200
#endif
#define HAL_SERIAL_DEVICE_LIST HAL_OTG1_CONFIG,HAL_UART7_CONFIG,HAL_UART5_CONFIG,HAL_USART1_CONFIG,HAL_UART8_CONFIG,HAL_USART2_CONFIG,HAL_USART3_CONFIG,HAL_OTG2_CONFIG,HAL_USART6_CONFIG

#define HAL_UART_NUM_SERIAL_PORTS 8
// USB configuration
#define HAL_USB_VENDOR_ID 0x3162
#define HAL_USB_PRODUCT_ID 0x0053
#define HAL_USB_STRING_MANUFACTURER "Holybro"
#define HAL_USB_STRING_PRODUCT "%BOARD%"
#define HAL_USB_STRING_SERIAL "%SERIAL%"


#define AP_BOOTLOADER_FLASHING_ENABLED 1

/*
* I/O ports initial setup, this configuration is established soon after reset
* in the initialization code.
* Please refer to the STM32 Reference Manual for details.
*/
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/* PORTA:
 PA0 TIM5_CH1 TIM5 AF2 PWM7
 PA1 TIM5_CH2 TIM5 AF2 PWM8
 PA2 BATT2_CURRENT_SENS ADC1 ADC1_IN14
 PA3 USART2_RX USART2 AF7
 PA4 VDD_5V_SENS ADC1 ADC1_IN18
 PA5 SPI1_SCK SPI1 AF5
 PA6 SPI1_MISO SPI1 AF5
 PA7 SPI1_MOSI SPI1 AF5
 PA8 TIM1_CH1 TIM1 AF1 PWM1
 PA9 VBUS INPUT
 PA10 USART1_RX USART1 AF7
 PA11 OTG_FS_DM OTG1 AF10
 PA12 OTG_FS_DP OTG1 AF10
 PA13 JTMS-SWDIO SWD AF0
 PA14 JTCK-SWCLK SWD AF0
 PA15 VDD_BRICK_nVALID INPUT
*/

#define VAL_GPIOA_MODER   (PIN_MODE_ALTERNATE(0U) | \
                           PIN_MODE_ALTERNATE(1U) | \
                           PIN_MODE_ANALOG(2U) | \
                           PIN_MODE_ALTERNATE(3U) | \
                           PIN_MODE_ANALOG(4U) | \
                           PIN_MODE_ALTERNATE(5U) | \
                           PIN_MODE_ALTERNATE(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_ALTERNATE(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_ALTERNATE(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOA_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_OPENDRAIN(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOA_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOA_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_PULLUP(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_PULLUP(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_PULLUP(13U) | \
                           PIN_PUPDR_PULLDOWN(14U) | \
                           PIN_PUPDR_PULLUP(15U))

#define VAL_GPIOA_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOA_AFRL    (PIN_AFIO_AF(0U, 2U) | \
                           PIN_AFIO_AF(1U, 2U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 7U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 5U) | \
                           PIN_AFIO_AF(6U, 5U) | \
                           PIN_AFIO_AF(7U, 5U))

#define VAL_GPIOA_AFRH    (PIN_AFIO_AF(8U, 1U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 7U) | \
                           PIN_AFIO_AF(11U, 10U) | \
                           PIN_AFIO_AF(12U, 10U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTB:
 PB0 TIM3_CH3 TIM3 AF2
 PB1 BATT2_VOLTAGE_SENS ADC1 ADC1_IN5
 PB2 VDD_3V3_SENSORS1_EN OUTPUT
 PB3 SDMMC2_D2 SDMMC2 AF9
 PB4 SDMMC2_D3 SDMMC2 AF9
 PB5 CAN2_RX CAN2 AF9
 PB6 USART1_TX USART1 AF7
 PB7 I2C1_SDA I2C1 AF4
 PB8 I2C1_SCL I2C1 AF4
 PB9 HEATER_EN OUTPUT
 PB10 I2C2_SCL I2C2 AF4
 PB11 I2C2_SDA I2C2 AF4
 PB12 VDD_BRICK2_nVALID INPUT
 PB13 CAN2_TX CAN2 AF9
 PB14 SDMMC2_D0 SDMMC2 AF9
 PB15 SDMMC2_D1 SDMMC2 AF9
*/

#define VAL_GPIOB_MODER   (PIN_MODE_ALTERNATE(0U) | \
                           PIN_MODE_ANALOG(1U) | \
                           PIN_MODE_OUTPUT(2U) | \
                           PIN_MODE_ALTERNATE(3U) | \
                           PIN_MODE_ALTERNATE(4U) | \
                           PIN_MODE_ALTERNATE(5U) | \
                           PIN_MODE_ALTERNATE(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_OUTPUT(9U) | \
                           PIN_MODE_ALTERNATE(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_ALTERNATE(15U))

#define VAL_GPIOB_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_OPENDRAIN(7U) | \
                           PIN_OTYPE_OPENDRAIN(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_OPENDRAIN(10U) | \
                           PIN_OTYPE_OPENDRAIN(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOB_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOB_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_PULLUP(3U) | \
                           PIN_PUPDR_PULLUP(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_PULLUP(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_PULLUP(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_PULLUP(14U) | \
                           PIN_PUPDR_PULLUP(15U))

#define VAL_GPIOB_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_LOW(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOB_AFRL    (PIN_AFIO_AF(0U, 2U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 9U) | \
                           PIN_AFIO_AF(4U, 9U) | \
                           PIN_AFIO_AF(5U, 9U) | \
                           PIN_AFIO_AF(6U, 7U) | \
                           PIN_AFIO_AF(7U, 4U))

#define VAL_GPIOB_AFRH    (PIN_AFIO_AF(8U, 4U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 4U) | \
                           PIN_AFIO_AF(11U, 4U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 9U) | \
                           PIN_AFIO_AF(14U, 9U) | \
                           PIN_AFIO_AF(15U, 9U))

/* PORTC:
 PC2 SPI2_MISO SPI2 AF5
 PC3 SPI2_MOSI SPI2 AF5
 PC4 BATT_CURRENT_SENS ADC1 ADC1_IN4
 PC5 BATT_VOLTAGE_SENS ADC1 ADC1_IN8
 PC6 USART6_TX USART6 AF7
 PC7 USART6_RX USART6 AF7
 PC8 UART5_RTS UART5
 PC9 UART5_CTS UART5 AF8
 PC10 nVDD_5V_HIPOWER_EN OUTPUT
 PC11 VDD_5V_HIPOWER_nOC INPUT
 PC12 UART5_TX UART5 AF8
 PC13 ICM42688_CS CS
 PC14 BMI055_G_CS CS
 PC15 BMI055_A_CS CS
*/

#define VAL_GPIOC_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_ALTERNATE(2U) | \
                           PIN_MODE_ALTERNATE(3U) | \
                           PIN_MODE_ANALOG(4U) | \
                           PIN_MODE_ANALOG(5U) | \
                           PIN_MODE_ALTERNATE(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_OUTPUT(8U) | \
                           PIN_MODE_ALTERNATE(9U) | \
                           PIN_MODE_OUTPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_ALTERNATE(12U) | \
                           PIN_MODE_OUTPUT(13U) | \
                           PIN_MODE_OUTPUT(14U) | \
                           PIN_MODE_OUTPUT(15U))

#define VAL_GPIOC_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOC_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOC_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_PULLUP(6U) | \
                           PIN_PUPDR_PULLUP(7U) | \
                           PIN_PUPDR_PULLDOWN(8U) | \
                           PIN_PUPDR_PULLUP(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_PULLUP(11U) | \
                           PIN_PUPDR_PULLUP(12U) | \
                           PIN_PUPDR_PULLUP(13U) | \
                           PIN_PUPDR_PULLUP(14U) | \
                           PIN_PUPDR_PULLUP(15U))

#define VAL_GPIOC_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_LOW(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_LOW(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOC_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 5U) | \
                           PIN_AFIO_AF(3U, 5U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 7U) | \
                           PIN_AFIO_AF(7U, 7U))

#define VAL_GPIOC_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 8U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 8U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTD:
 PD0 CAN1_RX CAN1 AF9
 PD1 CAN1_TX CAN1 AF9
 PD2 UART5_RX UART5 AF8
 PD3 SPI2_SCK SPI2 AF5
 PD4 FRAM_CS CS
 PD5 USART2_TX USART2 AF7
 PD6 SDMMC2_CK SDMMC2 AF11
 PD7 SDMMC2_CMD SDMMC2 AF11
 PD8 USART3_TX USART3 AF7
 PD9 USART3_RX USART3 AF7
 PD10 LED_RED OUTPUT
 PD11 LED_BLUE OUTPUT
 PD12 I2C4_SCL I2C4 AF4
 PD13 I2C4_SDA I2C4 AF4
 PD14 TIM4_CH3 TIM4 AF2 PWM5
 PD15 TIM4_CH4 TIM4 AF2 PWM6
*/

#define VAL_GPIOD_MODER   (PIN_MODE_ALTERNATE(0U) | \
                           PIN_MODE_ALTERNATE(1U) | \
                           PIN_MODE_ALTERNATE(2U) | \
                           PIN_MODE_ALTERNATE(3U) | \
                           PIN_MODE_OUTPUT(4U) | \
                           PIN_MODE_ALTERNATE(5U) | \
                           PIN_MODE_ALTERNATE(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_ALTERNATE(9U) | \
                           PIN_MODE_OUTPUT(10U) | \
                           PIN_MODE_OUTPUT(11U) | \
                           PIN_MODE_ALTERNATE(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_ALTERNATE(15U))

#define VAL_GPIOD_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_OPENDRAIN(10U) | \
                           PIN_OTYPE_OPENDRAIN(11U) | \
                           PIN_OTYPE_OPENDRAIN(12U) | \
                           PIN_OTYPE_OPENDRAIN(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOD_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOD_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_PULLUP(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_PULLUP(4U) | \
                           PIN_PUPDR_PULLUP(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_PULLUP(7U) | \
                           PIN_PUPDR_PULLUP(8U) | \
                           PIN_PUPDR_PULLUP(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOD_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOD_AFRL    (PIN_AFIO_AF(0U, 9U) | \
                           PIN_AFIO_AF(1U, 9U) | \
                           PIN_AFIO_AF(2U, 8U) | \
                           PIN_AFIO_AF(3U, 5U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 7U) | \
                           PIN_AFIO_AF(6U, 11U) | \
                           PIN_AFIO_AF(7U, 11U))

#define VAL_GPIOD_AFRH    (PIN_AFIO_AF(8U, 7U) | \
                           PIN_AFIO_AF(9U, 7U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 4U) | \
                           PIN_AFIO_AF(13U, 4U) | \
                           PIN_AFIO_AF(14U, 2U) | \
                           PIN_AFIO_AF(15U, 2U))

/* PORTE:
 PE0 UART8_RX UART8 AF8
 PE1 UART8_TX UART8 AF8
 PE2 nVDD_5V_PERIPH_EN OUTPUT
 PE3 VDD_5V_PERIPH_nOC INPUT
 PE4 BMI055_DRDY_A INPUT
 PE5 BMI055_DRDY_G INPUT
 PE6 ICM42688_DRDY INPUT
 PE7 UART7_RX UART7 AF7
 PE8 UART7_TX UART7 AF7
 PE9 UART7_RTS UART7
 PE10 UART7_CTS UART7 AF7
 PE11 TIM1_CH2 TIM1 AF1 PWM2
 PE12 HW_VER_REV_DRIVE OUTPUT
 PE13 TIM1_CH3 TIM1 AF1 PWM3
 PE14 TIM1_CH4 TIM1 AF1 PWM4
*/

#define VAL_GPIOE_MODER   (PIN_MODE_ALTERNATE(0U) | \
                           PIN_MODE_ALTERNATE(1U) | \
                           PIN_MODE_OUTPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_OUTPUT(9U) | \
                           PIN_MODE_ALTERNATE(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_OUTPUT(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOE_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOE_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOE_PUPDR   (PIN_PUPDR_PULLUP(0U) | \
                           PIN_PUPDR_PULLUP(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_PULLUP(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_PULLUP(7U) | \
                           PIN_PUPDR_PULLUP(8U) | \
                           PIN_PUPDR_PULLDOWN(9U) | \
                           PIN_PUPDR_PULLUP(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOE_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_LOW(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_LOW(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_LOW(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOE_AFRL    (PIN_AFIO_AF(0U, 8U) | \
                           PIN_AFIO_AF(1U, 8U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 7U))

#define VAL_GPIOE_AFRH    (PIN_AFIO_AF(8U, 7U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 7U) | \
                           PIN_AFIO_AF(11U, 1U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 1U) | \
                           PIN_AFIO_AF(14U, 1U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTF:
*/

#define VAL_GPIOF_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOF_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOF_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOF_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOF_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOF_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOF_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTG:
*/

#define VAL_GPIOG_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOG_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOG_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOG_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOG_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOG_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOG_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTH:
 PH2 SPEKTRUM_PWR OUTPUT
*/

#define VAL_GPIOH_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_OUTPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOH_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOH_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOH_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOH_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOH_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOH_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTI:
*/

#define VAL_GPIOI_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOI_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOI_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOI_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOI_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOI_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOI_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTJ:
*/

#define VAL_GPIOJ_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOJ_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOJ_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOJ_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOJ_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOJ_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOJ_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTK:
*/

#define VAL_GPIOK_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOK_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOK_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOK_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOK_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOK_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOK_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))


// normal defaults

// this file is inserted (by chibios_hwdef.py) into hwdef.h when
// configuring for "normal" builds - typically vehicle binaries but
// also examples.

#ifndef HAL_DSHOT_ALARM_ENABLED
#define HAL_DSHOT_ALARM_ENABLED (HAL_PWM_COUNT>0)
#endif

#ifndef HAL_BOARD_LOG_DIRECTORY
#define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
#endif

// a similar define is present in AP_HAL_Boards.h:
#ifndef HAL_OS_FATFS_IO
#define HAL_OS_FATFS_IO 0
#endif

#ifndef AP_TERRAIN_AVAILABLE
// enable terrain only if there's an SD card available:
#define AP_TERRAIN_AVAILABLE HAL_OS_FATFS_IO
#endif

#if AP_TERRAIN_AVAILABLE
#ifndef HAL_BOARD_TERRAIN_DIRECTORY
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
#endif
#endif  // AP_TERRAIN_AVAILABLE


// end normal defaults
