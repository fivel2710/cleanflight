/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "AFNA" // AFroNAze - NAZE might be considered misleading on Naze clones like the flip32.
#define USE_HARDWARE_REVISION_DETECTION

#define LED0                    PB13
#define LED1                    PB4

#undef BEEPER

#define INVERTER                PB2 // PB2 (BOOT1) abused as inverter select GPIO
#define INVERTER_USART          USART2

#define USE_EXTI
#define MAG_INT_EXTI            PC14
#define USE_MPU_DATA_READY_SIGNAL
//#define USE_MAG_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT
//#define DEBUG_MAG_DATA_READY_INTERRUPT

// We either have this 16mbit flash chip on SPI or the MPU6500 acc/gyro depending on board revision:
#define M25P16_CS_GPIO          NAZE_SPI_CS_GPIO
#define M25P16_CS_PIN           NAZE_SPI_CS_PIN
#define M25P16_SPI_INSTANCE     NAZE_SPI_INSTANCE

#define MPU6500_CS_GPIO_CLK_PERIPHERAL  NAZE_CS_GPIO_CLK_PERIPHERAL
#define MPU6500_CS_GPIO                 NAZE_SPI_CS_GPIO
#define MPU6500_CS_PIN                  NAZE_SPI_CS_PIN
#define MPU6500_SPI_INSTANCE            NAZE_SPI_INSTANCE

//#define USE_FLASHFS
//#define USE_FLASH_M25P16

#define GYRO
//#define USE_GYRO_MPU3050
#define USE_GYRO_MPU6050
//#define USE_GYRO_MPU6500
//#define USE_GYRO_SPI_MPU6500

//#define GYRO_MPU3050_ALIGN      CW0_DEG
//#define GYRO_MPU6050_ALIGN      CW0_DEG
//#define GYRO_MPU6500_ALIGN      CW0_DEG

#define ACC
#define USE_ACC_MPU6050

//#define ACC_ADXL345_ALIGN       CW270_DEG
//#define ACC_MPU6050_ALIGN       CW0_DEG
//#define ACC_MMA8452_ALIGN       CW90_DEG
//#define ACC_BMA280_ALIGN        CW0_DEG
//#define ACC_MPU6500_ALIGN       CW0_DEG

#define BARO
#define USE_BARO_BMP280

//#define MAG


#define USE_UART1
#define USE_UART2
#define SERIAL_PORT_COUNT       2

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)

#define USE_SPI
#define USE_SPI_DEVICE_1

#define USE_RX_NRF24
#ifdef USE_RX_NRF24

#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI1

//#define USE_RX_CX10
//#define USE_RX_H8_3D
//#define USE_RX_INAV
//#define NRF24_DEFAULT_PROTOCOL  NRF24RX_H8_3D

// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#define RX_CE_GPIO_CLK_PERIPHERAL    RCC_APB2Periph_GPIOB
#define RX_NSS_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOB
//#define RX_IRQ_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA
#define RX_CE_PIN                   	PB0
#define RX_NSS_PIN                  	PB1
#define RX_SCK_PIN              	PA5
#define RX_MISO_PIN             	PA6
#define RX_MOSI_PIN             	PA7
//#define RX_IRQ_PIN              	PA8
#define SPI1_NSS_PIN            RX_NSS_PIN
#define SPI1_SCK_PIN            RX_SCK_PIN
#define SPI1_MISO_PIN           RX_MISO_PIN
#define SPI1_MOSI_PIN           RX_MOSI_PIN

//#define USE_RX_CX10
//#define USE_RX_H8_3D
#define USE_RX_INAV
//#define USE_RX_SYMA
//#define USE_RX_V202

//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_SYMA_X5
//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_SYMA_X5C
#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_INAV
//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_H8_3D
//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_CX10
//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_V202_1M

#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI
//#define TELEMETRY
//#define TELEMETRY_LTM
//#define TELEMETRY_NRF24_LTM
#define SKIP_RX_PWM_PPM
#undef SERIAL_RX
//#undef SKIP_TASK_STATISTICS

#endif // USE_NRF24

#undef BRUSHED_MOTORS
#define DEFAULT_FEATURES        FEATURE_MOTOR_STOP

//#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define USE_QUAD_MIXER_ONLY
#undef USE_SERVOS

//#define SKIP_CLI_RESOURCES

//#undef TELEMETRY_HOTT
//#undef TELEMETRY_SMARTPORT
//#undef TELEMETRY_IBUS

// Disable all GPS protocols except UBLOX
#undef GPS_PROTO_NMEA
#undef GPS_PROTO_I2C_NAV
#undef GPS_PROTO_NAZA

#define STACK_CHECK
#define DEBUG_STACK

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    4

// IO - assuming all IOs on 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(13) | BIT(14) | BIT(15) )

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
