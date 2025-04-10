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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "io.h"
#include "system.h"

#include "bus_i2c.h"
#include "nvic.h"
#include "io_impl.h"
#include "rcc.h"

#ifndef SOFT_I2C

#define CLOCKSPEED 800000    // i2c clockspeed 400kHz default (conform specs), 800kHz  and  1200kHz (Betaflight default)

static void i2c_er_handler(I2CDevice device);
static void i2c_ev_handler(I2CDevice device);
static void i2cUnstick(IO_t scl, IO_t sda);

#define GPIO_AF_I2C GPIO_AF_I2C1

#ifdef STM32F4

#if defined(USE_I2C_PULLUP)
#define IOCFG_I2C IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_OD, GPIO_PuPd_UP)
#else
#define IOCFG_I2C IOCFG_AF_OD
#endif

#ifndef I2C1_SCL
#define I2C1_SCL PB8
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB9
#endif

#else

#ifndef I2C1_SCL
#define I2C1_SCL PB8
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB9
#endif
#define IOCFG_I2C   IO_CONFIG(GPIO_Mode_AF_OD, GPIO_Speed_50MHz)

#endif

#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif

#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif

#ifdef STM32F4
#ifndef I2C3_SCL
#define I2C3_SCL PA8
#endif
#ifndef I2C3_SDA
#define I2C3_SDA PB4
#endif
#endif

static i2cDevice_t i2cHardwareMap[] = {
    { .dev = I2C1, .scl = IO_TAG(I2C1_SCL), .sda = IO_TAG(I2C1_SDA), .rcc = RCC_APB1(I2C1), .overClock = I2C1_OVERCLOCK, .ev_irq = I2C1_EV_IRQn, .er_irq = I2C1_ER_IRQn },
    { .dev = I2C2, .scl = IO_TAG(I2C2_SCL), .sda = IO_TAG(I2C2_SDA), .rcc = RCC_APB1(I2C2), .overClock = I2C2_OVERCLOCK, .ev_irq = I2C2_EV_IRQn, .er_irq = I2C2_ER_IRQn },
#ifdef STM32F4
    { .dev = I2C3, .scl = IO_TAG(I2C3_SCL), .sda = IO_TAG(I2C3_SDA), .rcc = RCC_APB1(I2C3), .overClock = I2C2_OVERCLOCK, .ev_irq = I2C3_EV_IRQn, .er_irq = I2C3_ER_IRQn }
#endif
};

static volatile uint16_t i2cErrorCount = 0;

static i2cState_t i2cState[] = {
    { false, false, false, 0, 0, 0, 0, 0, 0, 0, 0 },
    { false, false, false, 0, 0, 0, 0, 0, 0, 0, 0 },
    { false, false, false, 0, 0, 0, 0, 0, 0, 0, 0 }
};

void i2cSetOverclock(uint8_t overClock)
{
    for (unsigned int i = 0; i < sizeof(i2cHardwareMap) / sizeof(i2cHardwareMap[0]); i++) {
        i2cHardwareMap[i].overClock = overClock;
    }
}

void I2C1_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_1);
}

void I2C1_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_1);
}

void I2C2_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_2);
}

void I2C2_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_2);
}

#ifdef STM32F4
void I2C3_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_3);
}

void I2C3_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_3);
}
#endif

static bool i2cHandleHardwareFailure(I2CDevice device)
{
    const i2cState_t *state = &(i2cState[device]);

    i2cErrorCount++;

    // reinit peripheral + clock out garbage
    if (state->busError) {
        i2cInit(device);
    }

    return false;
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{

    if (device == I2CINVALID)
        return false;

    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    I2C_TypeDef *I2Cx;
    I2Cx = i2cHardwareMap[device].dev;

    i2cState_t *state;
    state = &(i2cState[device]);

    if (!state->initialised)
        return false;

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 1;
    state->reading = 0;
    state->write_p = data;
    state->read_p = data;
    state->bytes = len_;
    state->busy = 1;
    state->error = false;
    state->busError = false;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CR1 & I2C_CR1_START)) {                             // ensure sending a start
            while (I2Cx->CR1 & I2C_CR1_STOP && --timeout > 0) {; }     // wait for any stop to finish sending
            if (state->error || timeout == 0)
                return i2cHandleHardwareFailure(device);
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    timeout = I2C_DEFAULT_TIMEOUT;
    while (state->busy && --timeout > 0) {; }
    if (timeout == 0)
        return i2cHandleHardwareFailure(device);

    return !state->error;
}

bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data);
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID)
        return false;

    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    I2C_TypeDef *I2Cx;
    I2Cx = i2cHardwareMap[device].dev;

    i2cState_t *state;
    state = &(i2cState[device]);

    if (!state->initialised)
        return false;

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 0;
    state->reading = 1;
    state->read_p = buf;
    state->write_p = buf;
    state->bytes = len;
    state->busy = 1;
    state->error = false;
    state->busError = false;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CR1 & I2C_CR1_START)) {                             // ensure sending a start
            while (I2Cx->CR1 & I2C_CR1_STOP && --timeout > 0) {; }     // wait for any stop to finish sending
            if (state->error || timeout == 0)
                return i2cHandleHardwareFailure(device);
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    timeout = I2C_DEFAULT_TIMEOUT;
    while (state->busy && --timeout > 0) {; }
    if (state->error || timeout == 0)
        return i2cHandleHardwareFailure(device);

    return true;
}

static void i2c_er_handler(I2CDevice device) {

    I2C_TypeDef *I2Cx;
    I2Cx = i2cHardwareMap[device].dev;

    i2cState_t *state;
    state = &(i2cState[device]);

    // Read the I2C1 status register
    volatile uint32_t SR1Register = I2Cx->SR1;

    if (SR1Register & 0x0F00) {                                         // an error
        state->error = true;

        // Only re-initialise bus if bus error indicated, don't reset bus when ARLO or AF
        if (SR1Register & I2C_SR1_BERR)
            state->busError = true;
    }

    // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
    if (SR1Register & 0x0700) {
        (void)I2Cx->SR2;                                                        // read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                                // disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
        if (!(SR1Register & I2C_SR1_ARLO) && !(I2Cx->CR1 & I2C_CR1_STOP)) {     // if we dont have an ARLO error, ensure sending of a stop
            if (I2Cx->CR1 & I2C_CR1_START) {                                    // We are currently trying to send a start, this is very bad as start, stop will hang the peripheral
                while (I2Cx->CR1 & I2C_CR1_START) {; }                         // wait for any start to finish sending
                I2C_GenerateSTOP(I2Cx, ENABLE);                                 // send stop to finalise bus transaction
                while (I2Cx->CR1 & I2C_CR1_STOP) {; }                          // wait for stop to finish sending
                i2cInit(device);                                                // reset and configure the hardware
            }
            else {
                I2C_GenerateSTOP(I2Cx, ENABLE);                                 // stop to free up the bus
                I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);           // Disable EVT and ERR interrupts while bus inactive
            }
        }
    }
    I2Cx->SR1 &= ~0x0F00;                                                       // reset all the error bits to clear the interrupt
    state->busy = 0;
}

void i2c_ev_handler(I2CDevice device) {

    I2C_TypeDef *I2Cx;
    I2Cx = i2cHardwareMap[device].dev;

    i2cState_t *state;
    state = &(i2cState[device]);

    static uint8_t subaddress_sent, final_stop;                                 // flag to indicate if subaddess sent, flag to indicate final bus condition
    static int8_t index;                                                        // index is signed -1 == send the subaddress
    uint8_t SReg_1 = I2Cx->SR1;                                                 // read the status register here

    if (SReg_1 & I2C_SR1_SB) {                                                  // we just sent a start - EV5 in ref manual
        I2Cx->CR1 &= ~I2C_CR1_POS;                                              // reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(I2Cx, ENABLE);                                    // make sure ACK is on
        index = 0;                                                              // reset the index
        if (state->reading && (subaddress_sent || 0xFF == state->reg)) {          // we have sent the subaddr
            subaddress_sent = 1;                                                // make sure this is set in case of no subaddress, so following code runs correctly
            if (state->bytes == 2)
                I2Cx->CR1 |= I2C_CR1_POS;                                       // set the POS bit so NACK applied to the final byte in the two byte read
            I2C_Send7bitAddress(I2Cx, state->addr, I2C_Direction_Receiver);      // send the address and set hardware mode
        }
        else {                                                                // direction is Tx, or we havent sent the sub and rep start
            I2C_Send7bitAddress(I2Cx, state->addr, I2C_Direction_Transmitter);   // send the address and set hardware mode
            if (state->reg != 0xFF)                                              // 0xFF as subaddress means it will be ignored, in Tx or Rx mode
                index = -1;                                                     // send a subaddress
        }
    }
    else if (SReg_1 & I2C_SR1_ADDR) {                                         // we just sent the address - EV6 in ref manual
        // Read SR1,2 to clear ADDR
        __DMB();                                                                // memory fence to control hardware
        if (state->bytes == 1 && state->reading && subaddress_sent) {             // we are receiving 1 byte - EV6_3
            I2C_AcknowledgeConfig(I2Cx, DISABLE);                               // turn off ACK
            __DMB();
            (void)I2Cx->SR2;                                                    // clear ADDR after ACK is turned off
            I2C_GenerateSTOP(I2Cx, ENABLE);                                     // program the stop
            final_stop = 1;
            I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                     // allow us to have an EV7
        }
        else {                                                        // EV6 and EV6_1
            (void)I2Cx->SR2;                                            // clear the ADDR here
            __DMB();
            if (state->bytes == 2 && state->reading && subaddress_sent) {         // rx 2 bytes - EV6_1
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                           // turn off ACK
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                        // disable TXE to allow the buffer to fill
            }
            else if (state->bytes == 3 && state->reading && subaddress_sent)    // rx 3 bytes
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                        // make sure RXNE disabled so we get a BTF in two bytes time
            else                                                                // receiving greater than three bytes, sending subaddress, or transmitting
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
        }
    }
    else if (SReg_1 & I2C_SR1_BTF) {                                  // Byte transfer finished - EV7_2, EV7_3 or EV8_2
        final_stop = 1;
        if (state->reading && subaddress_sent) {                         // EV7_2, EV7_3
            if (state->bytes > 2) {                                      // EV7_2
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   // turn off ACK
                state->read_p[index++] = (uint8_t)I2Cx->DR;              // read data N-2
                I2C_GenerateSTOP(I2Cx, ENABLE);                         // program the Stop
                final_stop = 1;                                         // required to fix hardware
                state->read_p[index++] = (uint8_t)I2Cx->DR;              // read data N - 1
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                 // enable TXE to allow the final EV7
            }
            else {                                                    // EV7_3
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                state->read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N - 1
                state->read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N
                index++;                                                // to show job completed
            }
        }
        else {                                                        // EV8_2, which may be due to a subaddress sent or a write completion
            if (subaddress_sent || (state->writing)) {
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                index++;                                                // to show that the job is complete
            }
            else {                                                    // We need to send a subaddress
                I2C_GenerateSTART(I2Cx, ENABLE);                        // program the repeated Start
                subaddress_sent = 1;                                    // this is set back to zero upon completion of the current task
            }
        }
        // we must wait for the start to clear, otherwise we get constant BTF
        while (I2Cx->CR1 & 0x0100) {; }
    }
    else if (SReg_1 & I2C_SR1_RXNE) {                                 // Byte received - EV7
        state->read_p[index++] = (uint8_t)I2Cx->DR;
        if (state->bytes == (index + 3))
            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                    // disable TXE to allow the buffer to flush so we can get an EV7_2
        if (state->bytes == index)                                             // We have completed a final EV7
            index++;                                                    // to show job is complete
    }
    else if (SReg_1 & I2C_SR1_TXE) {                                  // Byte transmitted EV8 / EV8_1
        if (index != -1) {                                              // we dont have a subaddress to send
            I2Cx->DR = state->write_p[index++];
            if (state->bytes == index)                                         // we have sent all the data
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        }
        else {
            index++;
            I2Cx->DR = state->reg;                                             // send the subaddress
            if (state->reading || !(state->bytes))                                      // if receiving or sending 0 bytes, flush now
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        }
    }
    if (index == state->bytes + 1) {                                           // we have completed the current job
        subaddress_sent = 0;                                            // reset this here
        if (final_stop)                                                 // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);       // Disable EVT and ERR interrupts while bus inactive
        state->busy = 0;
    }
}

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID)
        return;

    i2cDevice_t *i2c;
    i2c = &(i2cHardwareMap[device]);

    i2cState_t *state;
    state = &(i2cState[device]);

    NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2cInit;

    IO_t scl = IOGetByTag(i2c->scl);
    IO_t sda = IOGetByTag(i2c->sda);

    IOInit(scl, OWNER_I2C, RESOURCE_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C, RESOURCE_I2C_SDA, RESOURCE_INDEX(device));

    // Enable RCC
    RCC_ClockCmd(i2c->rcc, ENABLE);

    I2C_ITConfig(i2c->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    i2cUnstick(scl, sda);

    // Init pins
#ifdef STM32F4
    IOConfigGPIOAF(scl, IOCFG_I2C, GPIO_AF_I2C);
    IOConfigGPIOAF(sda, IOCFG_I2C, GPIO_AF_I2C);
#else
    IOConfigGPIO(scl, IOCFG_I2C);
    IOConfigGPIO(sda, IOCFG_I2C);

    gpioPinRemapConfig(AFIO_MAPR_I2C1_REMAP, true);
#endif

    I2C_DeInit(i2c->dev);
    I2C_StructInit(&i2cInit);

    I2C_ITConfig(i2c->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);               // Disable EVT and ERR interrupts - they are enabled by the first request
    i2cInit.I2C_Mode = I2C_Mode_I2C;
    i2cInit.I2C_DutyCycle = I2C_DutyCycle_2;
    i2cInit.I2C_OwnAddress1 = 0;
    i2cInit.I2C_Ack = I2C_Ack_Enable;
    i2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    if (i2c->overClock) {
        i2cInit.I2C_ClockSpeed = 800000; // 800khz Maximum speed tested on various boards without issues
    } else {
        i2cInit.I2C_ClockSpeed = 400000; // 400khz Operation according specs
    }

    I2C_Cmd(i2c->dev, ENABLE);
    I2C_Init(i2c->dev, &i2cInit);

    I2C_StretchClockCmd(i2c->dev, ENABLE);


    // I2C ER Interrupt
    nvic.NVIC_IRQChannel = i2c->er_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // I2C EV Interrupt
    nvic.NVIC_IRQChannel = i2c->ev_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV);
    NVIC_Init(&nvic);
    
    state->initialised = true;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

static void i2cUnstick(IO_t scl, IO_t sda)
{
    int i;

    IOHi(scl);
    IOHi(sda);

    IOConfigGPIO(scl, IOCFG_OUT_OD);
    IOConfigGPIO(sda, IOCFG_OUT_OD);

    // Analog Devices AN-686
    // We need 9 clock pulses + STOP condition
    for (i = 0; i < 9; i++) {
        // Wait for any clock stretching to finish
        int timeout = 100;
        while (!IORead(scl) && timeout) {
            delayMicroseconds(5);
            timeout--;
        }

        // Pull low
        IOLo(scl); // Set bus low
        delayMicroseconds(5);
        IOHi(scl); // Set bus high
        delayMicroseconds(5);
    }

    // Generate a stop condition in case there was none
    IOLo(scl);
    delayMicroseconds(5);
    IOLo(sda);
    delayMicroseconds(5);

    IOHi(scl); // Set bus scl high
    delayMicroseconds(5);
    IOHi(sda); // Set bus sda high
}

#endif
