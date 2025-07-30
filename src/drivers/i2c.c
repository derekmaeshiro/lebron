#include "i2c.h"
#include "../common/defines.h"
#include "../common/assert_handler.h"
#include "../common/trace.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stm32f4xx.h>

// --- State/context ---
typedef enum {
    I2C_IDLE = 0,
    I2C_WRITE_ADDR,
    I2C_WRITE_DATA,
    I2C_READ_ADDR,
    I2C_READ_DATA,
    I2C_DONE,
    I2C_ERROR,
} i2c_state_t;

typedef struct
{
    uint8_t slave_addr;
    const uint8_t *wbuf;
    uint8_t wlen;
    uint8_t windex;
    uint8_t *rbuf;
    uint8_t rlen;
    uint8_t rindex;
    i2c_callback_t cb;
    i2c_state_t state;
} i2c_ctx_t;

static volatile i2c_ctx_t ctx = { 0 };

// --- Initialization ---
void i2c_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    I2C1->CR2 = PCLK1 / 1e6; // APB1 in MHz
    I2C1->CCR = PCLK1 / (2 * 100000); // 100 kHz standard mode
    I2C1->TRISE = (PCLK1 / 1e6) + 1;

    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

bool i2c_is_busy(void)
{
    return ctx.state != I2C_IDLE;
}

// --- Write ---
bool i2c_write(uint8_t addr, const uint8_t *data, uint8_t len, i2c_callback_t cb)
{
    if (ctx.state != I2C_IDLE)
        return false;

    ctx.slave_addr = addr;
    ctx.wbuf = data;
    ctx.wlen = len;
    ctx.windex = 0;
    ctx.rbuf = NULL;
    ctx.rlen = 0;
    ctx.cb = cb;
    ctx.state = I2C_WRITE_ADDR;

    while (I2C1->SR2 & I2C_SR2_BUSY)
        ; // optional: wait for bus ready
    I2C1->CR1 |= I2C_CR1_START;

    return true;
}

// --- Read ---
bool i2c_read(uint8_t addr, uint8_t *data, uint8_t len, i2c_callback_t cb)
{
    if (ctx.state != I2C_IDLE)
        return false;

    ctx.slave_addr = addr;
    ctx.wbuf = NULL;
    ctx.wlen = 0;
    ctx.rbuf = data;
    ctx.rlen = len;
    ctx.rindex = 0;
    ctx.cb = cb;
    ctx.state = I2C_READ_ADDR;

    while (I2C1->SR2 & I2C_SR2_BUSY)
        ; // optional: wait for bus ready
    I2C1->CR1 |= I2C_CR1_ACK; // enable ACK for multi-byte reads
    I2C1->CR1 |= I2C_CR1_START;

    return true;
}

// --- Event IRQ ---
void i2c1_ev_handler(void)
{
    uint32_t sr1 = I2C1->SR1;

    switch (ctx.state) {

    case I2C_WRITE_ADDR:
        if (sr1 & I2C_SR1_SB) {
            I2C1->DR = (ctx.slave_addr << 1); // write mode
            ctx.state = I2C_WRITE_DATA;
        }
        break;

    case I2C_WRITE_DATA:
        if (sr1 & I2C_SR1_ADDR) {
            (void)I2C1->SR2;
            if (ctx.wlen == 0) {
                I2C1->CR1 |= I2C_CR1_STOP;
                ctx.state = I2C_DONE;
                if (ctx.cb)
                    ctx.cb(0);
                ctx.state = I2C_IDLE;
                return;
            }
        }
        if (sr1 & I2C_SR1_TXE) {
            if (ctx.windex < ctx.wlen) {
                I2C1->DR = ctx.wbuf[ctx.windex++];
            } else {
                I2C1->CR1 |= I2C_CR1_STOP;
                ctx.state = I2C_DONE;
                if (ctx.cb)
                    ctx.cb(0);
                ctx.state = I2C_IDLE;
            }
        }
        break;

    case I2C_READ_ADDR:
        if (sr1 & I2C_SR1_SB) {
            I2C1->DR = (ctx.slave_addr << 1) | 1; // read mode
            ctx.state = I2C_READ_DATA;
        }
        break;

    case I2C_READ_DATA:
        if (sr1 & I2C_SR1_ADDR) {
            if (ctx.rlen == 1) {
                I2C1->CR1 &= ~I2C_CR1_ACK;
            } else {
                I2C1->CR1 |= I2C_CR1_ACK;
            }
            (void)I2C1->SR2;
        }

        if (sr1 & I2C_SR1_RXNE) {
            ctx.rbuf[ctx.rindex++] = I2C1->DR;

            if (ctx.rindex == ctx.rlen - 1 && ctx.rlen > 1) {
                I2C1->CR1 &= ~I2C_CR1_ACK; // prepare NACK for last byte
            }

            if (ctx.rindex == ctx.rlen) {
                I2C1->CR1 |= I2C_CR1_STOP;
                ctx.state = I2C_DONE;
                if (ctx.cb)
                    ctx.cb(0);
                ctx.state = I2C_IDLE;
                I2C1->CR1 |= I2C_CR1_ACK; // re-enable ACK for future reads
            }
        }
        break;

    default:
        break;
    }
}

// --- Error IRQ ---
void i2c1_er_handler(void)
{
    uint32_t sr1 = I2C1->SR1;

    // Just read SR1 to clear error flags
    if (sr1 & I2C_SR1_BERR)
        (void)I2C1->SR1;
    if (sr1 & I2C_SR1_ARLO)
        (void)I2C1->SR1;
    if (sr1 & I2C_SR1_AF)
        (void)I2C1->SR1;
    if (sr1 & I2C_SR1_OVR)
        (void)I2C1->SR1;

    I2C1->CR1 |= I2C_CR1_STOP;

    ctx.state = I2C_ERROR;
    if (ctx.cb)
        ctx.cb(-1);
    ctx.state = I2C_IDLE;
}
