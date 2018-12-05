#include <hal.h>
#include <modules/driver_ak09916/driver_ak09916.h>
#include <common/ctor.h>

/**
 * TIMINGR register definitions
 */
#define STM32_TIMINGR_PRESC_MASK        (15U << 28)
#define STM32_TIMINGR_PRESC(n)          ((n) << 28)
#define STM32_TIMINGR_SCLDEL_MASK       (15U << 20)
#define STM32_TIMINGR_SCLDEL(n)         ((n) << 20)
#define STM32_TIMINGR_SDADEL_MASK       (15U << 16)
#define STM32_TIMINGR_SDADEL(n)         ((n) << 16)
#define STM32_TIMINGR_SCLH_MASK         (255U << 8)
#define STM32_TIMINGR_SCLH(n)           ((n) << 8)
#define STM32_TIMINGR_SCLL_MASK         (255U << 0)
#define STM32_TIMINGR_SCLL(n)           ((n) << 0)

#define AK09916_I2C_ADDR 0x0C

static void i2c_serve_interrupt(uint32_t isr) {
    static uint8_t i2c2_transfer_byte_idx;
    static uint8_t i2c2_transfer_address;
    static uint8_t i2c2_transfer_direction;
    if (isr & (1<<3)) { // ADDR
        i2c2_transfer_address = (isr >> 17) & 0x7FU; // ADDCODE
        i2c2_transfer_direction = (isr >> 16) & 1; // direction
        i2c2_transfer_byte_idx = 0;
        if (i2c2_transfer_direction) {
            I2C2->ISR |= (1<<0); // TXE
        }
        I2C2->ICR |= (1<<3); // ADDRCF
    }

    if (isr & I2C_ISR_RXNE) {
        uint8_t recv_byte = I2C2->RXDR & 0xff;; // reading clears our interrupt flag
        switch(i2c2_transfer_address) {
            case AK09916_I2C_ADDR:
                ak09916_recv_byte(i2c2_transfer_byte_idx, recv_byte);
                break;
        }
        i2c2_transfer_byte_idx++;
    }

    if (isr & I2C_ISR_TXIS) {
        uint8_t send_byte = 0;
        switch(i2c2_transfer_address) {
            case AK09916_I2C_ADDR:
                send_byte = ak09916_send_byte();
                break;
        }
    	I2C2->TXDR = send_byte;

        i2c2_transfer_byte_idx++;
    }
}

OSAL_IRQ_HANDLER(STM32_I2C2_EVENT_HANDLER) {
    uint32_t isr = I2C2->ISR;

    OSAL_IRQ_PROLOGUE();

    i2c_serve_interrupt(isr);

    OSAL_IRQ_EPILOGUE();
}

RUN_AFTER(INIT_END) {
    rccEnableI2C2(FALSE);
    rccResetI2C2();

    //Disable I2C
    I2C2->CR1 &= ~I2C_CR1_PE;

    //Enable Analog Filter
    I2C2->CR1 &= ~I2C_CR1_ANFOFF;

    //Disable Digital Filter
    I2C2->CR1 &=  ~(I2C_CR1_DNF);

    //Set Prescaler
    I2C2->TIMINGR = (I2C2->TIMINGR & ~STM32_TIMINGR_PRESC_MASK) |
                (STM32_TIMINGR_PRESC(8));

    //Set Data Setup Time
    I2C2->TIMINGR = (I2C2->TIMINGR & ~STM32_TIMINGR_SCLDEL_MASK) |
                (STM32_TIMINGR_SCLDEL(9));

    //Set Data Hold Time
    I2C2->TIMINGR = (I2C2->TIMINGR & ~STM32_TIMINGR_SDADEL_MASK) |
                (STM32_TIMINGR_SDADEL(11));

    //Enable Stretching
    I2C2->CR1 &= ~I2C_CR1_NOSTRETCH;

    //7Bit Address Mode
    I2C2->CR2 &= ~I2C_CR2_ADD10;

    I2C2->OAR1 = (AK09916_I2C_ADDR & 0xFF) << 1; //Emulate Toshiba LED I2C Slave
    I2C2->OAR1 |= (1<<15);

    //Enable I2C interrupt
    nvicEnableVector(I2C2_EV_IRQn, 3);

    I2C2->CR1 |= (1<<1); // TXIE
    I2C2->CR1 |= (1<<2); // RXIE
    I2C2->CR1 |= (1<<3); // ADDRIE
    I2C2->CR1 |= I2C_CR1_PE; // Enable I2C
}
