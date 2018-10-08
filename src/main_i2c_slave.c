#include <hal.h>
#include <main_i2c_slave.h>
#include <modules/driver_ak09916/driver_ak09916.h>

#define TOSHIBALED_I2C_ADDRESS 0x55
#define AK09916_I2C_ADDR 0x0C

static uint32_t led_color_hex;
static uint8_t led_reg;
static bool new_led_data = false;

static void toshibaled_interface_recv_byte(uint8_t recv_byte_idx, uint8_t recv_byte) {
    if (recv_byte_idx == 0 || ((recv_byte&(1<<7)) != 0)) {
        led_reg = recv_byte & ~(1<<7);
    } else {
        switch(led_reg) {
            case 1:
                led_color_hex &= ~((uint32_t)0xff<<0);
                led_color_hex |= (uint32_t)(((recv_byte << 4)&0xf0) | (recv_byte&0x0f))<<0;
                break;
            case 2:
                led_color_hex &= ~((uint32_t)0xff<<8);
                led_color_hex |= (uint32_t)(((recv_byte << 4)&0xf0) | (recv_byte&0x0f))<<8;
                break;
            case 3:
                led_color_hex &= ~((uint32_t)0xff<<16);
                led_color_hex |= (uint32_t)(((recv_byte << 4)&0xf0) | (recv_byte&0x0f))<<16;
                break;
        }
        led_reg++;
        new_led_data = true;
    }
}

bool i2c_slave_led_updated(void) {
    return new_led_data;
}
uint32_t i2c_slave_retrieve_led_color_hex(void) {
    new_led_data = false;
    return led_color_hex;
}

void i2c_serve_interrupt(uint32_t isr) {
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
            case TOSHIBALED_I2C_ADDRESS:
                toshibaled_interface_recv_byte(i2c2_transfer_byte_idx, recv_byte);
                break;
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
