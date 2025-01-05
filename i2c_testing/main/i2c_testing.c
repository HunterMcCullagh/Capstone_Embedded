#include <stdio.h>
#include <driver/i2c.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM I2C_NUM_0        // I2C port number
#define I2C_MASTER_SCL_IO 19           // GPIO for SCL
#define I2C_MASTER_SDA_IO 18           // GPIO for SDA
#define I2C_MASTER_FREQ_HZ 100000      // Frequency in Hz (100 kHz standard)
#define I2C_MASTER_TX_BUF_DISABLE 0    // Disable transmit buffer
#define I2C_MASTER_RX_BUF_DISABLE 0    // Disable receive buffer



// Function to initialize the I2C driver
void i2c_master_init() {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode,
                                       I2C_MASTER_RX_BUF_DISABLE,
                                       I2C_MASTER_TX_BUF_DISABLE, 0));
}

// Function to scan for I2C peripherals
void i2c_scan() {
    printf("Scanning for I2C devices...\n");
    for (uint8_t addr = 1; addr < 10; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | 0x50, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("Device found at address 0x%02X\n", addr);
        }
    }
    printf("I2C scan completed.\n");
}

// Function to write to a register
esp_err_t i2c_write_register(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function to read from a register
esp_err_t i2c_read_register(uint8_t device_addr, uint8_t reg_addr, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function to dump all registers of a device
void i2c_dump_registers(uint8_t device_addr) {
    printf("Dumping registers for device at 0x%02X:\n", device_addr);
    for (uint8_t reg = 0; reg < 0xFF; reg++) {
        uint8_t data;
        if (i2c_read_register(device_addr, reg, &data) == ESP_OK) {
            printf("Register 0x%02X: 0x%02X\n", reg, data);
        } else {
            printf("Register 0x%02X: Read error\n", reg);
        }
    }
    printf("Register dump completed.\n");
}


void app_main(void)
{
    i2c_master_init();  // Initialize I2C driver
    uint8_t i2c_address = 0x30;
    uint8_t i2c_register = 0x00;
    uint8_t i2c_data = 0x2F;

    while(1)
    {
        i2c_scan();         // Scan for I2C devices
        //i2c_write_register(i2c_address,i2c_register,i2c_data);
        //printf("Register write completed.\n");
        vTaskDelay(pdMS_TO_TICKS(4000)); // Delay for 1000 ms (1 second)
    }


    // uint8_t device_addr = 0x68; // Example device address (adjust as needed)
    // uint8_t reg_addr = 0x6B;    // Example register address (adjust as needed)
    // uint8_t data = 0x00;        // Example data to write

    // // Write to a register
    // if (i2c_write_register(device_addr, reg_addr, data) == ESP_OK) {
    //     printf("Successfully wrote 0x%02X to register 0x%02X on device 0x%02X\n",
    //            data, reg_addr, device_addr);
    // } else {
    //     printf("Failed to write to register 0x%02X on device 0x%02X\n",
    //            reg_addr, device_addr);
    // }

    // // Read from a register
    // if (i2c_read_register(device_addr, reg_addr, &data) == ESP_OK) {
    //     printf("Read 0x%02X from register 0x%02X on device 0x%02X\n",
    //            data, reg_addr, device_addr);
    // } else {
    //     printf("Failed to read from register 0x%02X on device 0x%02X\n",
    //            reg_addr, device_addr);
    // }

    // // Dump all registers
    //i2c_dump_registers(0x3C);
    
}
