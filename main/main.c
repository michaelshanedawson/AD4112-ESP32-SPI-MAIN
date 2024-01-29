/*
* AD4112-ESP32-SPI-MAIN
* main.c
*
* Author: Mike Dawson
* michaelshanedawson@gmail.com
* Creation Date: 1/26/2024
* Version: v0.02
* Refer to README.md for changelog
* 
* This code is to serve as a driver and interface for the Analog Devices AD411x 24-bit ADC unit. Written around the AD4112 unit but should work with others in the same series.
* Written in VS Code for the ESP32 using the ESP IDF. The registers are either 2 or 3 bytes with the exception of the COMMS and Status registers which are
* a single byte in length.
* All information can be found in the product datasheet, located here : https://www.analog.com/en/products/ad4112.html#product-documentation
*
* The code was written around the Evaluation kit from Analog Devices. The user guide cna be found here: 
* https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD4111.html#eb-documentation
*
* This particular ADC unit requires a lot of configuration and it features many options. This code example will follow the recommended flow in the datasheet.
*
* Step A:
* Channel configuration. Select the input and setup for each channel
*
* Step B:
* Setup configuration. With 8 possible ADC setups, we will select the filter order, output data rate and more.
*
* Step C:
* ADC mode and interface mode configuration. Select the ADC operating mode, clock source, enable CRC (if desired), data and status and more.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/*Pin Configurations for the SPI bus*/
#define SPI_HOST SPI2_HOST
#define PIN_NUM_MISO 19 //MISO Pin
#define PIN_NUM_MOSI 7 //MOSI Pin
#define PIN_NUM_CS 0 //CS (SS) Pin
#define PIN_NUM_CLK 1 //SPI CLK Pin

#define generic_gpio_pin 3

/*Configure an enum for the channels to make it easier to understand what is going on in the code*/
enum channel_selection
{
    CH0 = 0x10,
    CH1 = 0x11,
    CH2 = 0x12,
    CH3 = 0x13,
    CH4 = 0x14,
    CH5 = 0x15,
    CH6 = 0x16,
    CH7 = 0x17,
    CH8 = 0x18,
    CH9 = 0x19,
    CH10 = 0x1A,
    CH11 = 0x1B,
    CH12 = 0x1C,
    CH13 = 0x1D,
    CH14 = 0x1E,
    CH15 = 0x1F,
} ;

/*Configure an enum for the setup selection*/
enum setup_selection
{
    SETUP0 = 0b000,
    SETUP1 = 0b001,
    SETUP2 = 0b010,
    SETUP3 = 0b011,
    SETUP4 = 0b100,
    SETUP5 = 0b101,
    SETUP6 = 0b110,
    SETUP7 = 0b111,
};

/*Configure an enum for single ended input pair selections, 8 possible single ended input combinations*/
enum single_input_selection
{
    INPUT0 = 0b0000010000, //VIN0 to VINCOM
    INPUT1 = 0b0000110000, //VIN1 to VINCOM
    INPUT2 = 0b0001010000, //VIN2 to VINCOM
    INPUT3 = 0b0001110000, //VIN3 to VINCOM
    INPUT4 = 0b0010010000, //VIN4 to VINCOM
    INPUT5 = 0b0010110000, //VIN5 to VINCOM
    INPUT6 = 0b0011010000, //VIN6 to VINCOM
    INPUT7 = 0b0011110000, //VIN7 to VINCOM
};

/*Configure an enum for the setup configuration registers, 8 total*/
enum setup_configuration_selection
{
    SETUPCON0 = 0x20,
    SETUPCON1 = 0x21,
    SETUPCON2 = 0x22,
    SETUPCON3 = 0x23,
    SETUPCON4 = 0x24,
    SETUPCON5 = 0x25,
    SETUPCON6 = 0x26,
    SETUPCON7 = 0x27,
};

/*Configure an enum for the filter configuration registers, 8 total*/
enum filter_configuration_selection
{
    FILTERCON0 = 0x28,
    FILTERCON1 = 0x29,
    FILTERCON2 = 0x2A,
    FILTERCON3 = 0x2B,
    FILTERCON4 = 0x2C,
    FILTERCON5 = 0x2D,
    FILTERCON6 = 0x2E,
    FILTERCON7 = 0x2F,
};

/*Declare function prototypes*/
void spi_write(spi_device_handle_t spi, uint8_t reg, uint32_t value);
uint32_t spi_read(spi_device_handle_t spi, uint8_t reg);
void device_reset(spi_device_handle_t spi);

/*GPIO Pulse*/
void pulse(uint8_t pin)
{
    gpio_set_level(pin, 1);
    gpio_set_level(pin, 0);
}

uint32_t spi_read(spi_device_handle_t spi, uint8_t reg)
{
    /*
    * All register access starts with a write to the COMMs register to determine what register is being accessed next and if
    * the operation is a write or a read of that register.
    */    
    uint8_t operationByte = 0x00 | 1 << 6 | reg; //Since this is a read function, default to a read operation each time

    spi_device_acquire_bus(spi, portMAX_DELAY);
    esp_err_t ret;
    spi_transaction_t t;     
    
    memset(&t, 0, sizeof(t));                   //Zero out the transaction
    t.length=8;                                 //Data Length in bits
    t.tx_buffer=&operationByte;                 //The data is the command to the COMMs register
    t.user=(void*)0;                            //D/C needs to be set to 0
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE;         //Keep CS active after data transfer
    ret=spi_device_polling_transmit(spi, &t);   //Transmit!
    assert(ret==ESP_OK);                        //Should have had no issues.    

    memset(&t, 0, sizeof(t));                   //Zero out the transaction
    t.length=32;                                //Data size is 32 bits
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;
    ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );    
    spi_device_release_bus(spi);                // Release bus
    return *(int32_t*)t.rx_data;                //Performs the return of the requested register data

}

void spi_write(spi_device_handle_t spi, uint8_t reg, uint32_t value)
{
    /*
    * All register access starts with a write to the COMMs register to determine what register is being accessed next and if
    * the operation is a write or a read of that register.
    */    
    uint8_t operationByte = 0x00 | reg; //Since this is a write function, default to a write operation each time

    spi_device_acquire_bus(spi, portMAX_DELAY);
    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));                   //Zero out the transaction
    t.length=8;                                 //Data Length in bits
    t.tx_buffer=&operationByte;                 //The data is the command to the COMMs register
    t.user=(void*)0;                            //D/C needs to be set to 0
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE;         //Keep CS active after data transfer
    ret=spi_device_polling_transmit(spi, &t);   //Transmit!
    assert(ret==ESP_OK);                        //Should have had no issues.   

    memset(&t, 0, sizeof(t));                    //Zero out the transaction
    t.length=32;                                 //Data Length in bits
    t.tx_buffer=&value;                         //The data is the value to write to the selected register
    t.user=(void*)0;                            //D/C needs to be set to 0
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE;         //Keep CS active after data transfer
    ret=spi_device_polling_transmit(spi, &t);   //Transmit!
    assert(ret==ESP_OK);                        //Should have had no issues.   
    spi_device_release_bus(spi);                // Release bus    
}

void device_reset(spi_device_handle_t spi)
{
    spi_device_acquire_bus(spi, portMAX_DELAY);
    esp_err_t ret;
    spi_transaction_t t;
    uint32_t value = 0xFFFFFFFF;

    memset(&t, 0, sizeof(t));                    //Zero out the transaction
    t.length=32;                                 //Data Length in bits
    t.tx_buffer=&value;                         //The data is the value to write to the selected register
    t.user=(void*)0;                            //D/C needs to be set to 0
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE;         //Keep CS active after data transfer
    ret=spi_device_polling_transmit(spi, &t);   //Transmit!
    assert(ret==ESP_OK);                        //Should have had no issues.

    memset(&t, 0, sizeof(t));                    //Zero out the transaction
    t.length=32;                                 //Data Length in bits
    t.tx_buffer=&value;                         //The data is the value to write to the selected register
    t.user=(void*)0;                            //D/C needs to be set to 0   
    ret=spi_device_polling_transmit(spi, &t);   //Transmit!
    assert(ret==ESP_OK);                        //Should have had no issues.   
    spi_device_release_bus(spi);                // Release bus        
}

void app_main(void)
{
    /*Configure GPIO pins*/
    gpio_reset_pin(generic_gpio_pin);
    gpio_set_direction(generic_gpio_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(generic_gpio_pin, 0);

    /*Initialize the defined enums*/
    enum channel_selection channel;
    enum setup_selection setup;
    enum single_input_selection singleInput;
    enum setup_configuration_selection setupConf;
    enum filter_configuration_selection filterConf;

    /*Initializes the SPI driver*/
    esp_err_t ret;
    spi_device_handle_t spi;

    /*Configures the SPI driver*/
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0,
    };

    /*Device specific SPI settings*/
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=20*1000*1000,           //Clock out at 20 MHz
        .mode=3,                                //SPI mode 3
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
    };

    /*Initialize the SPI bus*/
    ret=spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    /*Attach the AD411x to the SPI bus*/
    ret=spi_bus_add_device(SPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    /*Send a device reset to the AD4112 unit*/
    vTaskDelay(100 / portTICK_PERIOD_MS);  //Delay the task for x number of milliseconds. 1000mS = 1 second.
    device_reset(spi);
    vTaskDelay(100 / portTICK_PERIOD_MS);  //Delay the task for x number of milliseconds. 1000mS = 1 second.

    /* Here we will perform a basic test to see if we can get the ID of the chip*/ 
    int32_t registerData = spi_read(spi, 0x7);
    registerData = SPI_SWAP_DATA_RX(registerData, 32); //I have no idea why this is necessary at the moment
    printf("The Chip ID is: %#lx \n", registerData); //The datasheet shows the ID for the AD4112 should be 0x30DX where X is not important and can be anything

    /* Here we will start the configuration of the ADC channels. There are 16 in total and they all follow the same general methodology.
    *  The ADC has a default configuration at power up but we will go through the process of configuring it ourselves. For now we will setup channel 0.
    *  The channel configuration is 16 bits in length. The ADC chip keeps clocking data in and shifting to the left, thus a 32 bit value is ok as only the final 
    *  16 bits remain as valid once the CLK goes idle when the transmission is completed.
    */ 

    uint32_t dataOut = 0x00; //A simple local variable to configure all SPI data write transactions

    dataOut = 1 << 15 | (setup = SETUP0) << 12 | (singleInput = INPUT1); // bit position 15 is 1 to enable the channel. 
    spi_write(spi, channel=CH0, dataOut);    

    /* Here we will configure the ADC setups. There are eight independent setups that can be used. Each setup contains the following four registers:
    *  Setup Configuration Register, Filter Configuration Register, Gain Register and Offset Register. The registers are all configured as a group. 
    *  Setup 0 goes with Filter 0, Gain 0 and Offset 0.
    */

    /* First we will configure the Setup Configuration for #0*/
    dataOut = 0x00 | 0b11 << 8;
    spi_write(spi, setupConf = SETUPCON0, dataOut);

    /* Now we will configure the Filter for #0. The filter is more complicated and some things depend on how many channels are active. Refer to datasheet for more info.*/
    dataOut = 0x00 | 1 << 11 | 0b010 << 8 | 0b01000;
    spi_write(spi, filterConf = FILTERCON0, dataOut);

    /* The Offset and Gain registers are used to compensate for errors in the ADC. We will not configure them at this stage, need to know how it performs first*/

    /* Here we will configure the ADC Mode register*/
    dataOut = 0x00 | 0b11 << 2;
    spi_write(spi, 0x1, dataOut); //0x1 is the address of the ADC mode register

    /* Here we will configure the ADC Interface register*/
    dataOut = 0x00; 
    spi_write(spi, 0x2, dataOut); //0x2 is the address of the ADC mode register


    registerData = spi_read(spi, 0x0);
    //registerData = (registerData & 0xFF);
    printf("The register data is: %#lx \n", (registerData & 0xFF)); 

    while(1)
    {
        uint8_t dataReady = 0x00;
        uint8_t channelReady = 0x00;
        registerData = spi_read(spi, 0x0);
        //printf("The register data is: %#lx \n", registerData);
        registerData = (registerData & 0xFF);
        dataReady = (registerData & 0x80);
        //printf("The data ready bit is: %#x \n", dataReady);
        channelReady = (registerData & 0xF);
        //printf("The register data is: %#lx \n", registerData);
        //printf("Data status = %u \n", dataReady);
        //printf("Channel read = %u \n", channelReady);

        if(dataReady == 0)
        {
            int32_t adcData = 0x00;
            adcData = spi_read(spi, 0x4);
            //printf("The ADC data is: %#lx \n", (adcData & 0xFFFFFF)); 
            printf("ADC Data = %.4f \n", (double)(adcData & 0xFFFFFF));            
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);  //Delay the task for x number of milliseconds. 1000mS = 1 second.
    }

    }

