/*
* AD4112-ESP32-SPI-MAIN
* main.c
*
* Author: Mike Dawson
* michaelshanedawson@gmail.com
* Creation Date: 1/26/2024
* Version: v0.01
* Refer to README.md for changelog
* 
* This code is to serve as a driver and interface for the Analog Devices AD411x 24-bit ADC unit. Written around the AD4112 unit but should work with others in the same series.
* Written in VS Code for the ESP32 using the ESP IDF. The registers are either 2 or 3 bytes with the exception of the COMMS and Status registers which are
* a single byte in length.
* All information can be found in the product datasheet, located here : https://www.analog.com/en/products/ad4112.html#product-documentation
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

/*Pin Configurations for the SPI bus*/
#define SPI_HOST SPI2_HOST
#define PIN_NUM_MISO 19 //MISO Pin
#define PIN_NUM_MOSI 7 //MOSI Pin
#define PIN_NUM_CS 0 //CS (SS) Pin
#define PIN_NUM_CLK 1 //SPI CLK Pin

#define generic_gpio_pin 3

/*Declare function prototypes*/
void spi_write(spi_device_handle_t spi, uint8_t reg, uint32_t value);
uint32_t spi_read(spi_device_handle_t spi, uint8_t reg);

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
}

void app_main(void)
{
    /*Configure GPIO pins*/
    gpio_reset_pin(generic_gpio_pin);
    gpio_set_direction(generic_gpio_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(generic_gpio_pin, 0);

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

    /* Here we will perform a basic test to see if we can get the ID of the chip*/ 
    int32_t registerData = spi_read(spi, 0x7);
    registerData = SPI_SWAP_DATA_RX(registerData, 32); //I have no idea why this is necessary at the moment
    printf("The Chip ID is: %#lx \n", registerData); //The datasheet shows the ID for the AD4112 should be 0x30DX where X is not important and can be anything
    }

