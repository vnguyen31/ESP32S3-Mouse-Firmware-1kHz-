//includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

//pin definitions
#define Button1     8
#define Button2     3
#define EncoderA    47
#define EncoderB    20
#define MISO        12
#define MOSI        11
#define SCLK        13
#define NCS         10


//main function
void app_main()
{
    //analog to digital converter variables
    int button1_digital, button2_digital, button1_voltage, button2_voltage;

    //analog to digital converter module setup
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL_7, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC_CHANNEL_2, ADC_ATTEN_DB_11);

    //scroll wheel digital input setup
    gpio_set_direction(EncoderA, GPIO_MODE_INPUT);      //set pin 47 to input
    gpio_set_direction(EncoderB, GPIO_MODE_INPUT);      //set pin 3 to input
    gpio_pullup_en(EncoderA);                         //pull down resistor enable pin 47
    gpio_pullup_en(EncoderB);                         //pull down resistor enable pin 3

    //Scroll wheel encoder variables
    uint8_t posCurrent = 0;
    uint8_t posPrevious = 0;
    uint8_t scroll = 0;


    //loop forever part
    while (1){

        //Analog switches reading
        button1_digital = adc1_get_raw(ADC1_CHANNEL_7);
        button2_digital = adc1_get_raw(ADC1_CHANNEL_2);
        button1_voltage = button1_digital * 2600/2981;
        button2_voltage = button2_digital * 2600/2981;
        printf("click 1: %d mV \tclick 2: %d mV\t", button1_voltage, button2_voltage);
        //vTaskDelay(pdMS_TO_TICKS(1000));

        //finding state of scroll based on A and B states
        uint8_t Astate = gpio_get_level(EncoderA);
        uint8_t Bstate = gpio_get_level(EncoderB);

        if ((Astate == 0) && (Bstate == 0)){
            posCurrent = 0;
        }
        else if ((Astate == 1) && (Bstate == 0)){
            posCurrent = 1;
        }
        else if ((Astate == 1) && (Bstate == 1)){
            posCurrent = 2;
        }

        //difference between current and previous position
        int8_t posDifference = posCurrent - posPrevious;

        //clockwise turn:
        if (posDifference == 0){
            printf("Scroll no change\t");
        }

        else if ((posDifference == 2) || (posDifference == -1)){
            posPrevious = posCurrent;
            if (scroll < 255){ scroll++; }
            printf("Scroll down\t");
        }
        //counterclockwise turn:
        else if ((posDifference == 1) || (posDifference == -2)){
            posPrevious = posCurrent;
            if (scroll > 0){ scroll--; }
            printf("Scroll up\t");
        }

        //in case a step got lost or turned too fast, IGNORE
        else if (posDifference == 3 || posDifference == -3){
            //blank
            printf("data lost\t");
        }

        printf("scroll var: %d\n", scroll);

        //reset scroll
        if (scroll == 255){
            scroll = 0;
        }
    }
}