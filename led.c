#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "leds.h"
#include "spi_comm.h"


int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    // Your initialisations here
    clear_leds();
    spi_comm_start();

    //set_body_led(1);
    //set_front_led(1);

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(500);

        /* 
        led_name_t led_number, unsigned int value
        typedef enum {
                LED1,
                LED3,
                LED5,
                LED7,
                NUM_LED,
        } led_name_t;
        */
        set_led(LED5, 1); //0 for off, 1 for on, or 2 to toggle

        //waits 1 second
        chThdSleepMilliseconds(500);

        /* 
        led_name_t led_number, unsigned int value
        typedef enum {
                LED1,
                LED3,
                LED5,
                LED7,
                NUM_LED,
        } led_name_t;
        */
        set_led(LED5, 0); //0 for off, 1 for on, or 2 to toggle

        /*
        rgb_led_name_t led_number, int red_val, int green_val, int blue_val
        typedef enum {
                LED2,
                LED4,
                LED6,
                LED8,
                NUM_RGB_LED,
        } rgb_led_name_t;
        */
        set_rgb_led(LED2, 255, 255, 0);
        set_rgb_led(LED8, 255, 0, 255);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
