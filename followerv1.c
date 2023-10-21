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
#include "sensors/VL53L0X/VL53L0X.h"

#define SCAN_THR 300 //30cm
#define TARGET_THR 30 //3cm
#define PRX_NOISE_THR 20
#define SCAN_SPEED MOTOR_SPEED_LIMIT*0.75

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void motor_stop() {
    // stop moving
    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

void motor_move_fw() {
    // move forward
    left_motor_set_speed(MOTOR_SPEED_LIMIT);
    right_motor_set_speed(MOTOR_SPEED_LIMIT);
}

void motor_move_bw() {
    // move backward
    left_motor_set_speed(-MOTOR_SPEED_LIMIT);
    right_motor_set_speed(-MOTOR_SPEED_LIMIT);
}

void target_found() {
    // light the leds yellow
    set_rgb_led(LED2, 255, 255, 0);
    set_rgb_led(LED4, 255, 255, 0);
    set_rgb_led(LED6, 255, 255, 0);
    set_rgb_led(LED8, 255, 255, 0);

}

void target_locked() {
    // light the leds green
    set_rgb_led(LED2, 0, 255, 0);
    set_rgb_led(LED4, 0, 255, 0);
    set_rgb_led(LED6, 0, 255, 0);
    set_rgb_led(LED8, 0, 255, 0);

}

void target_lost() {
    // light the leds red
    set_rgb_led(LED2, 255, 0, 0);
    set_rgb_led(LED4, 255, 0, 0);
    set_rgb_led(LED6, 255, 0, 0);
    set_rgb_led(LED8, 255, 0, 0);

    // start rotating clockwise
    left_motor_set_speed(SCAN_SPEED);
    right_motor_set_speed(-SCAN_SPEED);
}

uint16_t find_distance() {

    uint32_t pSum = 0;
    uint32_t nSum = 0;
    unsigned int N = 5;

    //reset the sum
    pSum = 0;
    // simple moving average
    for (unsigned int p = 0; p < N; p++) {
        nSum = 0;
        for (unsigned int n = 0; n < N; n++) {
            // ToF distance sensor
            nSum += VL53L0X_get_dist_mm();
            //chThdSleepMilliseconds(5);
        }
        pSum += nSum/N;
    }

    return pSum/N;
}

void scan_target() {
    //scan
    target_lost();
    while(distance >= SCAN_THR) {
        distance = find_distance();
    }
    target_found();
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    // Your initialisations here
    clear_leds();
    spi_comm_start();
    VL53L0X_start();
    motors_init();

    //UART1
    serial_start();
    char str[100];
    int str_length;

    uint16_t distance = 0;

    /* Infinite loop. */
    while (1) {
    	// waits 1 second
        //chThdSleepMilliseconds(20);

        distance = find_distance();
        // find the target
        if (distance >= SCAN_THR) {
            scan_target();
        }

        // stop rotating and move towards the target
        motor_move_fw();

        // approaching the target
        while(distance >= TARGET_THR) {
            distance = find_distance();
            // in case target changes position
            if (distance >= SCAN_THR) {
                scan_target();
                // stop rotating and move towards the target
                motor_move_fw();
            }
        }

        // target in range
        target_locked();
        motor_stop();
        
        str_length = sprintf(str, "distance,%d\n",distance);
        e_send_uart1_char(str, str_length);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
