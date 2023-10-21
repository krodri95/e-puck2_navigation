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
#include "sensors/proximity.h"
#include "motors.h"
#include "follower.h"

#define SCAN_THR 300 //30cm
#define TARGET_THR 25 //2.5cm
#define PRX_NOISE_THR 50
#define SCAN_SPEED MOTOR_SPEED_LIMIT*0.8

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

uint16_t find_distance(void) {

    uint32_t pSum = 0, nSum = 0;;
    unsigned int N = 5;

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

void motor_stop(void) {
    // stop moving
    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

void motor_move_fw(void) {
    // move forward
    left_motor_set_speed(MOTOR_SPEED_LIMIT);
    right_motor_set_speed(MOTOR_SPEED_LIMIT);
}

void motor_move_bw(void) {
    // move backward
    left_motor_set_speed(-MOTOR_SPEED_LIMIT);
    right_motor_set_speed(-MOTOR_SPEED_LIMIT);
}

void motor_move(void) {

    uint16_t dist = find_distance();

    if(dist >= TARGET_THR) {
        // move forward
        motor_move_fw();
    } else if (dist < TARGET_THR - 10) {
        // move backward
        motor_move_bw();
    } else {
        // stop moving
        motor_stop();
    }
}

void target_found(void) {
    // light the leds yellow
    set_rgb_led(LED2, 255, 255, 0);
    set_rgb_led(LED4, 255, 255, 0);
    set_rgb_led(LED6, 255, 255, 0);
    set_rgb_led(LED8, 255, 255, 0);

}

void target_locked(void) {
    // light the leds green
    set_rgb_led(LED2, 0, 255, 0);
    set_rgb_led(LED4, 0, 255, 0);
    set_rgb_led(LED6, 0, 255, 0);
    set_rgb_led(LED8, 0, 255, 0);

}

void target_lost(void) {
    // light the leds red
    set_rgb_led(LED2, 255, 0, 0);
    set_rgb_led(LED4, 255, 0, 0);
    set_rgb_led(LED6, 255, 0, 0);
    set_rgb_led(LED8, 255, 0, 0);

    // start rotating clockwise
    left_motor_set_speed(SCAN_SPEED);
    right_motor_set_speed(-SCAN_SPEED);
}

void get_cal_prox_readings(int *cal_prox) {

    unsigned int N = 5;

    // averaging
    for (unsigned int n = 0; n < N; n++) {
        //sensors 0-7
        for(unsigned int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
            cal_prox[i] += get_calibrated_prox(i);
        }
        chThdSleepMilliseconds(5);
    }
    
    for(unsigned int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
        cal_prox[i] /= N;
        if(cal_prox[i] < PRX_NOISE_THR) {
            cal_prox[i] = 0;
        }
    }

}

void scan_target(uint16_t dist) {
    //scan
    target_lost();
    while(dist >= SCAN_THR) {
        dist = find_distance();
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

    uint16_t distance = 0, sum_sensors = 0;
    int left_speed, right_speed;
    int cal_prox[PROXIMITY_NB_CHANNELS];

    /* Infinite loop. */
    while (1) {

        distance = find_distance();
        // find the target
        if (distance >= SCAN_THR) {
            scan_target(distance);
        }

        // move towards or away from the target
        motor_move();

        distance = find_distance();
        // in case target changes position
        if (distance >= SCAN_THR) {
            scan_target(distance);
            // move towards or away from the target
            motor_move();
        }

        // approaching the target
        distance = find_distance();
        if(distance <= TARGET_THR) {
            // target in range
            target_locked();
            motor_move();
        }

        // Consider small values to be noise thus set them to zero.
        get_cal_prox_readings(cal_prox);

        // The following table shows the weights of all the proximity sensors for the resulting rotation.
        //  Prox	0		1		2		3		4		5		6		7
        //	w		-0.5	-0.5	-1		-1		1	    1       0.5 	0.5

        // Sum the contribution of each sensor (based on the previous weights table).
        sum_sensors = -(cal_prox[0]>>1) - (cal_prox[1]>>1) - cal_prox[2] - cal_prox[3] + cal_prox[4] + cal_prox[5] + (cal_prox[6]>>1) + (cal_prox[7]>>1);

        // Modify the velocity components based on sensor values.
        left_speed = left_motor_get_desired_speed();
        right_speed = right_motor_get_desired_speed();

        left_speed -= (sum_sensors<<2);
        right_speed += ((sum_sensors<<2));
 
        left_motor_set_speed(left_speed);
        right_motor_set_speed(right_speed);

        str_length = sprintf(str, "distance,%d, cal_prox,%d,%d,%d,%d,%d,%d,%d,%d\n",distance,cal_prox[0],cal_prox[1],cal_prox[2],
                             cal_prox[3],cal_prox[4],cal_prox[5],cal_prox[6],cal_prox[7]);
        e_send_uart1_char(str, str_length);

        // waits 20 milliseconds
        chThdSleepMilliseconds(20);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
