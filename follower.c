/*
 * File:          follower.c
 * Date:          25/10/2023
 * Description:   Chase an object using the distance and proximity sensors
 * Author:        Keith Rodrigues
 * Modifications:
 */
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
#include "selector.h"

//#include "follower.h"

#define SCAN_SPEED MOTOR_SPEED_LIMIT*0.5

//Thresholds
#define SCAN_THR 150 //15?cm
#define TARGET_THR 60 //4.0cm
#define COLLISION_THR 40 //2.0cm
#define PRX_NOISE_THR 100
#define PRX_CLOSE_THR 900

static bool go_right = true;
static float speed_factor =  0.5;

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

uint16_t find_distance(void) {

    uint32_t pSum = 0, nSum = 0;;
    unsigned int N = 5;

    // simple average
    for (unsigned int p = 0; p < N; p++) {
        nSum = 0;
        for (unsigned int n = 0; n < N; n++) {
            // ToF distance sensor
            nSum += VL53L0X_get_dist_mm();
        }
        pSum += nSum/N;
        chThdSleepMilliseconds(1);
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
    left_motor_set_speed(MOTOR_SPEED_LIMIT*speed_factor);
    right_motor_set_speed(MOTOR_SPEED_LIMIT*speed_factor);
}

void motor_move_bw(void) {
    // move backward
    left_motor_set_speed(-MOTOR_SPEED_LIMIT*speed_factor);
    right_motor_set_speed(-MOTOR_SPEED_LIMIT*speed_factor);
}

void motor_move(void) {

    uint16_t dist = find_distance();

    if(dist > TARGET_THR && dist < SCAN_THR) {
        // move forward
        motor_move_fw();
     } else if (dist < COLLISION_THR) {
         // move backward
         motor_move_bw();
    } else {
        // stop moving
        motor_stop();
    }
}

void target_found(void) {
    // light the leds yellow
    set_rgb_led(LED2, 10, 10, 0);
    set_rgb_led(LED4, 10, 10, 0);
    set_rgb_led(LED6, 10, 10, 0);
    set_rgb_led(LED8, 10, 10, 0);

}

void target_locked(void) {
    // light the leds green
    set_rgb_led(LED2, 0, 10, 0);
    set_rgb_led(LED4, 0, 10, 0);
    set_rgb_led(LED6, 0, 10, 0);
    set_rgb_led(LED8, 0, 10, 0);

}

void target_lost(void) {
    // light the leds red
    set_rgb_led(LED2, 10, 0, 0);
    set_rgb_led(LED4, 10, 0, 0);
    set_rgb_led(LED6, 10, 0, 0);
    set_rgb_led(LED8, 10, 0, 0);

    if (go_right) {
        // start rotating clockwise
        left_motor_set_speed(SCAN_SPEED);
        right_motor_set_speed(-SCAN_SPEED);
    } else {
        // start rotating ant-clockwise
        left_motor_set_speed(-SCAN_SPEED);
        right_motor_set_speed(SCAN_SPEED);
    }

}

void get_prox_readings(int *prox, unsigned int *mIdx) {

    unsigned int N = 5;

    // averaging
    for (unsigned int n = 0; n < N; n++) {
        //sensors 0-7
        for(unsigned int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
            prox[i] += get_prox(i);
        }
        chThdSleepMilliseconds(1);
    }
    
    int lVal = 0;
    for(unsigned int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
        prox[i] /= N;

        if(prox[i] > lVal) {
            lVal = prox[i];
            *mIdx = i;
        }

        if(prox[i] < PRX_NOISE_THR) {
            prox[i] = 0;
        }
    }

}

void scan_target(uint16_t dist) {
    //scan
    target_lost();
    while(dist > SCAN_THR) {
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

    //leds
    clear_leds();
    spi_comm_start();

    //ToF distance
    VL53L0X_start();

    //motor
    motors_init();

    //proximity
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(FAST_UPDATE);
    calibrate_ir();

    //UART1
    serial_start();
    char str[100];
    int str_length;

    uint16_t distance = 0, sum_sensors = 0;
    int left_speed, right_speed;
    int prox[PROXIMITY_NB_CHANNELS];
    unsigned int max_idx;
    unsigned int selector = 0;

    /* Infinite loop. */
    while (1) {

        selector = get_selector();
        if (selector == 0) {
            speed_factor = 0.5;
        } else if (selector == 1) {
            speed_factor = 0.6;
        } else if (selector == 2) {
            speed_factor = 0.7;
        } else {
            speed_factor = 0.8;
        }

        distance = find_distance();

        // find the target
        if (distance > SCAN_THR) {
            scan_target(distance);
        }

        // move towards or away from the target
        motor_move();
        distance = find_distance();

        // in case target changes position
        if (distance > SCAN_THR) {
            scan_target(distance);
            // move towards or away from the target
            motor_move();
        }

        // approaching the target
        distance = find_distance();

        if(distance < TARGET_THR) {
            // target in range
            target_locked();
            motor_move();
        }

        // Consider small values to be noise thus set them to zero.
        get_prox_readings(prox, &max_idx);

        // The following table shows the weights of all the proximity sensors for the resulting rotation.
        //  Prox    0	    1	    2	    3	    4	    5	    6	    7
        //	w	   -1	   -1	   -1	   -1	    1	    1       1 	    1

        // Sum the contribution of each sensor (based on the previous weights table).
        sum_sensors = -(prox[0]) - (prox[1]) - prox[2] - prox[3] + prox[4] + prox[5] + (prox[6]) + (prox[7]);

        motor_move();

        // Modify the velocity components based on sensor values.
        left_speed = left_motor_get_desired_speed();
        right_speed = right_motor_get_desired_speed();

        left_speed -= (sum_sensors<<2);
        right_speed += ((sum_sensors<<2));

        //check bounds
        left_speed = (left_speed < -MOTOR_SPEED_LIMIT*speed_factor) ? -MOTOR_SPEED_LIMIT*speed_factor : left_speed;
        left_speed = (left_speed > MOTOR_SPEED_LIMIT*speed_factor) ? MOTOR_SPEED_LIMIT*speed_factor : left_speed;

        right_speed = (right_speed < -MOTOR_SPEED_LIMIT*speed_factor) ? -MOTOR_SPEED_LIMIT*speed_factor : right_speed;
        right_speed = (right_speed > MOTOR_SPEED_LIMIT*speed_factor) ? MOTOR_SPEED_LIMIT*speed_factor : right_speed;

        if((prox[0] > prox[7]) || (prox[1] > prox[6])) {
			go_right = true;
		} else {
			go_right = false;
		}

        if(distance < SCAN_THR){
            if(prox[1] > PRX_CLOSE_THR || prox[6] > PRX_CLOSE_THR || prox[0] > PRX_CLOSE_THR || prox[7] > PRX_CLOSE_THR) {
            	motor_move_bw();
            	chThdSleepMilliseconds(50);
            } else {
                motor_stop();
            }
        } else {
            left_motor_set_speed(left_speed);
            right_motor_set_speed(right_speed);
            chThdSleepMilliseconds(50);
        }

        str_length = sprintf(str, "distance,%d, prox,%d,%d,%d,%d,%d,%d,%d,%d, motor_speeds,%d,%d, selector, %d\n",distance,prox[0],prox[1],prox[2],
                             prox[3],prox[4],prox[5],prox[6],prox[7], left_motor_get_desired_speed(), right_motor_get_desired_speed(), get_selector());
        e_send_uart1_char(str, str_length);

        // waits 20 milliseconds
        //chThdSleepMilliseconds(20);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
