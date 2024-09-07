/*
 * File:          obstacle_avoid.c
 * Date:          23/10/23
 * Description:  
 * Author:        Andrew Prince
 * Modifications:
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stddef.h> 
#include <time.h>
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

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar); 

#define MAX_MOTOR_SPEED 1000
#define DISTANCE_THRESHOLD 50
#define SCAN_THRESHOLD 150
#define PI 3.14159

static int proximity_sensors_values[PROXIMITY_NB_CHANNELS];
double tof_sensor_weight = 2;

static float speed_factor =  0.7;

#define TOF_THRESHOLD 80

static int rotate_speed = 500;

static double speed_weights[8][2] = { {-3,3}, {-2.5,2.5}, {-2.0,2.0}, {5 ,5}, {5,5}, {2.0,-2.0}, {2.5,-2.5},{3,-3} };
static double motor_speed[2];

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

void led_yellow(void) {
    // light the leds yellow
    set_rgb_led(LED2, 10, 10, 0);
    set_rgb_led(LED4, 10, 10, 0);
    set_rgb_led(LED6, 10, 10, 0);
    set_rgb_led(LED8, 10, 10, 0);

}

void led_green(void) {
    // light the leds green
    set_rgb_led(LED2, 0, 10, 0);
    set_rgb_led(LED4, 0, 10, 0);
    set_rgb_led(LED6, 0, 10, 0);
    set_rgb_led(LED8, 0, 10, 0);

}

void led_red(void) {
    // light the leds red
    set_rgb_led(LED2, 10, 0, 0);
    set_rgb_led(LED4, 10, 0, 0);
    set_rgb_led(LED6, 10, 0, 0);
    set_rgb_led(LED8, 10, 0, 0);
}

static void get_proximity_sensor_values()
{
    for (int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
        proximity_sensors_values[i] = 0;
        for (unsigned int j = 0; j < PROXIMITY_NB_CHANNELS; j++) {
            proximity_sensors_values[i] += get_prox(i);
        }
    } 
    //Averaging sensor reading
    for (unsigned int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
        proximity_sensors_values[i] /= PROXIMITY_NB_CHANNELS;
    }
}

//Checks the presence of an obstacle in the front
static bool object_near() {

    //in case of corners
    uint16_t dist = find_distance();
    if (dist < DISTANCE_THRESHOLD) {
        return true;
    } else {
        return false;
    }
}

int max_index()
{
    unsigned int idx = 0;
    int lVal = 0;
    for(unsigned int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
        if(proximity_sensors_values[i] > lVal) {
            lVal = proximity_sensors_values[i];
            idx = i;
        }
    }
    return idx;
}

static void wait_time(double sec)
{
   systime_t start_time = chVTGetSystemTimeX();
    do {
        //nothing
    } while (sec > ST2MS(chVTGetSystemTimeX() - start_time)/1000);

	//chThdSleepMilliseconds(sec*1000);
}

static void turn_left(double sec)
{
    left_motor_set_speed(-rotate_speed);
    right_motor_set_speed(rotate_speed);
    wait_time(sec);
}

static void turn_right(double sec)
{
    left_motor_set_speed(rotate_speed);
    right_motor_set_speed(-rotate_speed);
    wait_time(sec);
}

static void go_back(double sec)
{
    left_motor_set_speed(-rotate_speed);
    right_motor_set_speed(-rotate_speed);
    wait_time(sec);
}

static bool cornered()
{
	int count = 0;
	int prox_sensor_number[6] = {0, 1, 2, 5, 6, 7};
	for(int i = 0; i < sizeof(prox_sensor_number); i++) {
		if(proximity_sensors_values[prox_sensor_number[i]] > SCAN_THRESHOLD) {
			count++;
		}
	}

	if(count > sizeof(prox_sensor_number) - 1) {
		return true;
    } else {
		return false;
    }
}

static void scan_distance()
{
    left_motor_set_speed(rotate_speed);
    right_motor_set_speed(-rotate_speed);

    uint16_t tof_sensor_value = find_distance();
	while(tof_sensor_value < TOF_THRESHOLD)
	{
        tof_sensor_value = find_distance();
	}

    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

static void avoid_object()
{
    led_red();
    double angle = rand() % 3 * 30;
    int idx = max_index();

    if(cornered()) {
    	scan_distance();
    } else {
        if (idx == 0 || idx == 1 || idx == 2) {
            turn_left((angle * (PI / 180)) / rotate_speed);
        } else if (idx == 5 || idx == 6 || idx == 7) {
            turn_right((angle * (PI / 180)) / rotate_speed);
        } else {
            //to do
        }
    }
}

void movement_handler()
{
    for (int j = 0; j < 2; j++) {
        motor_speed[j] = 0;
        for (int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
            motor_speed[j] += (MAX_MOTOR_SPEED * speed_factor * speed_weights[i][j] * (proximity_sensors_values[i]));
        }

        motor_speed[j] = (motor_speed[j] < -MAX_MOTOR_SPEED * speed_factor) ? -MAX_MOTOR_SPEED * speed_factor : motor_speed[j];
        motor_speed[j] = (motor_speed[j] >  MAX_MOTOR_SPEED * speed_factor) ?  MAX_MOTOR_SPEED * speed_factor : motor_speed[j];
    }
    left_motor_set_speed(motor_speed[0]);
    right_motor_set_speed(motor_speed[1]);

    if(abs(motor_speed[0] - motor_speed[1]) < 300) {
        led_green();
    } else {
        led_yellow();
    }
}

int main() {

    halInit();
    chSysInit();
    mpu_init();

    clear_leds();
    spi_comm_start();
    VL53L0X_start();
    motors_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(0);

    //UART1
    serial_start();
    char str[100];
    int str_length;

    unsigned int selector = 0;

    while (1) {

        selector = get_selector();
        if (selector == 0) {
            speed_factor = 0.4;
        } else if (selector == 1) {
            speed_factor = 0.5;
        } else if (selector == 2) {
            speed_factor = 0.6;
        } else {
            speed_factor = 0.7;
        }

        get_proximity_sensor_values();
        str_length = sprintf(str, " cal_prox,%d,%d,%d,%d,%d,%d,%d,%d\n", proximity_sensors_values[0],proximity_sensors_values[1],proximity_sensors_values[2],
        		proximity_sensors_values[3],proximity_sensors_values[4],proximity_sensors_values[5],proximity_sensors_values[6],proximity_sensors_values[7]);
        e_send_uart1_char(str, str_length);
        if (object_near()) {
            avoid_object();
        } else {
            movement_handler();    
        }
        chThdSleepMilliseconds(50);
    };
    return 0;
} 

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

