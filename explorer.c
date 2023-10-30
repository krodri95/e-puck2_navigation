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

#include "spi_comm.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"
#include "motors.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar); 

#define MAX_MOTOR_SPEED 800
#define PROXIMITY_SENSORS_NUMBER 8
#define DISTANCE_THRESHOLD 600
#define PI 3.14159

static int proximity_sensors_values[PROXIMITY_SENSORS_NUMBER];

static double tof_sensor_value;
double tof_sensor_weight = 2;

#define TOF_THRESHOLD 70

static int rotate_speed = 500;
static double speed_weights[8][2] = { {-1,1}, {-0.5,0.5}, {-0.5,0.5}, {5 ,5}, {5,5}, {0.5,-0.5}, {0.5,-0.5},{1,-1} };
static double motor_speed[2];


static void get_proximity_sensor_values()
{
    for (int i = 0; i < PROXIMITY_SENSORS_NUMBER; i++)
    {
        for (unsigned int j = 0; j < PROXIMITY_SENSORS_NUMBER; j++)  
        {
            proximity_sensors_values[i] = get_prox(i);
        }

    } 
    //Averaging sensor reading
    for (unsigned int i = 0; i < PROXIMITY_SENSORS_NUMBER; i++)  
    {
        proximity_sensors_values[i] /= PROXIMITY_SENSORS_NUMBER;

    }
    //tof_sensor_value = wb_distance_sensor_get_value(tof_sensor);
}

//Detect object
static bool object_near()
{
    for (int i = 0; i < PROXIMITY_SENSORS_NUMBER; i++)
    {
        if (proximity_sensors_values[i] > DISTANCE_THRESHOLD)
        {
            return true;
        }

        else
            return false;
    }
}



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

int max_index()
{
    int idx;
    for (int i = 0; i < PROXIMITY_SENSORS_NUMBER - 1; i++)
    {
        if (proximity_sensors_values[i] > proximity_sensors_values[i + 1])
        {
            idx = i;
        }
        else
            idx = i + 1;
    }
    return idx;
}

static void wait_time(double sec)
{
   systime_t start_time = chVTGetSystemTimeX();
    do {

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

static void avoid_object()
{

    double angle = rand() % 12 * 30 * (PI / 180);
    int rand_direction = rand() % 2;

    if (angle > PI)
        angle = 2*PI - angle;
        turn_left(angle / rotate_speed);
    else
        turn_right(angle / rotate_speed);
}


void movement_handler()
{
    for (int j = 0; j < 2; j++)
    {
        motor_speed[j] = 0;
        for (int i = 0; i < PROXIMITY_SENSORS_NUMBER; i++)
        {
            motor_speed[j] += (MAX_MOTOR_SPEED * speed_weights[i][j] * (proximity_sensors_values[i]));
        }
        if (motor_speed[j] > MAX_MOTOR_SPEED)
            motor_speed[j] = MAX_MOTOR_SPEED;
        if (motor_speed[j] < -MAX_MOTOR_SPEED)
            motor_speed[j] = -MAX_MOTOR_SPEED;
    }
    left_motor_set_speed(motor_speed[0]);
    right_motor_set_speed(motor_speed[1]);

}

int main() {

    halInit();
    chSysInit();
    mpu_init();

    //clear_leds();
    spi_comm_start();
    VL53L0X_start();
    motors_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(0);


    //UART1
    serial_start();
    char str[100];
    int str_length;

    
    while (1) {

        get_proximity_sensor_values();
        str_length = sprintf(str, " cal_prox,%d,%d,%d,%d,%d,%d,%d,%d\n", proximity_sensors_values[0],proximity_sensors_values[1],proximity_sensors_values[2],
        		proximity_sensors_values[3],proximity_sensors_values[4],proximity_sensors_values[5],proximity_sensors_values[6],proximity_sensors_values[7]);
        e_send_uart1_char(str, str_length);
        if (object_near())
        {
            avoid_object();
        }
        else
        {
            movement_handler();    
        }
        chThdSleepMilliseconds(100);
    };
    return 0;
} 

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

