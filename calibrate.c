#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"

#define PRX_NOISE_THR 0

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

uint16_t find_distance(void) {

    uint32_t pSum = 0, nSum = 0;;
    unsigned int N = 10;

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

void get_cal_prox_readings(int *cal_prox, unsigned int *mIdx) {

    unsigned int N = 50;

    // averaging
    for (unsigned int n = 0; n < N; n++) {
        //sensors 0-7
        for(unsigned int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
            cal_prox[i] += get_calibrated_prox(i);
        }
        chThdSleepMilliseconds(1);
    }
    
    int lVal = 0;
    for(unsigned int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
        cal_prox[i] /= N;

        if(cal_prox[i] > lVal) {
            lVal = cal_prox[i];
            *mIdx = i;
        }

        if(cal_prox[i] < PRX_NOISE_THR) {
            cal_prox[i] = 0;
        }
    }

}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    // Your initialisations here
    VL53L0X_start();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(FAST_UPDATE);
    calibrate_ir();

    //UART1
    serial_start();
    char str[100];
    int str_length;

    int cal_prox[PROXIMITY_NB_CHANNELS];

    uint16_t distance, sum_sensors;
    unsigned int max_idx;

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        //chThdSleepMilliseconds(20);

        distance = find_distance();
        get_cal_prox_readings(cal_prox, &max_idx);

        // Sum the contribution of each sensor (based on the previous weights table).
        sum_sensors = -(cal_prox[0]>>2) - (cal_prox[1]>>1) - cal_prox[2] - cal_prox[3] + cal_prox[4] + cal_prox[5] + (cal_prox[6]>>1) + (cal_prox[7]>>2);

        str_length = sprintf(str, "dist,%d\n",distance);
        e_send_uart1_char(str, str_length);
        
        str_length = sprintf(str, "max index,%d\n",max_idx);
        e_send_uart1_char(str, str_length);

        str_length = sprintf(str, "cal_prox,%d,%d,%d,%d,%d,%d,%d,%d\n",cal_prox[0],cal_prox[1],cal_prox[2],
                             cal_prox[3],cal_prox[4],cal_prox[5],cal_prox[6],cal_prox[7]);
        e_send_uart1_char(str, str_length);

        str_length = sprintf(str, "sum_sensors,%d\n",sum_sensors);
        e_send_uart1_char(str, str_length);

        str_length = sprintf(str, "---------------------------------\n");
        e_send_uart1_char(str, str_length);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
