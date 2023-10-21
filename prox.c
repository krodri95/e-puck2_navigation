#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "sensors/proximity.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    // Your initialisations here
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(FAST_UPDATE);
    calibrate_ir();

    //UART1
    serial_start();
    char str[100];
    int str_length;

    int prox[PROXIMITY_NB_CHANNELS];
    int cal_prox[PROXIMITY_NB_CHANNELS];
    int amb_light[PROXIMITY_NB_CHANNELS];

    unsigned int N = 10;

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(20);

        //reset the arrays
        memset(prox, 0, sizeof(prox));
        memset(cal_prox, 0, sizeof(cal_prox));
        memset(amb_light, 0, sizeof(amb_light));

        // simple moving average
        //for (unsigned int p = 0; p < N; p++) {
        for (unsigned int n = 0; n < N; n++) {
            //sensors 0-7
            for(unsigned int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
                prox[i] += get_prox(i);
                cal_prox[i] += get_calibrated_prox(i);
                amb_light[i] += get_ambient_light(i);
            }
            chThdSleepMilliseconds(5);
        }
        
        for(unsigned int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
            prox[i] /= N;
            cal_prox[i] /= N;
            amb_light[i] /= N;
        }
        //}

        str_length = sprintf(str, "prox,%d,%d,%d,%d,%d,%d,%d,%d\n",prox[0],prox[1],prox[2],prox[3],
                             prox[4],prox[5],prox[6],prox[7]);
        e_send_uart1_char(str, str_length);

        str_length = sprintf(str, "cal_prox,%d,%d,%d,%d,%d,%d,%d,%d\n",cal_prox[0],cal_prox[1],cal_prox[2],
                             cal_prox[3],cal_prox[4],cal_prox[5],cal_prox[6],cal_prox[7]);
        e_send_uart1_char(str, str_length);

        str_length = sprintf(str, "amb_light,%d,%d,%d,%d,%d,%d,%d,%d\n",amb_light[0],amb_light[1],amb_light[2],
                             amb_light[3],amb_light[4],amb_light[5],amb_light[6],amb_light[7]);
        e_send_uart1_char(str, str_length);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
