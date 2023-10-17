#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "sensors/VL53L0X/VL53L0X.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    // Your initialisations here
    VL53L0X_start();

    //UART1
    serial_start();
    char str[100];
    int str_length;

    uint16_t distance = 0;
    uint32_t dSum = 0;
    unsigned int N = 10;

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(20);

        //reset the sum
        dSum = 0;

        // simple moving average
        //for (unsigned int p = 0; p < N; p++) {
        for (unsigned int n = 0; n < N; n++) {
            //ToF distance sensor
            dSum += VL53L0X_get_dist_mm();
            chThdSleepMilliseconds(5);
        }
        
        distance = dSum/N;
        //}

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
