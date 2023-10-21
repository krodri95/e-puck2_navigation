#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//The header files for UART
#include "epuck1x/uart/e_uart_char.h"
#include "stdio.h"
#include "serial_comm.h"

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

 /**
 * @brief   Returns the distance value of the detected object
 * 
 * @return					Distance measured by the ToF sensor
 */
uint16_t find_distance(void);

 /**
 * @brief   Sets the motor speed to 0
 */
void motor_stop(void);

 /**
 * @brief   Sets the motor speed to the maximum in the forward direction
 */
void motor_move_fw(void);

 /**
 * @brief   Sets the motor speed to the maximum in the reverse direction
 */
void motor_move_bw(void);

 /**
 * @brief   Controls the movement (forward, backward and stop) of the motor based on distance thresholds
 */
void motor_move(void);

 /**
 * @brief   Sets the RGB leds to yellow if it finds the target
 */
void target_found(void);

 /**
 * @brief   Sets the RGB leds to green if it locks on the target
 */
void target_locked(void);

 /**
 * @brief   Sets the RGB leds to red if it loses the target and starts rotating to robot, preparing to scan
 */
void target_lost(void);

 /**
 * @brief   Gets the average reading from all the 8 proximity sensors
 * 
 * @param cal_prox pointer to the array storing the proximity sensor readings
 */
void get_cal_prox_readings(int *cal_prox);

 /**
 * @brief   Stays in an infinite loop until the target is found
 * 
 * @param dist	distance reading from the ToF sensor
 * 
 * @return					Last ambiant light value measured by the sensor
 */
void scan_target(uint16_t dist);

#ifdef __cplusplus
}
#endif

#endif
