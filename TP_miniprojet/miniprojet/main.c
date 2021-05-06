#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <i2c_bus.h>
#include <leds.h>
#include <usbcfg.h>
#include <motors.h>
#include <chprintf.h>

#include "main.h"
#include "compute_imu_data.h"
#include "pid_regulator.h"
#include "estimator.h"
#include "proximity_sensor.h"
#include "send_data.h"


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


int main(void)
{
  reg_param_t reg_param;

  reg_param.kp = 0.0;
  reg_param.kd = 0.0;
  reg_param.ki = 0.0;

  halInit();
  chSysInit();

  mpu_init();
  //inits the motors
  motors_init();

  //starts the serial communication
  serial_start();
  //start the USB communication
  usb_start();

  //starts the accelerometer
  imu_start();

  // starts the proximity sensors
  proximity_start();

  // Inits the Inter Process Communication bus
  messagebus_init(&bus, &bus_lock, &bus_condvar);

  // IMU calibration
  set_front_led(1);
  calibrate_gyro();
  calibrate_acc();
  set_front_led(0);

  // Calibration of the ambient IR intensity
  calibrate_ir();

  // Start to compute the datas from the IR sensors
  prox_compute_start();

  //stars the threads for the pi regulator
  // auto_regulator_start(&reg_param);
  estimator_start();
  pi_regulator_start(&reg_param);

  /* Infinite loop. */
  while (1) {


	  /*
	   *
	   *
	   * DO NOTHING
	   *
	   *
	   */


    //waits 1 second
    chThdSleepMilliseconds(1000);
  }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
  chSysHalt("Stack smashing detected");
}
