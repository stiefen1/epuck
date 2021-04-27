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
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include "pi_regulator.h"
#include "auto_regulator.h"
#include "compute_data.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *) &SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *) &SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *) &SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
  RegParam reg_param;
  float data[2];

  reg_param.kp = -3.0;
  reg_param.kd = -200.0;
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
  //starts the camera
  dcmi_start();
  po8030_start();

  // starts the i2c communication
  //i2c_start(); Already done in the imu_start() function


  //starts the accelerometer
  imu_start();

  // starts the proximity sensors
  proximity_start();

  // init LED
  // set_front_led(1);


  // Inits the Inter Process Communication bus
  messagebus_init(&bus, &bus_lock, &bus_condvar);

  //messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
  //imu_msg_t imu_values;

  //messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
  //proximity_msg_t prox_values;

  //stars the threads for the pi regulator
  auto_regulator_start(&reg_param);

  /* Infinite loop. */
  while (1) {
    // Wait for new measures to be published on the i2C bus
    //messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));


    //messagebus_topic_wait(proximity_topic, &prox_values, sizeof(prox_values));

    //chprintf((BaseSequentialStream *)&SD3, "prox = %d", get_prox(0));

    //waits 1 second
    chThdSleepMilliseconds(100);
  }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
  chSysHalt("Stack smashing detected");
}
