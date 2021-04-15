#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "sensors/imu.h"
#include "i2c_bus.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <process_image.h>

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

	// Inits the Inter Process Communication bus
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	imu_msg_t imu_values;

	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

    /* Infinite loop. */
    while (1) {
    	// Wait for new measures to be published on the i2C bus
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

    	chprintf((BaseSequentialStream *)&SD3, "Ax = %-7d", imu_values.acc_raw[X_AXIS]);

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
