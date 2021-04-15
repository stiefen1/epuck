#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static float distance_cm = 0;
static float position_pxl = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	po8030_set_awb(0);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
	systime_t time;

    while(1){
        //time = chVTGetSystemTime();
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();

		//chThdSleepMilliseconds(12);

		//time = chVTGetSystemTime() - time;
        //chprintf((BaseSequentialStream *)&SD3, "time = %d", time);

		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	uint16_t width = 0;
	uint8_t counter = 0; // Permet de ne pas lire une image à chaque cycle
	uint16_t stop_index = 0;
	uint16_t start_index = 0;
	double distance = 0;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		if(counter>=2) // Limits number of images/s
		{
			for(int i = 0 ; i < IMAGE_BUFFER_SIZE ; ++i)
			{
				// Read image datas
				image[i] = (((img_buff_ptr[2*i]) & 0b11111000) >> 3);

				// Search start and end of the black line
				if(i>=10)
				{
					/*
					 * Pour chaque pixel, on regarde si la valeur
					 * 10 pixel plus loin est différente d'au moins 8 (intensité)
					 * Si c'est le cas, on définit que c'est un flanc
					 * Le signe de la différence définit si c'est un flanc
					 * montant ou descendant
					 * L'information est contenue dans les variables start_index
					 * (pixel de début) et stop_index (pixel de fin)
					 */
					if((image[i-10] - image[i]) > 8)
						start_index = i;

					else if((image[i] - image[i-10] > 8))
						stop_index = i;
				}
			}

			// Compute line width
			width = stop_index - start_index;
			// Compute center position
			position_pxl = (width / 2 + start_index);

			/* Vérifie si les valeurs de l'image sont proches de 0
			 * dans la zone détecté de la ligne. Si c'est le cas, on calcule
			 * la distance en cm, sinon on pose la distance_cm = 0.
			 * Cette valeur sera utile dans le thread du régulateur
			 * pour lui faire comprendre que la ligne n'a pas été détectée
			 */
			if(image[start_index + 10] < 8 && image[stop_index - 10] < 8)
				distance_cm = width*width*0.0002 - 0.1628*width + 29.42;
				/* Approximation quadratique de la distance en fonction de l'épaisseur
				 * détectée de la ligne
				 */

			else
				distance_cm = 0;

			/* distance_cm est une variable globale statique
			 * La fonction get_distance_cm() permet juste d'accéder
			 * à cette variable depuis d'autres fichier.c (régulateur)
			 */
			distance = get_distance_cm();

			//chprintf((BaseSequentialStream *) &SD3, "distance = %f ", distance);
			//SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
			counter = 0;
		}
		else
			counter++;
    }
}

float get_distance_cm(void){
	return distance_cm;
}

float get_position_pxl(void){
	return position_pxl;
}

void image_filter(void)
{

}

uint8_t get_width()
{
	uint8_t width = 0;




//
//		// Save the index of max derivative point (2nd point)
//		if(derivative[i] > derivative[max_derivative_index])
//			max_derivative_index = i;
//
//		// Save the index of min derivative point (1st point)
//		else if(derivative[i] < derivative[min_derivative_index])
//			min_derivative_index = i;
//	}

	width = 0;

	return width;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
