/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/***********************************/
/* VL53L8CX ULD interrupt checkers */
/***********************************/
/*
* This example shows the possibility of VL53L8CX to program interrupt checkers. It
* initializes the VL53L8CX ULD, create 2 checkers per zone for a 4x4 resolution,
* and starts a ranging to capture 10 frames.

* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l8cx_api.h"
#include "vl53l8cx_plugin_detection_thresholds.h"


int example9(VL53L8CX_Configuration *p_dev)
{

	/*********************************/
	/*   VL53L8CX ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i;
	VL53L8CX_ResultsData 	Results;		/* Results data from VL53L8CX */

	
	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L8CX sensor connected */
	status = vl53l8cx_is_alive(p_dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L8CX not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L8CX sensor */
	status = vl53l8cx_init(p_dev);
	if(status)
	{
		printf("VL53L8CX ULD Loading failed\n");
		return status;
	}

	printf("VL53L8CX ULD ready ! (Version : %s)\n",
			VL53L8CX_API_REVISION);
			
	/*********************************/
	/*  Program interrupt checkers   */
	/*********************************/

	/* In this example, we want 2 checkers per zone for a 4x4 resolution */
	/* Create array of interrupts (size cannot be changed) */
	VL53L8CX_DetectionThresholds thresholds[VL53L8CX_NB_THRESHOLDS];

	/* Set all values to 0 */
	memset(&thresholds, 0, sizeof(thresholds));

	/* Add thresholds for all zones (16 zones in resolution 4x4, or 64 in 8x8) */
	for(i = 0; i < 16; i++){
		/* The first wanted thresholds is GREATER_THAN mode. Please note that the
		 * first one must always be set with a mathematic_operation
		 * VL53L8CX_OPERATION_NONE.
		 * For this example, the signal thresholds is set to 150 kcps/spads
		 * (the format is automatically updated inside driver)
		 */
		thresholds[2*i].zone_num = i;
		thresholds[2*i].measurement = VL53L8CX_SIGNAL_PER_SPAD_KCPS;
		thresholds[2*i].type = VL53L8CX_GREATER_THAN_MAX_CHECKER;
		thresholds[2*i].mathematic_operation = VL53L8CX_OPERATION_NONE;
		thresholds[2*i].param_low_thresh = 150;
		thresholds[2*i].param_high_thresh = 150;

		/* The second wanted checker is IN_WINDOW mode. We will set a
		 * mathematical thresholds VL53L8CX_OPERATION_OR, to add the previous
		 * checker to this one.
		 * For this example, distance thresholds are set between 200mm and
		 * 400mm (the format is automatically updated inside driver).
		 */
		thresholds[2*i+1].zone_num = i;
		thresholds[2*i+1].measurement = VL53L8CX_DISTANCE_MM;
		thresholds[2*i+1].type = VL53L8CX_IN_WINDOW;
		thresholds[2*i+1].mathematic_operation = VL53L8CX_OPERATION_OR;
		thresholds[2*i+1].param_low_thresh = 200;
		thresholds[2*i+1].param_high_thresh = 400;
	}

	/* The last thresholds must be clearly indicated. As we have 32
	 * checkers (16 zones x 2), the last one is the 31 */
	thresholds[31].zone_num = VL53L8CX_LAST_THRESHOLD | thresholds[31].zone_num;

	/* Send array of thresholds to the sensor */
	vl53l8cx_set_detection_thresholds(p_dev, thresholds);

	/* Enable detection thresholds */
	vl53l8cx_set_detection_thresholds_enable(p_dev, 1);

	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l8cx_set_ranging_frequency_hz(p_dev, 10);
	status = vl53l8cx_start_ranging(p_dev);
	printf("Put an object between 200mm and 400mm to catch an interrupt (or a reflective target where you want)\n");

	loop = 0;
	while(loop < 100)
	{
		/* The function allows catching the interrupt raised on
		 * pin A3 (INT), when the checkers detect the programmed
		 * conditions.
		 */

		isReady = VL53L8CX_wait_for_dataready(&p_dev->platform);
		if(isReady)
		{
			vl53l8cx_get_ranging_data(p_dev, &Results);

				/* As the sensor is set in 4x4 mode by default, we have a total
				 * of 16 zones to print. For this example, only the data of
				 * first zone are print */
			printf("Print data no : %3u\n", p_dev->streamcount);
			for(i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm, Signal : %5d kcps/SPADs\n",
					i,
					Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE*i],
					Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i],
					(int)Results.signal_per_spad[VL53L8CX_NB_TARGET_PER_ZONE*i]);
			}
			printf("\n");
			loop++;
		}
	}

	status = vl53l8cx_stop_ranging(p_dev);
	printf("End of ULD demo\n");
	return status;
}
