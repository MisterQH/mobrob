/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file tasks.c
 *
 * \author MAV'RIC Team
 *   
 * \brief Definition of the tasks executed on the autopilot
 *
 ******************************************************************************/


#include "tasks.h"
#include "central_data.h"
#include "print_util.h"
#include "stabilisation.h"
#include "gps_ublox.h"
#include "navigation.h"
#include "led.h"
#include "imu.h"
#include "delay.h"
#include "sonar_i2cxl.h"
#include "analog_monitor.h"
#include "lsm330dlc.h"
#include "hmc5883l.h"
#include "stdio_usb.h"
#include "data_logging.h"

#include "pwm_servos.h"

#include "remote.h"
#include "attitude_controller_p2.h"
#include "attitude_controller.h"
#include "altitude_estimation.h"

central_data_t* central_data;


task_set_t* tasks_get_main_taskset() 
{
	central_data = central_data_get_pointer_to_struct();

	return central_data->scheduler.task_set;
}


void tasks_run_imu_update(void* arg)
{
	if (central_data->state.mav_mode.HIL == HIL_ON)
	{
		simulation_update(&central_data->sim_model);
	} 
	else 
	{
		lsm330dlc_gyro_update(&(central_data->imu.raw_gyro));
		lsm330dlc_acc_update(&(central_data->imu.raw_accelero));
		hmc5883l_update(&(central_data->imu.raw_compass));
	}
	
	imu_update(	&central_data->imu);
	qfilter_update(&central_data->attitude_filter);
	
	if (central_data->imu.calibration_level == OFF)
	{
		position_estimation_update(&central_data->position_estimation);
	}
}


task_return_t tasks_run_stabilisation(void* arg);
task_return_t tasks_run_stabilisation(void* arg)
{
	tasks_run_imu_update(0);
	remote_update( &central_data->remote );
	
	mav_mode_t mode = central_data->state.mav_mode;

	if( mode.ARMED == ARMED_OFF )
	{
		// Set command to current heading
		central_data->command.attitude.rpy[2] = coord_conventions_quat_to_aero(central_data->ahrs.qe).rpy[2];
		servos_set_value_failsafe( &central_data->servos );
	}
	else if( mode.MANUAL == MANUAL_ON && mode.GUIDED == GUIDED_ON )
	{
		// Get manual control
		if( central_data->state.remote_active == 1 )
		{
			// Get attitude from remote
			central_data->command.attitude.mode = ATTITUDE_COMMAND_MODE_RPY;
			remote_get_attitude_command_integrate_yaw (  &central_data->remote,
														 0.01f,
														 &central_data->command.attitude );
		}
		else
		{
			// Get attitude from joystick
			joystick_parsing_get_attitude_command_integrate_yaw(&central_data->joystick_parsing, 
																0.01f,
														 		&central_data->command.attitude );
		}

		// Control attitude
		attitude_controller_update( &central_data->attitude_controller );
		
		// Estimate altitude
		altitude_estimation_update( &central_data->altitude_estimation );

		// Control altitude
		central_data->command.position.mode = POSITION_COMMAND_MODE_LOCAL;
		altitude_controller_update( &central_data->altitude_controller );

		// Mix to servos
		servos_mix_quadcopter_diag_update( &central_data->servo_mix );
	}
	else if( mode.MANUAL == MANUAL_ON && mode.STABILISE == STABILISE_ON )
	{
		// Get manual control
		if( central_data->state.remote_active == 1 )
		{
			// Get attitude and thrust from remote
			central_data->command.attitude.mode = ATTITUDE_COMMAND_MODE_RPY;
			remote_get_attitude_command_integrate_yaw (  &central_data->remote,
														 0.01f,
														 &central_data->command.attitude );
			remote_get_thrust_command(	&central_data->remote, 
										&central_data->command.thrust );
		}
		else
		{
			// Get attitude from joystick
			central_data->command.attitude.mode = ATTITUDE_COMMAND_MODE_RPY;
			joystick_parsing_get_attitude_command_integrate_yaw(&central_data->joystick_parsing, 
																0.01f,
														 		&central_data->command.attitude );
			joystick_parsing_get_thrust_command(	&central_data->joystick_parsing, 
													&central_data->command.thrust );
		}

	
		
		// Control attitude
		attitude_controller_update( &central_data->attitude_controller );
			
		// Mix to servos
		servos_mix_quadcopter_diag_update( &central_data->servo_mix );
	}
	else
	{
		servos_set_value_failsafe( &central_data->servos );
	}

	// !!! -- for safety, this should remain the only place where values are written to the servo outputs! --- !!!
	if ( mode.HIL == HIL_OFF )
	{
		pwm_servos_write_to_hardware( &central_data->servos );
	}
	
	return TASK_RUN_SUCCESS;
} 


task_return_t tasks_run_gps_update(void* arg) 
{
	if (central_data->state.mav_mode.HIL == HIL_ON)
	{
		simulation_simulate_gps(&central_data->sim_model);
	} 
	else 
	{
		gps_ublox_update(&central_data->gps);
	}
	
	return TASK_RUN_SUCCESS;
}


task_return_t tasks_run_barometer_update(void* arg)
{
	if (central_data->state.mav_mode.HIL == HIL_ON)
	{
		simulation_simulate_barometer(&central_data->sim_model);
	} 
	else
	{
		bmp085_update(&(central_data->pressure));
	}

	return TASK_RUN_SUCCESS;
}


task_return_t tasks_run_sonar_update(void* arg)
{
	if (central_data->state.mav_mode.HIL == HIL_ON)
	{
		simulation_simulate_sonar(&central_data->sim_model);
	}
	else
	{
		sonar_i2cxl_update(&(central_data->sonar_i2cxl));
	}

	return TASK_RUN_SUCCESS;
}


task_return_t tasks_led_toggle(void* arg)
{
	LED_Toggle(LED1);
	
	return TASK_RUN_SUCCESS;
}


void tasks_create_tasks() 
{	
	central_data = central_data_get_pointer_to_struct();
	
	scheduler_t* scheduler = &central_data->scheduler;


	// Main stabilisation loop
	scheduler_add_task(scheduler, 4000,	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_HIGHEST, &tasks_run_stabilisation											, 0														, 0);

	// LED
	scheduler_add_task(scheduler, 500000,	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_LOWEST , &tasks_led_toggle													, 0														, 1);
	
	// State
	scheduler_add_task(scheduler, 200000,   RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_NORMAL , (task_function_t)&state_machine_update              				, (task_argument_t)&central_data->state_machine         , 2);

	// Communication
	scheduler_add_task(scheduler, 4000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_NORMAL , (task_function_t)&mavlink_communication_update                    , (task_argument_t)&central_data->mavlink_communication , 3);

	// Remote
	scheduler_add_task(scheduler, 20000, 	RUN_REGULAR, PERIODIC_RELATIVE, PRIORITY_HIGH   , (task_function_t)&remote_update 									, (task_argument_t)&central_data->remote 				, 4);
	
	// Barometer
	scheduler_add_task(scheduler, 15000, 	RUN_REGULAR, PERIODIC_RELATIVE, PRIORITY_HIGH   , &tasks_run_barometer_update                                       , 0 													, 5);
	
	// GPS
	scheduler_add_task(scheduler, 100000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_HIGH   , &tasks_run_gps_update                                             , 0 													, 6);
	
	// Others
	scheduler_add_task(scheduler, 100000,   RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_LOW	, &tasks_run_sonar_update											, 0														, 7);
	
	// Waypoint handler
	scheduler_add_task(scheduler, 10000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_LOW    , (task_function_t)&waypoint_handler_control_time_out_waypoint_msg  , (task_argument_t)&central_data->waypoint_handler 		, 8);
	
	// Navigation
	// scheduler_add_task(scheduler, 10000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_HIGH   , (task_function_t)&navigation_update                               , (task_argument_t)&central_data->navigation 			, 9);
	
	// Analog monitor
	scheduler_add_task(scheduler, 100000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_LOW    , (task_function_t)&analog_monitor_update                           , (task_argument_t)&central_data->analog_monitor 		, 10);
	
	// Data logging
	// scheduler_add_task(scheduler, 100000,   RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_LOW	, (task_function_t)&data_logging_update								, (task_argument_t)&central_data->data_logging			, 11);
		
	// Sort tasks by priority and update frequency
	scheduler_sort_tasks(scheduler);
}
