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
 * \file altitude_estimation_telemetry.c
 *
 * \author MAV'RIC Team
 *
 * \brief This module takes care of sending periodic telemetric messages from altitude estimation
 *
 ******************************************************************************/


#include "altitude_estimation_telemetry.h"
#include "time_keeper.h"
#include "central_data.h"

void altitude_estimation_telemetry_send(const altitude_estimation_t* altitude_estimation,
										const mavlink_stream_t* mavlink_stream,
										mavlink_message_t* msg)
{
	mavlink_msg_hil_sensor_pack( 	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,
									time_keeper_get_micros(),
									altitude_estimation->ahrs->linear_acc[0],			// float xacc,
									altitude_estimation->ahrs->linear_acc[1],			// float yacc,
									altitude_estimation->ahrs->linear_acc[2],			// float zacc,
									pos_pid_out,	// float xgyro,
									vel_pid_out,	// float ygyro,
									altitude_estimation->sonar->current_distance,		// float zgyro,
									0.0f,												// float xmag,
									altitude_estimation->barometer->vario_vz,			// float ymag,
									altitude_estimation->barometer->altitude,			// float zmag,
									altitude_estimation->altitude_estimated->above_sea,		// float abs_pressure,
									altitude_estimation->altitude_estimated->above_ground,	// float diff_pressure,
									altitude_estimation->altitude_estimated->rate,			// float pressure_alt,
									0.0f,												// float temperature,
									0		);											//uint32_t fields_updated);


}
