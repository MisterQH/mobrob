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
 * \file altitude_estimation.h
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief 	Altitude estimation
 *
 ******************************************************************************/


#include "altitude_estimation.h"
#include "time_keeper.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void altitude_estimation_init(altitude_estimation_t* estimator, const altitude_estimation_conf_t* config, const sonar_t* sonar, const barometer_t* barometer, const ahrs_t* ahrs, altitude_t* altitude_estimated)
{
	// Init dependencies
	estimator->sonar 				= sonar;
	estimator->barometer 			= barometer;
	estimator->ahrs 				= ahrs;
	estimator->altitude_estimated 	= altitude_estimated;

	// Init members
}

static float low_pass_filter(float x, float y_old, float deltaT, float tau)
{
	float alpha = deltaT/(deltaT+tau);
	float y = alpha*x + (1-alpha)*y_old;
	return y;
}


static float high_pass_filter(float x, float x_old, float y_old, float deltaT, float tau)
{
	float alpha = tau/(deltaT+tau);
	float y = alpha*(x-x_old) + alpha*y_old;
	return y;
}

static float accelerometer_hp_hp_integral(float x, float delta_t, float tau)
{
    static float x_1 = 0;
    static float y = 0, y_1 = 0, y_2 = 0;

	float alpha = tau/(delta_t+tau);

    y = delta_t * alpha * alpha * (x - x_1) + 2 * alpha * y_1 - alpha * alpha * y_2;

    x_1 = x;
    y_2 = y_1;
    y_1 = y;

    return y;
}

static float sonar_lp_lp_diff(float x, float delta_t, float tau)
{
    static float x_1 = 0;
    static float y = 0, y_1 = 0, y_2 = 0;

	float alpha = delta_t / (delta_t + tau);
    float alpha_comp = 1 - alpha;

    y = 1/delta_t * alpha * alpha * (x - x_1) + 2 * alpha_comp * y_1 - alpha_comp * alpha_comp * y_2;

    x_1 = x;
    y_2 = y_1;
    y_1 = y;

    return y;
}

static float sonar_lp(float x, float delta_t, float tau)
{
    static float y_1 = 0;

	float alpha = delta_t / (delta_t + tau);
	float y = alpha * x + (1-alpha) * y_1;

    y_1 = y;

	return y;
}

void altitude_estimation_update(altitude_estimation_t* estimator)
{
    float tau_altitude = 0.2f;
	float tau_rate = 0.1f;
    static uint32_t time_stamp_old = 0;

    uint32_t time_stamp = time_keeper_get_micros();

    float delta_t = (float)(time_stamp - time_stamp_old) / 1000000.0f;
    time_stamp_old = time_stamp;

    float sonar = sonar_lp(-estimator->sonar->current_distance,
                           delta_t,
                           tau_altitude);

    float sonar_rate = sonar_lp_lp_diff(-estimator->sonar->current_distance,
                                        delta_t,
                                        tau_rate);

    float accelerometer_rate = accelerometer_hp_hp_integral(estimator->ahrs->linear_acc[2],
                                                            delta_t,
                                                            tau_rate);

    estimator->altitude_estimated->rate = sonar_rate + accelerometer_rate;
    estimator->altitude_estimated->above_ground = sonar;
    estimator->altitude_estimated->above_sea = 400.0f;

}
