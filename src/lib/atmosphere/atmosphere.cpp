/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file atmosphere.cpp
 *
 */

#include <geo/geo.h>
#include "atmosphere.h"

static constexpr float kTempRefKelvin = 15.f - CONSTANTS_ABSOLUTE_NULL_CELSIUS; // temperature at base height in Kelvin
static constexpr float kTempGradient = -6.5f / 1000.f; // temperature gradient in degrees per meter
static constexpr float kPressRefSeaLevelPa = 101325.f; // pressure at sea level in kPa

float getDensityFromPressureAndTemp(const float pressure_pa, const float temperature_celsius)
{
	return (pressure_pa / (CONSTANTS_AIR_GAS_CONST * (temperature_celsius - CONSTANTS_ABSOLUTE_NULL_CELSIUS)));
}
float getPressureFromAltitude(const float altitude_m)
{

	return kPressRefSeaLevelPa * powf((altitude_m * kTempGradient + kTempRefKelvin) / kTempRefKelvin,
					  -CONSTANTS_ONE_G / (kTempGradient * CONSTANTS_AIR_GAS_CONST));
}
float getAltitudeFromPressure(float pressure_pa, float pressure_sealevel_pa)
{
	// calculate altitude using the hypsometric equation

	const float pressure_ratio = pressure_pa / pressure_sealevel_pa;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	return (((powf(pressure_ratio, (-(kTempGradient * CONSTANTS_AIR_GAS_CONST) / CONSTANTS_ONE_G))) * kTempRefKelvin) -
		kTempRefKelvin) / kTempGradient;

}
