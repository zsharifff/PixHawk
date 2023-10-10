/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file atmosphere.h
 *
 */

#ifndef PX4_SRC_LIB_ATMOSPHERE_ATMOSPHERE_H_
#define PX4_SRC_LIB_ATMOSPHERE_ATMOSPHERE_H_

/**
* Calculate air density given air pressure and temperature.
*
* @param pressure_pa ambient pressure in Pa
* @param temperature_celsius ambient temperature in degrees Celsius
*/
float getDensityFromPressureAndTemp(const float pressure_pa, const float temperature_celsius);

/**
* Calculate standard airpressure given altitude.
*
* @param altitude_m altitude above MSL in meters in the standard atmosphere
*/
float getPressureFromAltitude(const float altitude_m);

/**
* Calculate altitude from air pressure and temperature.
*
* @param pressure_pa ambient pressure in Pa
* @param pressure_sealevel_pa sea level pressure in Pa
*/
float getAltitudeFromPressure(float pressure_pa, float pressure_sealevel_pa);
#endif //PX4_SRC_LIB_ATMOSPHERE_ATMOSPHERE_H_
