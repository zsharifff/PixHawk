/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "FlightModeManager.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using namespace time_literals;

FlightModeManager::FlightModeManager() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();

	// initialize all flight-tasks
	// currently this is required to get all parameters read
	for (int i = 0; i < static_cast<int>(FlightTaskIndex::Count); i++) {
		_initTask(static_cast<FlightTaskIndex>(i));
	}

	// disable all tasks
	_initTask(FlightTaskIndex::None);
}

FlightModeManager::~FlightModeManager()
{
	if (_current_task.task) {
		_current_task.task->~FlightTask();
	}

	perf_free(_loop_perf);
}

bool FlightModeManager::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// limit to every other vehicle_local_position update (50 Hz)
	_vehicle_local_position_sub.set_interval_us(20_ms);
	_time_stamp_last_loop = hrt_absolute_time();
	return true;
}

void FlightModeManager::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// generate setpoints on local position changes
	vehicle_local_position_s vehicle_local_position;

	if (_vehicle_local_position_sub.update(&vehicle_local_position)) {
		const hrt_abstime time_stamp_now = vehicle_local_position.timestamp_sample;
		// Guard against too small (< 0.2ms) and too large (> 100ms) dt's.
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) / 1e6f), 0.0002f, 0.1f);
		_time_stamp_last_loop = time_stamp_now;

		_home_position_sub.update();
		_vehicle_control_mode_sub.update();
		_vehicle_land_detected_sub.update();
		_vehicle_status_sub.update();
		_navigator_mission_item_sub.update();

		start_flight_task();

		if (_vehicle_command_sub.updated()) {
			handleCommand();
		}

		tryApplyCommandIfAny();

		if (isAnyTaskActive()) {
			generateTrajectorySetpoint(dt, vehicle_local_position);
		}

	}

	perf_end(_loop_perf);
}

void FlightModeManager::updateParams()
{
	ModuleParams::updateParams();

	if (isAnyTaskActive()) {
		_current_task.task->handleParameterUpdate();
	}
}

void FlightModeManager::start_flight_task()
{
	bool task_failure = false;
	bool should_disable_task = true;

	// land/rtl mode is precland
	const bool land_should_be_precland =
			(_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL ||
			_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)
			&& _param_rtl_pld_md.get() > 0;


	// Mission item precland
	const bool precland_mission_item_active = _vehicle_status_sub.get().nav_state ==
			vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION &&
			_navigator_mission_item_sub.get().nav_sub_cmd == navigator_mission_item_s::WORK_ITEM_TYPE_PRECISION_LAND;

	// When issuing the auto:precland mode
	const bool precland_flight_mode = _vehicle_status_sub.get().nav_state ==
					  vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND;

	// PX4_INFO("start_flight_task: %d %d %d", land_should_be_precland, precland_mission_item_active, precland_flight_mode);

	// Do not run any flight task for VTOLs in fixed-wing mode
	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		switchTask(FlightTaskIndex::None);
		return;
	}

	// Only run transition flight task if altitude control is enabled (e.g. in Altitdue, Position, Auto flight mode)
	if (_vehicle_status_sub.get().in_transition_mode && _vehicle_control_mode_sub.get().flag_control_altitude_enabled) {
		switchTask(FlightTaskIndex::Transition);
		return;
	}

	// Auto-follow me
	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::InvalidTask;

#if !defined(CONSTRAINED_FLASH)
		error = switchTask(FlightTaskIndex::AutoFollowTarget);
#endif // !CONSTRAINED_FLASH

		if (error != FlightTaskError::NoError) {
			task_failure = true;
		}

	} else if (land_should_be_precland || precland_mission_item_active || precland_flight_mode) {
		// Take-over landing from navigator if precision landing is enabled
		should_disable_task = false;

		if (switchTask(FlightTaskIndex::AutoPrecisionLanding) != FlightTaskError::NoError) {
			task_failure = true;
		}

	} else if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND) {

		// Emergency descend
		should_disable_task = false;

		if (switchTask(FlightTaskIndex::Descend) != FlightTaskError::NoError) {
			task_failure = true;
		}

	} else if (_vehicle_control_mode_sub.get().flag_control_auto_enabled) {
		// Auto related maneuvers
		should_disable_task = false;

		if (switchTask(FlightTaskIndex::Auto) != FlightTaskError::NoError) {
			task_failure = true;
		}

	}

	// manual position control
	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 0:
			error = switchTask(FlightTaskIndex::ManualPosition);
			break;

		case 3:
			error = switchTask(FlightTaskIndex::ManualPositionSmoothVel);
			break;

		case 4:
		default:
			if (_param_mpc_pos_mode.get() != 4) {
				PX4_ERR("MPC_POS_MODE %" PRId32 " invalid, resetting", _param_mpc_pos_mode.get());
				_param_mpc_pos_mode.set(4);
				_param_mpc_pos_mode.commit();
			}

			error = switchTask(FlightTaskIndex::ManualAcceleration);
			break;
		}

		task_failure = error != FlightTaskError::NoError;
	}

	// manual altitude control
	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 0:
			error = switchTask(FlightTaskIndex::ManualAltitude);
			break;

		case 3:
		default:
			error = switchTask(FlightTaskIndex::ManualAltitudeSmoothVel);
			break;
		}

		task_failure = error != FlightTaskError::NoError;
	}

	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT) {
		should_disable_task = false;

		if (!_command_failed) {
			FlightTaskError error = FlightTaskError::InvalidTask;

#if !defined(CONSTRAINED_FLASH)
			error = switchTask(FlightTaskIndex::Orbit);
#endif // !CONSTRAINED_FLASH

			if (error != FlightTaskError::NoError) {
				task_failure = true;
			}

		}
	}

	// check task failure
	if (task_failure) {
		// If failsafe task is activated for a prolonged time, print an error, as this is unexpected
		if (_failsafe_task_activated_start == 0) {
			_failsafe_task_activated_start = hrt_absolute_time();
		}

		if (hrt_absolute_time() - _failsafe_task_activated_start > 5_s && !_failsafe_task_error_printed) {
			PX4_ERR("Failsafe task activated");
			_failsafe_task_error_printed = true;
		}

		// for some reason no flighttask was able to start.
		// go into failsafe flighttask
		FlightTaskError error = switchTask(FlightTaskIndex::Failsafe);

		if (error != FlightTaskError::NoError) {
			// No task was activated.
			switchTask(FlightTaskIndex::None);
		}

	} else {
		_failsafe_task_activated_start = 0;
		_failsafe_task_error_printed = false;

		if (should_disable_task) {
			switchTask(FlightTaskIndex::None);
		}
	}

}

void FlightModeManager::tryApplyCommandIfAny()
{
	if (isAnyTaskActive() && _current_command.command != 0 && hrt_absolute_time() < _current_command.timestamp + 200_ms) {
		bool success = false;

		if (_current_task.task->applyCommandParameters(_current_command, success)) {
			_current_command.command = 0;

			if (!success) {
				switchTask(FlightTaskIndex::Failsafe);
				_command_failed = true;
			}
		}
	}
}

void FlightModeManager::handleCommand()
{
	// get command
	vehicle_command_s command;

	while (_vehicle_command_sub.update(&command)) {

		switch (command.command) {
		case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
			// The command might trigger a mode switch, and the mode switch can happen before or
			// after we receive the command here, so we store it for later.
			memcpy(&_current_command, &command, sizeof(vehicle_command_s));
			_command_failed = false;
			break;
		}

		if (_current_task.task) {
			// check for other commands not related to task switching
			if ((command.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED)
			    && (static_cast<uint8_t>(command.param1 + .5f) == vehicle_command_s::SPEED_TYPE_GROUNDSPEED)
			    && (command.param2 > 0.f)) {
				_current_task.task->overrideCruiseSpeed(command.param2);
			}
		}
	}
}

void FlightModeManager::generateTrajectorySetpoint(const float dt,
		const vehicle_local_position_s &vehicle_local_position)
{
	// If the task fails sned out empty NAN setpoints and the controller will emergency failsafe
	trajectory_setpoint_s setpoint = FlightTask::empty_trajectory_setpoint;
	vehicle_constraints_s constraints = FlightTask::empty_constraints;

	if (_current_task.task->updateInitialize() && _current_task.task->update()) {
		// setpoints and constraints for the position controller from flighttask
		setpoint = _current_task.task->getTrajectorySetpoint();
		constraints = _current_task.task->getConstraints();
	}

	// limit altitude according to land detector
	limitAltitude(setpoint, vehicle_local_position);

	if (_takeoff_status_sub.updated()) {
		takeoff_status_s takeoff_status;

		if (_takeoff_status_sub.copy(&takeoff_status)) {
			_takeoff_state = takeoff_status.takeoff_state;
		}
	}

	if (_takeoff_state < takeoff_status_s::TAKEOFF_STATE_RAMPUP) {
		// reactivate the task which will reset the setpoint to current state
		_current_task.task->reActivate();
	}


	setpoint.timestamp = hrt_absolute_time();
	_trajectory_setpoint_pub.publish(setpoint);

	constraints.timestamp = hrt_absolute_time();
	_vehicle_constraints_pub.publish(constraints);

	// if there's any change in landing gear setpoint publish it
	landing_gear_s landing_gear = _current_task.task->getGear();

	if (landing_gear.landing_gear != _old_landing_gear_position
	    && landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {

		landing_gear.timestamp = hrt_absolute_time();
		_landing_gear_pub.publish(landing_gear);
	}

	_old_landing_gear_position = landing_gear.landing_gear;
}

void FlightModeManager::limitAltitude(trajectory_setpoint_s &setpoint,
				      const vehicle_local_position_s &vehicle_local_position)
{
	if (_param_lndmc_alt_max.get() < 0.0f || !_home_position_sub.get().valid_alt
	    || !vehicle_local_position.z_valid || !vehicle_local_position.v_z_valid) {
		// there is no altitude limitation present or the required information not available
		return;
	}

	// maximum altitude == minimal z-value (NED)
	const float min_z = _home_position_sub.get().z + (-_param_lndmc_alt_max.get());

	if (vehicle_local_position.z < min_z) {
		// above maximum altitude, only allow downwards flight == positive vz-setpoints (NED)
		setpoint.position[2] = min_z;
		setpoint.velocity[2] = math::max(setpoint.velocity[2], 0.f);
	}
}

FlightTaskError FlightModeManager::switchTask(FlightTaskIndex new_task_index)
{
	// switch to the running task, nothing to do
	if (new_task_index == _current_task.index) {
		return FlightTaskError::NoError;
	}

	// Save current setpoints for the next FlightTask
	trajectory_setpoint_s last_setpoint = FlightTask::empty_trajectory_setpoint;
	ekf_reset_counters_s last_reset_counters{};

	if (isAnyTaskActive()) {
		last_setpoint = _current_task.task->getTrajectorySetpoint();
		last_reset_counters = _current_task.task->getResetCounters();
	}

	if (_initTask(new_task_index)) {
		// invalid task
		return FlightTaskError::InvalidTask;
	}

	if (!isAnyTaskActive()) {
		// no task running
		return FlightTaskError::NoError;
	}

	// activation failed
	if (!_current_task.task->updateInitialize() || !_current_task.task->activate(last_setpoint)) {
		_current_task.task->~FlightTask();
		_current_task.task = nullptr;
		_current_task.index = FlightTaskIndex::None;
		return FlightTaskError::ActivationFailed;
	}

	_current_task.task->setResetCounters(last_reset_counters);
	_command_failed = false;

	return FlightTaskError::NoError;
}

FlightTaskError FlightModeManager::switchTask(int new_task_index)
{
	// make sure we are in range of the enumeration before casting
	if (static_cast<int>(FlightTaskIndex::None) <= new_task_index &&
	    static_cast<int>(FlightTaskIndex::Count) > new_task_index) {
		return switchTask(FlightTaskIndex(new_task_index));
	}

	switchTask(FlightTaskIndex::None);
	return FlightTaskError::InvalidTask;
}

const char *FlightModeManager::errorToString(const FlightTaskError error)
{
	switch (error) {
	case FlightTaskError::NoError: return "No Error";

	case FlightTaskError::InvalidTask: return "Invalid Task";

	case FlightTaskError::ActivationFailed: return "Activation Failed";
	}

	return "This error is not mapped to a string or is unknown.";
}

int FlightModeManager::task_spawn(int argc, char *argv[])
{
	FlightModeManager *instance = new FlightModeManager();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FlightModeManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlightModeManager::print_status()
{
	if (isAnyTaskActive()) {
		PX4_INFO("Running, active flight task: %" PRIu32, static_cast<uint32_t>(_current_task.index));

	} else {
		PX4_INFO("Running, no flight task active");
	}

	perf_print_counter(_loop_perf);
	return 0;
}

int FlightModeManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the setpoint generation for all modes. It takes the current mode state of the vehicle as input
and outputs setpoints for controllers.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flight_mode_manager", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flight_mode_manager_main(int argc, char *argv[])
{
	return FlightModeManager::main(argc, argv);
}
