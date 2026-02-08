/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file RoverCollisionPrevention.cpp
 *
 * Collision prevention implementation for ground rovers.
 *
 * @author PXLABS
 */

#include "RoverCollisionPrevention.hpp"

using namespace matrix;

RoverCollisionPrevention::RoverCollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{
	// Initialize obstacle distances to max range
	for (int i = 0; i < BIN_COUNT; i++) {
		_obstacle_distances[i] = UINT16_MAX;
		_data_timestamps[i] = 0;
		_data_maxranges[i] = UINT16_MAX;
		_data_fov[i] = 0;
	}
}

bool RoverCollisionPrevention::isActive()
{
	// Collision prevention is active if minimum distance parameter is positive
	const bool active = _param_cp_dist.get() > 0.f;

	if (active && !_was_active) {
		_time_activated = hrt_absolute_time();
	}

	_was_active = active;
	return active;
}

float RoverCollisionPrevention::modifySpeedSetpoint(float speed_setpoint, float vehicle_yaw)
{
	(void)vehicle_yaw;

	if (!isActive()) {
		return speed_setpoint;
	}

	const hrt_abstime now = hrt_absolute_time();

	// Update obstacle data from sensors
	_updateObstacleData();

	// Only limit forward speed (positive speed setpoint)
	if (speed_setpoint <= 0.f) {
		// TODO: Could add rear obstacle detection for reverse
		return speed_setpoint;
	}

	// Get closest obstacle distance in front sector
	float obstacle_distance = getObstacleDistanceFront();

	// Check if we have valid data in the commanded forward sector.
	// Match multicopter semantics: CP_GO_NO_DATA allows unknown areas, but not timed-out bins inside known FOV.
	if (!_front_obstacle_data_present && (!_param_cp_go_no_data.get() || _front_data_timed_out_in_fov)) {
		// No obstacle data and not allowed to move without data
		mavlink_log_warning(&_mavlink_log_pub, "Rover collision prevention: no sensor data\t");

		// If no data for a prolonged time, command hold/loiter
		if ((now - _last_timeout_warning) > 1_s && (now - _time_activated) > 1_s) {
			if ((now - _last_data_time) > TIMEOUT_HOLD_US && (now - _time_activated) > TIMEOUT_HOLD_US) {
				_publishVehicleCmdDoLoiter();
			}

			_last_timeout_warning = now;
		}

		return 0.f;
	}

	const float data_age_s = (_closest_distance_front_timestamp > 0) ?
				 ((now - _closest_distance_front_timestamp) * 1e-6f) : 0.f;

	// Calculate speed limit based on obstacle distance
	float speed_limit_factor = _calculateSpeedLimit(obstacle_distance, speed_setpoint, data_age_s);

	// Apply minimum speed if we're slowing down but not stopped
	float modified_speed = speed_setpoint * speed_limit_factor;

	if (speed_limit_factor < 1.0f && speed_limit_factor > 0.f) {
		modified_speed = math::max(modified_speed, 0.f);
	}

	// Publish constraints for logging/debugging
	collision_constraints_s constraints{};
	constraints.timestamp = hrt_absolute_time();
	constraints.original_setpoint[0] = speed_setpoint;
	constraints.original_setpoint[1] = 0.f;
	constraints.adapted_setpoint[0] = modified_speed;
	constraints.adapted_setpoint[1] = 0.f;
	_constraints_pub.publish(constraints);

	return modified_speed;
}

float RoverCollisionPrevention::modifyYawSetpoint(float desired_yaw, float vehicle_yaw)
{
	if (!isActive() || _param_cp_guide_ang.get() <= 0.f) {
		return desired_yaw;
	}

	_updateObstacleData();

	if (!_any_obstacle_data_present) {
		return desired_yaw;
	}

	const float desired_direction_body = matrix::wrap_pi(desired_yaw - vehicle_yaw);
	const float guided_direction_body = _selectGuidedDirection(desired_direction_body);
	return matrix::wrap_pi(vehicle_yaw + guided_direction_body);
}

float RoverCollisionPrevention::getObstacleDistanceFront()
{
	_updateObstacleData();
	return _closest_distance_front;
}

bool RoverCollisionPrevention::shouldStop()
{
	if (!isActive()) {
		return false;
	}

	float obstacle_distance = getObstacleDistanceFront();
	return obstacle_distance <= _param_cp_dist.get();
}

void RoverCollisionPrevention::_updateObstacleData()
{
	const hrt_abstime now = hrt_absolute_time();

	// Get vehicle attitude for sensor orientation
	vehicle_attitude_s attitude;
	float vehicle_yaw = 0.f;

	if (_vehicle_attitude_sub.copy(&attitude)) {
		vehicle_yaw = Eulerf(Quatf(attitude.q)).psi();
	}

	// Read distance sensor data
	for (auto &sub : _distance_sensor_subs) {
		distance_sensor_s distance_sensor;

		if (sub.update(&distance_sensor)) {
			_addDistanceSensorData(distance_sensor);
		}
	}

	// Read obstacle distance message (from offboard/companion computer)
	if (_sub_obstacle_distance.update()) {
		_addObstacleDistanceData(_sub_obstacle_distance.get(), vehicle_yaw);
	}

	// Check for stale data and find closest obstacle in front
	_data_stale = true;
	_any_obstacle_data_present = false;
	_front_obstacle_data_present = false;
	_front_data_timed_out_in_fov = false;
	_closest_distance_front = FLT_MAX;
	_closest_distance_front_timestamp = 0;

	// Front sector: -45 to +45 degrees (bins centered around 0)
	int front_start_bin = BIN_COUNT - (int)(FRONT_SECTOR_HALF_WIDTH / BIN_SIZE);
	int front_end_bin = (int)(FRONT_SECTOR_HALF_WIDTH / BIN_SIZE);

	for (int i = 0; i < BIN_COUNT; i++) {
		// Check if this bin is in the front sector
		bool in_front_sector = (i >= front_start_bin) || (i <= front_end_bin);

		const bool data_valid = (_data_timestamps[i] > 0) && ((now - _data_timestamps[i]) < DATA_TIMEOUT_US);

		if (data_valid) {
			_data_stale = false;
			_any_obstacle_data_present = true;

			if (in_front_sector) {
				_front_obstacle_data_present = true;
				float distance_m = _obstacle_distances[i] / 100.f;  // cm to m

				if (distance_m < _closest_distance_front) {
					_closest_distance_front = distance_m;
					_closest_distance_front_timestamp = _data_timestamps[i];
				}
			}

		} else {
			_obstacle_distances[i] = UINT16_MAX;

			if (in_front_sector && _data_fov[i]) {
				_front_data_timed_out_in_fov = true;
			}
		}
	}

	if (_any_obstacle_data_present) {
		_last_data_time = now;
	}

	// Publish fused obstacle distance map for logging/debugging
	obstacle_distance_s fused{};
	fused.timestamp = now;
	fused.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	fused.increment = BIN_SIZE;
	fused.angle_offset = 0.f;
	fused.min_distance = UINT16_MAX;
	fused.max_distance = 0;

	for (int i = 0; i < BIN_COUNT; i++) {
		fused.distances[i] = _obstacle_distances[i];

		if (_obstacle_distances[i] < UINT16_MAX) {
			fused.min_distance = math::min(fused.min_distance, _obstacle_distances[i]);
		}

		if (_data_maxranges[i] > fused.max_distance) {
			fused.max_distance = _data_maxranges[i];
		}
	}

	_obstacle_distance_fused_pub.publish(fused);

	_last_update = now;
}

float RoverCollisionPrevention::_getObstacleDistance(float direction_rad)
{
	// Convert direction to bin index
	float direction_deg = math::degrees(direction_rad);

	// Normalize to 0-360
	while (direction_deg < 0.f) { direction_deg += 360.f; }

	while (direction_deg >= 360.f) { direction_deg -= 360.f; }

	int bin_index = (int)(direction_deg / BIN_SIZE) % BIN_COUNT;

	if (_obstacle_distances[bin_index] < UINT16_MAX) {
		return _obstacle_distances[bin_index] / 100.f;  // cm to m
	}

	return FLT_MAX;
}

float RoverCollisionPrevention::_calculateSpeedLimit(float obstacle_distance, float speed_setpoint, float data_age_s)
{
	const float min_dist = _param_cp_dist.get();

	if (obstacle_distance == FLT_MAX || min_dist <= 0.f) {
		return 1.f;
	}

	// Define a slow-down distance based on minimum distance and compensated sensor delay.
	const float delay_s = math::max(0.f, _param_cp_delay.get() + data_age_s);
	const float slow_dist = min_dist + math::max(1.f, speed_setpoint * delay_s);

	if (obstacle_distance <= min_dist) {
		// Too close - stop
		return 0.f;

	} else if (obstacle_distance >= slow_dist) {
		// Far enough - no speed limit
		return 1.f;

	} else {
		// Linear interpolation between min_dist and slow_dist
		// At min_dist: factor = 0
		// At slow_dist: factor = 1
		float factor = (obstacle_distance - min_dist) / (slow_dist - min_dist);
		return math::constrain(factor, 0.f, 1.f);
	}
}

float RoverCollisionPrevention::_selectGuidedDirection(float desired_direction_rad)
{
	const float guide_angle = _param_cp_guide_ang.get();

	if (guide_angle <= 0.f || !_any_obstacle_data_present) {
		return desired_direction_rad;
	}

	float desired_deg = math::degrees(desired_direction_rad);

	while (desired_deg < 0.f) { desired_deg += 360.f; }
	while (desired_deg >= 360.f) { desired_deg -= 360.f; }

	const int sp_index_original = (int)(desired_deg / BIN_SIZE) % BIN_COUNT;
	const int guidance_bins = (int)floor(guide_angle / BIN_SIZE);

	float best_cost = FLT_MAX;
	int best_index = sp_index_original;

	for (int i = sp_index_original - guidance_bins; i <= sp_index_original + guidance_bins; i++) {
		const int bin = (i % BIN_COUNT + BIN_COUNT) % BIN_COUNT;

		if (_obstacle_distances[bin] == UINT16_MAX) {
			continue;
		}

		// Simple moving average to center in wider gaps
		const int filter_size = 1;
		float mean_dist = 0.f;

		for (int j = i - filter_size; j <= i + filter_size; j++) {
			const int wrapped = (j % BIN_COUNT + BIN_COUNT) % BIN_COUNT;
			if (_obstacle_distances[wrapped] == UINT16_MAX) {
				mean_dist += _param_cp_dist.get() * 100.f;
			} else {
				mean_dist += _obstacle_distances[wrapped];
			}
		}

		mean_dist = mean_dist / (2.f * filter_size + 1.f);
		const float deviation_cost = _param_cp_dist.get() * 50.f * fabsf((float)(i - sp_index_original));
		const float bin_cost = deviation_cost - mean_dist - _obstacle_distances[bin];

		if (bin_cost < best_cost) {
			best_cost = bin_cost;
			best_index = bin;
		}
	}

	if (best_index == sp_index_original) {
		return desired_direction_rad;
	}

	const float guided_deg = (float)best_index * BIN_SIZE;
	return math::radians(guided_deg);
}

void RoverCollisionPrevention::_addDistanceSensorData(const distance_sensor_s &distance_sensor)
{
	// Only use horizontal sensors (orientation pointing forward, left, right, back)
	if (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING ||
	    distance_sensor.orientation == distance_sensor_s::ROTATION_UPWARD_FACING) {
		return;
	}

	float distance_reading = math::min(distance_sensor.current_distance, distance_sensor.max_distance);

	// Negative current distance with invalid quality indicates "no return, but sensor healthy".
	if (fabsf(distance_sensor.current_distance - -1.f) < FLT_EPSILON && distance_sensor.signal_quality == 0) {
		distance_reading = distance_sensor.max_distance;
	}

	// Discard values below minimum range.
	if (distance_reading <= distance_sensor.min_distance) {
		return;
	}

	// Get sensor orientation in body frame.
	float sensor_yaw_body_rad = 0.f;

	if (distance_sensor.orientation <= distance_sensor_s::ROTATION_YAW_315) {
		sensor_yaw_body_rad = math::radians(45.f * distance_sensor.orientation);

	} else if (distance_sensor.orientation == distance_sensor_s::ROTATION_CUSTOM) {
		const Quatf sensor_orientation_q(distance_sensor.q);
		sensor_yaw_body_rad = Eulerf(sensor_orientation_q).psi();

	} else {
		return;
	}

	float sensor_yaw_deg = math::degrees(matrix::wrap_pi(sensor_yaw_body_rad));
	while (sensor_yaw_deg < 0.f) { sensor_yaw_deg += 360.f; }

	const float fov_half_deg = math::max(0.5f * BIN_SIZE, 0.5f * math::degrees(distance_sensor.h_fov));
	const int lower_bound = (int)roundf((sensor_yaw_deg - fov_half_deg) / BIN_SIZE);
	const int upper_bound = (int)roundf((sensor_yaw_deg + fov_half_deg) / BIN_SIZE);
	const uint16_t distance_cm = static_cast<uint16_t>(distance_reading * 100.f + 0.5f);
	const uint16_t max_range_cm = static_cast<uint16_t>(distance_sensor.max_distance * 100.f + 0.5f);

	for (int bin = lower_bound; bin <= upper_bound; ++bin) {
		const int wrapped_bin = (bin % BIN_COUNT + BIN_COUNT) % BIN_COUNT;

		if (_data_timestamps[wrapped_bin] + DATA_TIMEOUT_US < distance_sensor.timestamp ||
		    _obstacle_distances[wrapped_bin] == UINT16_MAX ||
		    distance_cm <= _obstacle_distances[wrapped_bin]) {
			_obstacle_distances[wrapped_bin] = distance_cm;
			_data_timestamps[wrapped_bin] = distance_sensor.timestamp;
			_data_maxranges[wrapped_bin] = max_range_cm;
			_data_fov[wrapped_bin] = 1;
		}
	}
}

void RoverCollisionPrevention::_addObstacleDistanceData(const obstacle_distance_s &obstacle, float vehicle_yaw)
{
	if (obstacle.increment <= 0.f) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime sample_time = obstacle.timestamp > 0 ? obstacle.timestamp : now;
	const float vehicle_orientation_deg = math::degrees(vehicle_yaw);
	const bool is_global_frame = obstacle.frame == obstacle_distance_s::MAV_FRAME_GLOBAL
				     || obstacle.frame == obstacle_distance_s::MAV_FRAME_LOCAL_NED;

	for (int i = 0; i < BIN_COUNT && i < (int)(sizeof(obstacle.distances) / sizeof(obstacle.distances[0])); i++) {
		const uint16_t distance_cm = obstacle.distances[i];

		// Ignore "no obstacle" and unknown values.
		if (distance_cm == UINT16_MAX || distance_cm > obstacle.max_distance) {
			continue;
		}

		float angle_deg = obstacle.angle_offset + (i * obstacle.increment);

		if (is_global_frame) {
			angle_deg -= vehicle_orientation_deg;
		}

		while (angle_deg < 0.f) { angle_deg += 360.f; }
		while (angle_deg >= 360.f) { angle_deg -= 360.f; }

		const int bin_index = (int)(angle_deg / BIN_SIZE) % BIN_COUNT;

		if (_data_timestamps[bin_index] + DATA_TIMEOUT_US < sample_time ||
		    _obstacle_distances[bin_index] == UINT16_MAX ||
		    distance_cm <= _obstacle_distances[bin_index]) {
			_obstacle_distances[bin_index] = distance_cm;
			_data_timestamps[bin_index] = sample_time;
			_data_maxranges[bin_index] = obstacle.max_distance;
			_data_fov[bin_index] = 1;
		}
	}
}

void RoverCollisionPrevention::_publishVehicleCmdDoLoiter()
{
	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = 1.f; // base mode VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
	command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
	command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;
	command.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(command);
}
