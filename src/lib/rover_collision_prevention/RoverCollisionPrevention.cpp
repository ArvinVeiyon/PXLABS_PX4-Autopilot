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
	}
}

bool RoverCollisionPrevention::isActive()
{
	// Collision prevention is active if minimum distance parameter is positive
	return _param_rcp_dist.get() > 0.f;
}

float RoverCollisionPrevention::modifySpeedSetpoint(float speed_setpoint, float vehicle_yaw)
{
	if (!isActive()) {
		return speed_setpoint;
	}

	// Update obstacle data from sensors
	_updateObstacleData();

	// Only limit forward speed (positive speed setpoint)
	if (speed_setpoint <= 0.f) {
		// TODO: Could add rear obstacle detection for reverse
		return speed_setpoint;
	}

	// Get closest obstacle distance in front sector
	float obstacle_distance = getObstacleDistanceFront();

	// Check if we have valid data
	if (!_obstacle_data_present && !_param_rcp_go_no_data.get()) {
		// No obstacle data and not allowed to move without data
		mavlink_log_warning(&_mavlink_log_pub, "Rover collision prevention: no sensor data\t");
		return 0.f;
	}

	// Calculate speed limit based on obstacle distance
	float speed_limit_factor = _calculateSpeedLimit(obstacle_distance);

	// Apply minimum speed if we're slowing down but not stopped
	float modified_speed = speed_setpoint * speed_limit_factor;

	if (speed_limit_factor < 1.0f && speed_limit_factor > 0.f) {
		modified_speed = math::max(modified_speed, _param_rcp_min_speed.get());
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
	return obstacle_distance <= _param_rcp_dist.get();
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
			_addDistanceSensorData(distance_sensor, vehicle_yaw);
		}
	}

	// Read obstacle distance message (from offboard/companion computer)
	if (_sub_obstacle_distance.update()) {
		_addObstacleDistanceData(_sub_obstacle_distance.get(), vehicle_yaw);
	}

	// Check for stale data and find closest obstacle in front
	_data_stale = true;
	_obstacle_data_present = false;
	_closest_distance_front = FLT_MAX;

	// Front sector: -45 to +45 degrees (bins centered around 0)
	int front_start_bin = BIN_COUNT - (int)(FRONT_SECTOR_HALF_WIDTH / BIN_SIZE);
	int front_end_bin = (int)(FRONT_SECTOR_HALF_WIDTH / BIN_SIZE);

	for (int i = 0; i < BIN_COUNT; i++) {
		// Check if this bin is in the front sector
		bool in_front_sector = (i >= front_start_bin) || (i <= front_end_bin);

		if (_data_timestamps[i] > 0 && (now - _data_timestamps[i]) < DATA_TIMEOUT_US) {
			_data_stale = false;
			_obstacle_data_present = true;

			if (in_front_sector) {
				float distance_m = _obstacle_distances[i] / 100.f;  // cm to m

				if (distance_m < _closest_distance_front) {
					_closest_distance_front = distance_m;
				}
			}
		}
	}

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

float RoverCollisionPrevention::_calculateSpeedLimit(float obstacle_distance)
{
	const float min_dist = _param_rcp_dist.get();
	const float slow_dist = _param_rcp_slow_dist.get();

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

void RoverCollisionPrevention::_addDistanceSensorData(const distance_sensor_s &distance_sensor, float vehicle_yaw)
{
	// Only use horizontal sensors (orientation pointing forward, left, right, back)
	if (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING ||
	    distance_sensor.orientation == distance_sensor_s::ROTATION_UPWARD_FACING) {
		return;
	}

	// Get sensor orientation in body frame
	float sensor_yaw_body = 0.f;

	switch (distance_sensor.orientation) {
	case distance_sensor_s::ROTATION_FORWARD_FACING:
		sensor_yaw_body = 0.f;
		break;

	case distance_sensor_s::ROTATION_RIGHT_FACING:
		sensor_yaw_body = math::radians(90.f);
		break;

	case distance_sensor_s::ROTATION_BACKWARD_FACING:
		sensor_yaw_body = math::radians(180.f);
		break;

	case distance_sensor_s::ROTATION_LEFT_FACING:
		sensor_yaw_body = math::radians(-90.f);
		break;

	default:
		// Use h_fov for custom orientations if available
		sensor_yaw_body = distance_sensor.h_fov / 2.f;  // Approximate
		break;
	}

	// Convert to bin index (relative to body frame, 0 = forward)
	float sensor_yaw_deg = math::degrees(sensor_yaw_body);

	while (sensor_yaw_deg < 0.f) { sensor_yaw_deg += 360.f; }

	int bin_index = (int)(sensor_yaw_deg / BIN_SIZE) % BIN_COUNT;

	// Store distance if valid
	if (distance_sensor.current_distance >= distance_sensor.min_distance &&
	    distance_sensor.current_distance <= distance_sensor.max_distance) {

		uint16_t distance_cm = (uint16_t)(distance_sensor.current_distance * 100.f);
		_obstacle_distances[bin_index] = distance_cm;
		_data_timestamps[bin_index] = distance_sensor.timestamp;
		_data_maxranges[bin_index] = (uint16_t)(distance_sensor.max_distance * 100.f);
	}
}

void RoverCollisionPrevention::_addObstacleDistanceData(const obstacle_distance_s &obstacle, float vehicle_yaw)
{
	const hrt_abstime now = hrt_absolute_time();

	// Copy obstacle distances from message
	for (int i = 0; i < BIN_COUNT && i < (int)(sizeof(obstacle.distances) / sizeof(obstacle.distances[0])); i++) {
		if (obstacle.distances[i] < obstacle.max_distance) {
			// Adjust bin index based on message angle offset
			float angle_deg = obstacle.angle_offset + (i * obstacle.increment);

			while (angle_deg < 0.f) { angle_deg += 360.f; }

			while (angle_deg >= 360.f) { angle_deg -= 360.f; }

			int bin_index = (int)(angle_deg / BIN_SIZE) % BIN_COUNT;

			_obstacle_distances[bin_index] = obstacle.distances[i];
			_data_timestamps[bin_index] = now;
			_data_maxranges[bin_index] = obstacle.max_distance;
		}
	}
}
