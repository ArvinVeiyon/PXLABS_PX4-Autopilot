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
 * @file RoverCollisionPrevention.hpp
 *
 * Collision prevention for ground rovers.
 * Limits speed and stops the rover when obstacles are detected.
 *
 * @author PXLABS
 */

#pragma once

#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/collision_constraints.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>

using namespace time_literals;

class RoverCollisionPrevention : public ModuleParams
{
public:
	RoverCollisionPrevention(ModuleParams *parent);
	~RoverCollisionPrevention() override = default;

	/**
	 * Check if collision prevention is enabled and active
	 * @return true if collision prevention is active
	 */
	bool isActive();

	/**
	 * Modify the speed setpoint based on obstacle distances
	 * @param speed_setpoint Current speed setpoint [m/s], will be modified
	 * @param vehicle_yaw Current vehicle yaw [rad]
	 * @return Modified speed setpoint [m/s]
	 */
	float modifySpeedSetpoint(float speed_setpoint, float vehicle_yaw);

	/**
	 * Modify the yaw setpoint to guide around obstacles.
	 * @param desired_yaw Desired yaw setpoint [rad]
	 * @param vehicle_yaw Current vehicle yaw [rad]
	 * @return Modified yaw setpoint [rad]
	 */
	float modifyYawSetpoint(float desired_yaw, float vehicle_yaw);

	/**
	 * Get the minimum distance to obstacle in front of the rover
	 * @return Distance to closest obstacle in front [m], or max range if none
	 */
	float getObstacleDistanceFront();

	/**
	 * Check if rover should stop due to obstacle
	 * @return true if rover should stop
	 */
	bool shouldStop();

private:
	/**
	 * Update obstacle data from sensors
	 */
	void _updateObstacleData();

	/**
	 * Get distance to obstacle in a specific direction
	 * @param direction_rad Direction relative to vehicle heading [rad]
	 * @return Distance [m]
	 */
	float _getObstacleDistance(float direction_rad);

	/**
	 * Calculate speed limit based on obstacle distance
	 * @param obstacle_distance Distance to obstacle [m]
	 * @return Speed limit factor [0, 1]
	 */
	float _calculateSpeedLimit(float obstacle_distance, float speed_setpoint, float data_age_s);

	/**
	 * Select a guided direction in body frame within CP_GUIDE_ANG.
	 * @param desired_direction_rad Desired direction in body frame [rad]
	 * @return Guided direction in body frame [rad]
	 */
	float _selectGuidedDirection(float desired_direction_rad);

	/**
	 * Add distance sensor data to obstacle map
	 */
	void _addDistanceSensorData(const distance_sensor_s &distance_sensor);

	/**
	 * Add obstacle distance message data to obstacle map
	 */
	void _addObstacleDistanceData(const obstacle_distance_s &obstacle, float vehicle_yaw);

	// Obstacle map
	static constexpr int BIN_COUNT = 72;  // 5 degree resolution (360/5)
	static constexpr int BIN_SIZE = 360 / BIN_COUNT;
	static constexpr float FRONT_SECTOR_HALF_WIDTH = 45.0f;  // +/- 45 degrees from heading

	uint16_t _obstacle_distances[BIN_COUNT] {};  // in cm
	uint64_t _data_timestamps[BIN_COUNT] {};
	uint16_t _data_maxranges[BIN_COUNT] {};  // in cm
	uint8_t _data_fov[BIN_COUNT] {};  // bin has been covered by any sensor/FOV

	bool _data_stale{true};
	bool _any_obstacle_data_present{false};
	bool _front_obstacle_data_present{false};
	bool _front_data_timed_out_in_fov{false};
	float _closest_distance_front{FLT_MAX};
	uint64_t _closest_distance_front_timestamp{0};
	hrt_abstime _last_data_time{0};

	hrt_abstime _last_update{0};
	static constexpr uint64_t DATA_TIMEOUT_US{500_ms};
	static constexpr uint64_t TIMEOUT_HOLD_US{5_s};

	orb_advert_t _mavlink_log_pub{nullptr};

	// Subscriptions
	uORB::SubscriptionData<obstacle_distance_s> _sub_obstacle_distance{ORB_ID(obstacle_distance)};
	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	// Publications
	uORB::Publication<collision_constraints_s> _constraints_pub{ORB_ID(collision_constraints)};
	uORB::Publication<obstacle_distance_s> _obstacle_distance_fused_pub{ORB_ID(obstacle_distance_fused)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	// State for no-data handling
	hrt_abstime _last_timeout_warning{0};
	hrt_abstime _time_activated{0};
	bool _was_active{false};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CP_DIST>) _param_cp_dist,            /**< Minimum distance to keep from obstacles */
		(ParamFloat<px4::params::CP_DELAY>) _param_cp_delay,          /**< Delay compensation for sensors */
		(ParamFloat<px4::params::CP_GUIDE_ANG>) _param_cp_guide_ang,  /**< Guidance angle to steer around obstacles */
		(ParamBool<px4::params::CP_GO_NO_DATA>) _param_cp_go_no_data  /**< Allow movement where no sensor data */
	)

	/**
	 * Publishes vehicle command to hold/loiter.
	 */
	void _publishVehicleCmdDoLoiter();
};
