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
 * @file rover_collision_prevention_params.c
 *
 * Parameters for rover collision prevention.
 *
 * @author PXLABS
 */

/**
 * Rover collision prevention minimum distance
 *
 * Minimum distance the rover should keep to obstacles.
 * The rover will stop if an obstacle is closer than this distance.
 * Set to -1 to disable collision prevention.
 *
 * @min -1
 * @max 10
 * @unit m
 * @decimal 2
 * @group Rover Collision Prevention
 */
PARAM_DEFINE_FLOAT(RCP_DIST, -1.0f);

/**
 * Rover collision prevention slow down distance
 *
 * Distance at which the rover starts to slow down when approaching an obstacle.
 * The rover will linearly reduce speed between this distance and RCP_DIST.
 *
 * @min 0.5
 * @max 20
 * @unit m
 * @decimal 1
 * @group Rover Collision Prevention
 */
PARAM_DEFINE_FLOAT(RCP_SLOW_DIST, 5.0f);

/**
 * Rover collision prevention minimum speed
 *
 * Minimum speed the rover will maintain when slowing down for obstacles.
 * This prevents the rover from moving too slowly when obstacles are detected
 * but still at a safe distance.
 *
 * @min 0.0
 * @max 2.0
 * @unit m/s
 * @decimal 2
 * @group Rover Collision Prevention
 */
PARAM_DEFINE_FLOAT(RCP_MIN_SPEED, 0.3f);

/**
 * Rover collision prevention allow movement without sensor data
 *
 * If enabled, the rover is allowed to move in directions where there is
 * no sensor coverage. If disabled, the rover will stop when sensor data
 * is not available.
 *
 * @boolean
 * @group Rover Collision Prevention
 */
PARAM_DEFINE_INT32(RCP_GO_NO_DATA, 1);
