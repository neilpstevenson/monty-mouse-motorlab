/******************************************************************************
 * Project: mazerunner-core                                                   *
 * -----                                                                      *
 * Copyright 2022 - 2023 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>
#include "motors.h"
#include "profile.h"

/***
 *
 * The motion class handles all the higher level Locomotion tasks. It is responsible
 * for converting instructions from a path planner into actual movement of the
 * robot.
 *
 * Motion control needs to know about the dynamics and kinematics of the robot
 * and may need environmental data as well. In particular, the battery voltage and
 * characteristics of the motors will be important.
 *
 * The output from the motion controller should be suitable control signals for
 * the actuators that make the robot move. In UKMARSBOT, that will simply be
 * voltages for each of the two drive motors. Other robots may have more motors,
 * an independent steering system or other kinds of effectors such as grippers or
 * weapons. Remember that the motors may be stepper motors which will need a
 * slightly modified approach.
 *
 * Here, the motion control makes use of simple trapezoidal profiles to calculate
 * continuously varying speeds for linear and rotary motion components.
 *
 * How these speeds are converted into individual control signals for the drive
 * system is the job of the Motors class. Thus is should be relatively easy to
 * change that class to suit different drive systems.
 *
 * TODO: Motion should have be given the profilers
 * TODO: should it also be given the motors?
 */
class Motion {
 public:
  /**
   * Before the robot begins a sequence of moves, this method can be used to
   * make sure everything starts off in a known state.
   *
   * @brief Reset profiles, counters and controllers. Motors off. Steering off.
   */
  void reset_drive_system() {
    motors.stop();
    motors.disable_controllers();
    encoders.reset();
    profile.reset();
    motors.reset_controllers();
    motors.enable_controllers();
  }

  void stop() {
    motors.stop();
  }

  void disable_drive() {
    motors.disable_controllers();
  }

  float position() {
    return profile.position();
  }

  float velocity() {
    return profile.speed();
  }

  float acceleration() {
    return profile.acceleration();
  }

  void set_target_velocity(float velocity) {
    profile.set_target_speed(velocity);
  }

  void start_move(float distance, float top_speed, float final_speed, float acceleration) {
    profile.start(distance, top_speed, final_speed, acceleration);
  }

  void stop_move() {
    profile.stop();
  }

  bool move_finished() {
    return profile.is_finished();
  }

  void move(float distance, float top_speed, float final_speed, float acceleration) {
    profile.start(distance, top_speed, final_speed, acceleration);
  }

  void update() {
    profile.update();
  }

  void set_position(float pos) {
    profile.set_position(pos);
  }

  void adjust_forward_position(float delta) {
    profile.adjust_position(delta);
  }

  //***************************************************************************//
  /**
   * These are examples of ways to use the motion control functions
   */

  /**
   * The robot is assumed to be moving. This call will stop at a specific
   * distance. Clearly, there must be enough distance remaining for it to
   * brake to a halt.
   *
   * The current values for speed and acceleration are used.
   *
   * Calling this with the robot stationary is undefined. Don't do that.
   *
   * @brief bring the robot to a halt at a specific distance
   */
  void stop_at(float position) {
    float remaining = position - profile.position();
    profile.start(remaining, profile.speed(), 0, profile.acceleration());
  }

  /**
   * The robot is assumed to be moving. This call will stop  after a
   * specific distance has been travelled
   *
   * Clearly, there must be enough distance remaining for it to
   * brake to a halt.
   *
   * The current values for speed and acceleration are used.
   *
   * Calling this with the robot stationary is undefined. Don't do that.
   *
   * @brief bring the robot to a halt after a specific distance
   */
  void stop_after(float distance) {
    profile.start(distance, profile.speed(), 0, profile.acceleration());
  }

  /**
   * The robot is assumed to be moving. This utility function call will just
   * do a busy-wait until the forward profile gets to the supplied position.
   *
   * @brief wait until the given position is reached
   */
  void wait_until_position(float position) {
    while (profile.position() < position) {
      delay(2);
    }
  }

  /**
   * The robot is assumed to be moving. This utility function call will just
   * do a busy-wait until the forward profile has moved by the given distance.
   *
   * @brief wait until the given distance has been travelled
   */
  void wait_until_distance(float distance) {
    float target = profile.position() + distance;
    wait_until_position(target);
  }
};

extern Motion motion;

#endif
