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
#pragma once

#include <Arduino.h>

// Customised interfaces for this robot hardware
//#include "indicators-rp2040zero.h"

/*****************************************************************************
 *
 * HALF MONTY is a half-size mouse based on an Waveshare RP2040 Zero board
 *
 * It uses a wall sensor board with four emitter-detector pairs. Gearmotors
 * with 20:1 ratio gearboxes and encoder discs with 6 magnets in each.
 *
 * The sensors consist of SFH4550 emitters and SFH309FA detectors.
 *
 *****************************************************************************/
#define NAME "HALF MONTY"

// the ADVANCED sensor board has only one LED so use the value twice
const int LED_LEFT = LED_LEFT_IO;
const int LED_RIGHT = LED_RIGHT_IO;
//const int LED_USER = LED_NEOPIXEL_IO; // TODO
// but two emitter pins
const int EMITTER_FRONT = EMITTER_A;
const int EMITTER_DIAGONAL = EMITTER_B;

//***** SENSOR HARDWARE *****************************************************//
// the ADC channels corresponding to the sensor inputs. There are 8 available
// Channels 0..3 are normally used for sensors.
// Channels 4 and 5 are available if you do not want to add an I2C device
// Channel 6 is pre-allocated to the Battery monitor
// Channel 7 is re-allocated to the function switch and button

// NOTE - these are the AnalogueConverter channel indexes, not necessariy the
// hardware ADC channel numbers

// ADVANCED SENSOR
const int RFS_ADC_CHANNEL = 1;
const int RSS_ADC_CHANNEL = 3;
const int LSS_ADC_CHANNEL = 0;
const int LFS_ADC_CHANNEL = 2;

// BASIC SENSOR - just repeat the front sensor to make the code cleaner
// #define RFS_ADC_CHANNEL 1
// #define RSS_ADC_CHANNEL 0
// #define LSS_ADC_CHANNEL 2
// #define LFS_ADC_CHANNEL 1
// there are two other ADC channels used by the robot
const int SWITCHES_ADC_CHANNEL = SWITCH_SELECT_PIN;
const int BATTERY_ADC_CHANNEL = -1; // dummy - we don't have this
//***************************************************************************//
const uint32_t BAUDRATE = 115200;

//***************************************************************************//
// set this to zero to disable profile data logging over SerialPort
#define DEBUG_LOGGING 1
// time between logged lines when reporting is enabled (milliseconds)
const int REPORTING_INTERVAL = 10;

//***************************************************************************//
// Some physical constants that are likely to be robot-specific
// with robot against back wall, how much travel is there to the cell center?
const int BACK_WALL_TO_CENTER = 21; //14;

//***************************************************************************//
// We need to know about the drive mechanics.
// The encoder pulse counts should be obvious from the encoder itself.
// Work out the gear ratio by rotating the wheel a number of turns and counting
// the pulses.
// Finally, move the mouse in a straight line through 1000mm of travel to work
// out the wheel diameter.
const float ENCODER_PULSES = 12.00;
const float GEAR_RATIO = 42.0; //37.6; //42.0;
const float WHEEL_DIAMETER = 20.19; //19.85; //20.05; //20.20;

// Mouse radius is the distance between the contact patches of the drive wheels.
// A good starting approximation is half the distance between the wheel centres.
// After testing, you may find the working value to be larger or smaller by some
// small amount. AFTER you have the wheel diameter and gear ratio calibrated,
// have the mouse turn in place and adjust the MOUSE_RADIUS until these turns are
// as accurate as you can get them
const float MOUSE_RADIUS = 21.0; //18.5; // Adjust on test

// The robot is likely to have wheels of different diameters or motors of slightly
// different characteristics and that must be compensated for if the robot is to
// reliably drive in a straight line.
// This number adjusts the encoder count and must be  added to the right
// and subtracted from the left motor.
const float ROTATION_BIAS = -0.0017; //-0.0025; //-0.035; // Negative makes robot curve to left

// Now we can pre-calculate the key constats for the motion control
const float MM_PER_COUNT = PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * MM_PER_COUNT;
const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * MM_PER_COUNT;
const float DEG_PER_MM_DIFFERENCE = (180.0 / (2 * MOUSE_RADIUS * PI));

//*** MOTION CONTROLLER CONSTANTS **********************************************//

//***************************************************************************//
// Control loop timing. Pre-calculate to save time in interrupts
const float LOOP_FREQUENCY = 500.0;
const float LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY);

// Dynamic performance constants
// There is a video describing how to get these numbers and calculate the feedforward
// constnats here: https://youtu.be/BrabDeHGsa0
const float FWD_KM = 98.0; //78.1; //475.0;  // mm/s/Volt
const float FWD_TM = 0.049; //0.190;  // forward time constant
const float ROT_KM = 775.0;  // deg/s/Volt
const float ROT_TM = 0.210;  // rotation time constant

// Motor Feedforward
/***
 * Speed Feedforward is used to add a drive voltage proportional to the motor speed
 * The units are Volts per mm/s and the value will be different for each
 * robot where the motor + gearbox + wheel diamter + robot weight are different
 * You can experimentally determine a suitable value by turning off the controller
 * and then commanding a set voltage to the motors. The same voltage is applied to
 * each motor. Have the robot report its speed regularly or have it measure
 * its steady state speed after a period of acceleration.
 * Do this for several applied voltages from 0.5 Volts to 5 Volts in steps of 0.5V
 * Plot a chart of steady state speed against voltage. The slope of that graph is
 * the speed feedforward, SPEED_FF.
 * Note that the line will not pass through the origin because there will be
 * some minimum voltage needed just to ovecome friction and get the wheels to turn at all.
 * That minimum voltage is the BIAS_FF. It is not dependent upon speed but is expressed
 * here as a fraction for comparison.
 */
const float MAX_MOTOR_VOLTS = 6.0;

const float SPEED_FF = (1.0 / FWD_KM);
const float ACC_FF = (FWD_TM / FWD_KM);
const float BIAS_FF = 0.07; //0.121;
const float TOP_SPEED = (MAX_MOTOR_VOLTS - BIAS_FF) / SPEED_FF;

//*** MOTION CONTROL CONSTANTS **********************************************//

// forward motion controller constants
const float FWD_ZETA = 0.817; //0.707;
const float FWD_TD = 0.194; //FWD_TM;

const float FWD_KP = 16 * FWD_TM / (FWD_KM * FWD_ZETA * FWD_ZETA * FWD_TD * FWD_TD);
const float FWD_KD = LOOP_FREQUENCY * (8 * FWD_TM - FWD_TD) / (FWD_KM * FWD_TD);

// rotation motion controller constants
const float ROT_ZETA = 0.707;
const float ROT_TD = ROT_TM;

const float ROT_KP = 16 * ROT_TM / (ROT_KM * ROT_ZETA * ROT_ZETA * ROT_TD * ROT_TD);
const float ROT_KD = LOOP_FREQUENCY * (8 * ROT_TM - ROT_TD) / (ROT_KM * ROT_TD);

// controller constants for the steering controller
const float STEERING_KP = 0.002;
const float STEERING_KD = 0.00;
const float STEERING_ADJUST_LIMIT = 10.0;  // deg/s

// encoder polarity is either 1 or -1 and is used to account for reversal of the encoder phases
//#define ENCODER_LEFT_POLARITY (1)
//#define ENCODER_RIGHT_POLARITY (-1)

// similarly, the motors may be wired with different polarity and that is defined here so that
// setting a positive voltage always moves the robot forwards
#define MOTOR_LEFT_POLARITY (-1)
#define MOTOR_RIGHT_POLARITY (1)

//***************************************************************************//

//***** PERFORMANCE CONSTANTS************************************************//
// search and run speeds in mm/s and mm
const int SEARCH_SPEED_DEFAULT = 500; //220;
#define SEARCH_ACCEL_HALF_CELL(speed) (speed*speed/(45-10)/2)   // Acceleration to stop in half a cell - 10
//const int SEARCH_ACCELERATION_DEFAULT = 1800; //2000;
const int SEARCH_TURN_SPEED_DEFAULT = 200; //300; //200;

const int OMEGA_SPIN_TURN_DEFAULT = 360;
const int ALPHA_SPIN_TURN_DEFAULT = 3600;

//***************************************************************************//
// Battery resistor bridge //Derek Hall//
// The battery measurement is performed by first reducing the battery voltage
// with a potential divider formed by two resistors. Here they are named R1 and R2
// though that may not be their designation on the schematics.
//
// Resistor R1 is the high-side resistor and connects to the battery supply
// Resistor R2 is the low-side resistor and connects to ground
// Battery voltage is measured at the junction of these resistors
// The ADC port used for the conversion will have a full scale reading (FSR) that
// depends on the device being used. Typically that will be 1023 for a 10-bit ADC as
// found on an Arduino but it may be 4095 if you have a 12-bit ADC.
// Finally, the ADC converter on your processor will have a reference voltage. On
// the Arduinos for example, this is 5 Volts. Thus, a full scale reading of
// 1023 would represent 5 Volts, 511 would be 2.5Volts and so on.
//
// in this section you can enter the appropriate values for your ADC and potential
// divider setup to ensure that the battery voltage reading performed by the sensors
// is as accurate as possible.
//
// By calculating the battery multiplier here, you can be sure that the actual
// battery voltage calulation is done as efficiently as possible.
// The compiler will do all these calculations so your program does not have to.

const float BATTERY_R1 = 10000.0;  // resistor to battery +
const float BATTERY_R2 = 10000.0;  // resistor to Gnd
const float BATTERY_DIVIDER_RATIO = BATTERY_R2 / (BATTERY_R1 + BATTERY_R2);
const float ADC_FSR = 1023.0;     // The maximum reading for the ADC
const float ADC_REF_VOLTS = 5.0;  // Reference voltage of ADC

const float BATTERY_MULTIPLIER = (ADC_REF_VOLTS / ADC_FSR / BATTERY_DIVIDER_RATIO);

const int MOTOR_MAX_PWM = 255;

