/** @file */
#ifndef ROBOT_H_
#define ROBOT_H_

#include "mbed.h"

extern int16_t countsLeft;
extern int16_t countsRight;

/** \brief Sets the speed for both motors.
 *
 * \param leftSpeed A number from -300 to 300 representing the speed and
 * direction of the left motor. Values of -300 or less result in full speed
 * reverse, and values of 300 or more result in full speed forward.
 * \param rightSpeed A number from -300 to 300 representing the speed and
 * direction of the right motor. Values of -300 or less result in full speed
 * reverse, and values of 300 or more result in full speed forward. */
void setSpeeds(int16_t leftSpeed, int16_t rightSpeed);

/** \brief Sets the speed for the left motor.
 *
 * \param speed A number from -300 to 300 representing the speed and
 * direction of the left motor.  Values of -300 or less result in full speed
 * reverse, and values of 300 or more result in full speed forward. */
void setLeftSpeed(int16_t speed);

/** \brief Sets the speed for the right motor.
 *
 * \param speed A number from -300 to 300 representing the speed and
 * direction of the right motor. Values of -300 or less result in full speed
 * reverse, and values of 300 or more result in full speed forward. */
void setRightSpeed(int16_t speed);

/*! Returns the number of counts that have been detected from the both
 * encoders.  These counts start at 0.  Positive counts correspond to forward
 * movement of the wheel of the Romi, while negative counts correspond
 * to backwards movement.
 *
 * The count is returned as a signed 16-bit integer.  When the count goes
 * over 32767, it will overflow down to -32768.  When the count goes below
 * -32768, it will overflow up to 32767. */
void getCounts();

/*! This function is just like getCounts() except it also clears the
 *  counts before returning.  If you call this frequently enough, you will
 *  not have to worry about the count overflowing. */
void getCountsAndReset();
//Pose estimation [time,omega,x,y,phi,v] -> [xk,yk,phik]

double limitSpeed(double speed);

#endif /* ROBOT_H_ */