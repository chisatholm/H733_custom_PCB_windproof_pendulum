	/*
	 * pendulum.h
	 *
	 *  Created on: Mar 7, 2026
	 *      Author: andrewchisholm
	 */


	#ifndef PENDULUM_H_
	#define PENDULUM_H_

	#include "arm_math.h"
	#include "ekf.h"

	/* --- Directly Measured Physics Constants 	--- */
	static const float64_t PEND_MGL				= (0.178 * 9.81 * 0.13); // Nm, measured torque required to hold pendulum at 90 deg to vertical
	static const float64_t PEND_PERIOD			= 1.28; // seconds

	/* --- Inferred Physics Constants 			--- */
	static const float64_t PEND_ZETA 			= 0.1; // Damping coefficient, dimensionless [to be evaluated empirically]

	/* --- Calculated Physics Constants 		--- */
	static const float64_t PEND_I  				= (PEND_MGL * PEND_PERIOD * PEND_PERIOD) / (4.0 * M_PI * M_PI); // Moment of inertia, kg.m^2

	/* --- Engineering Decisions 				--- */
	static const float64_t PEND_DT 				= 0.002; // 500Hz Sample Rate

	/* --- Sensor Calibration 					--- */
	#define AS5048A_ZERO_OFFSET    14692    // Raw value representing 0 radians
	#define AS5048A_RANGE          16384    // 14-bit resolution
	#define AS5048A_DIRECTION      1.0f     // 1.0 for CCW positive, -1.0 for CW positive
	#define BMI270_DIRECTION      -1.0f     // Negate to align with Right-Hand Rule

	/* --- EKF Dimension Definitions --- */
	#define PEND_STATE_DIM    3    // [theta, omega, wind_bias]
	#define PEND_MEAS_DIM     2    // [encoder_angle, gyro_velocity]
	#define PEND_INPUT_DIM    1    // [motor_torque]

	/* --- Function Prototypes --- */
	// These are the "Plugs" that will go into the EKF sockets

	void Pendulum_Predict_F(arm_matrix_instance_f64 *x_out,
							const arm_matrix_instance_f64 *x_in,
							const arm_matrix_instance_f64 *u);

	void Pendulum_Jacobian_F(arm_matrix_instance_f64 *F_out,
							 const arm_matrix_instance_f64 *x_in,
							 const arm_matrix_instance_f64 *u);

	void Pendulum_Obs_H(arm_matrix_instance_f64 *z_out,
						const arm_matrix_instance_f64 *x_in);

	void Pendulum_Jacobian_H(arm_matrix_instance_f64 *H_out,
							 const arm_matrix_instance_f64 *x_in);

	#endif /* PENDULUM_H_ */
