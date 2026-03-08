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

	/* --- Physics Constants --- */
	static const float64_t PEND_ZETA 	= 0.1;     // Damping coefficient, dimensionless
	static const float64_t PEND_DT 		= 0.002;   // 500Hz Sample Rate
	static const float64_t PEND_I  		= 1.0; 	   // Moment of inertia, kg.m^2
	static const float64_t PEND_MGL		= 1.0;     // kg.m^2.s^-2: pendulum mass x gravitational accel x distance of CofG below pivot
	// MGL seems like an odd measure to use, but it is the torque per radian displacement from centre, so it is actually convenient

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
