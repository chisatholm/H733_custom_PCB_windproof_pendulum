/*
 * pendulum.c
 *
 * Created on: Mar 7, 2026
 * Author: andrewchisholm
 */

#include "pendulum.h"
#include <math.h>

/**
 * @brief State Transition Function: x_out = f(x_in, u)
 * States: [theta, omega, wind_bias]
 */
void Pendulum_Predict_F(arm_matrix_instance_f64 *x_out,
                        const arm_matrix_instance_f64 *x_in,
                        const arm_matrix_instance_f64 *u)
{
    float64_t theta 	= x_in->pData[0];
    float64_t theta_dot = x_in->pData[1];
    float64_t wind  	= x_in->pData[2];
    float64_t motor 	= u->pData[0];

    // Natural frequency (rad/s)
    float64_t wn = sqrt(PEND_MGL / PEND_I);

    // Damping torque coefficient: c = 2 * zeta * wn * I
    float64_t c = 2.0 * PEND_ZETA * wn * PEND_I;

    // Acceleration calculation
    // alpha = (Torque_gravity + Torque_damping + Torque_motor + Torque_wind) / I
    float64_t alpha = (-(PEND_MGL * sin(theta)) - (c * theta_dot) + motor + wind) / PEND_I;

    // Euler Integration
    x_out->pData[0] = theta + (theta_dot * PEND_DT);
    x_out->pData[1] = theta_dot + (alpha * PEND_DT);
    x_out->pData[2] = wind; // Wind bias modeled as a constant/random walk
}

/**
 * @brief Jacobian of the state transition: F = df/dx
 */
void Pendulum_Jacobian_F(arm_matrix_instance_f64 *F_out,
                         const arm_matrix_instance_f64 *x_in,
                         const arm_matrix_instance_f64 *u)
{
    float64_t theta = x_in->pData[0];
    float64_t wn = sqrt(PEND_MGL / PEND_I);
    float64_t c = 2.0 * PEND_ZETA * wn * PEND_I;

    // Row 0: d(next_theta)/dx
    F_out->pData[0] = 1.0;              // d/d_theta
    F_out->pData[1] = PEND_DT;          // d/d_theta_dot
    F_out->pData[2] = 0.0;              // d/d_wind

    // Row 1: d(next_theta_dot)/dx
    // d_alpha/d_theta = -(MGL/I) * cos(theta)
    F_out->pData[3] = -(PEND_MGL / PEND_I) * cos(theta) * PEND_DT;
    // d_alpha/d_theta_dot = -c/I
    F_out->pData[4] = 1.0 - (c / PEND_I) * PEND_DT;
    // d_alpha/d_wind = 1/I
    F_out->pData[5] = (1.0 / PEND_I) * PEND_DT;

    // Row 2: d(next_wind)/dx
    F_out->pData[6] = 0.0;
    F_out->pData[7] = 0.0;
    F_out->pData[8] = 1.0;
}

/**
 * @brief Observation Function: z = h(x)
 * Maps internal states to [encoder, gyro] measurements
 */
void Pendulum_Obs_H(arm_matrix_instance_f64 *z_out,
                    const arm_matrix_instance_f64 *x_in)
{
    z_out->pData[0] = x_in->pData[0]; // Encoder measures theta directly
    z_out->pData[1] = x_in->pData[1]; // Gyro measures omega directly
}

/**
 * @brief Jacobian of the observation function: H = dh/dx
 */
void Pendulum_Jacobian_H(arm_matrix_instance_f64 *H_out,
                         const arm_matrix_instance_f64 *x_in)
{
    // Row 0: d(encoder)/dx
    H_out->pData[0] = 1.0; H_out->pData[1] = 0.0; H_out->pData[2] = 0.0;

    // Row 1: d(gyro)/dx
    H_out->pData[3] = 0.0; H_out->pData[4] = 1.0; H_out->pData[5] = 0.0;
}
