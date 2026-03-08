/*
 * ekf.h
 *
 *  Created on: Mar 1, 2026
 *      Author: andrewchisholm
 */
#ifndef EKF_H_
#define EKF_H_

#ifndef ARM_MATH_CM7
#define ARM_MATH_CM7
#endif

#include "arm_math.h"
#include <string.h>
#include "main.h"

/**
 * @brief EKF Handle Structure using Double Precision (float64_t)
 * Optimized for the STM32H733VGT's DP-FPU.
 */
typedef struct {
    // --- Dimensions ---
    uint16_t n;      // Number of states (3: theta, omega, wind)
    uint16_t m;      // Number of measurements (2: encoder, gyro)
    uint16_t u_dim;  // Number of inputs (e.g., motor torque)

    // --- State and Covariance ---
    arm_matrix_instance_f64 x; // State vector [n x 1]
    arm_matrix_instance_f64 P; // State covariance [n x n]

    // --- Noise Covariances ---
    arm_matrix_instance_f64 Q; // Process noise covariance [n x n]
    arm_matrix_instance_f64 R; // Measurement noise covariance [m x m]

    // --- Model Function Pointers (Physics Callbacks) ---
    void (*predict_f)(arm_matrix_instance_f64 *x_out, const arm_matrix_instance_f64 *x_in, const arm_matrix_instance_f64 *u);
    void (*obs_h)(arm_matrix_instance_f64 *z_out, const arm_matrix_instance_f64 *x_in);
    void (*jacob_F)(arm_matrix_instance_f64 *F_out, const arm_matrix_instance_f64 *x_in, const arm_matrix_instance_f64 *u);
    void (*jacob_H)(arm_matrix_instance_f64 *H_out, const arm_matrix_instance_f64 *x_in);

    /* * --- Internal Scratchpad Buffers ---
     * These are pointers to memory used in pendulum_model.c
     * to avoid using malloc() during real-time execution.
     */
    arm_matrix_instance_f64 F;      // State transition Jacobian [n x n]
    arm_matrix_instance_f64 H;      // Observation Jacobian [m x n]
    arm_matrix_instance_f64 K;      // Kalman Gain [n x m]
    arm_matrix_instance_f64 I_KH;   // (Identity - K*H) [n x n]

    // Additional buffers for Joseph Form: P = (I-KH)P(I-KH)' + KRK'
    arm_matrix_instance_f64 temp_nn; // Size [n x n]
    arm_matrix_instance_f64 temp_nm; // Size [n x m]
    arm_matrix_instance_f64 temp_mn; // Size [m x n]
    arm_matrix_instance_f64 temp_mm; // Size [m x m]

} EKF_Handle_t;

/* --- API Functions --- */

/**
 * @brief Initializes the EKF handle dimensions and internal matrix structures.
 * This should be called before the main control loop.
 */
void EKF_Init(EKF_Handle_t *ekf, uint16_t n, uint16_t m, uint16_t u_dim);

/**
 * @brief Performs the EKF Prediction step (State extrapolation).
 */
arm_status EKF_Predict(EKF_Handle_t *ekf, arm_matrix_instance_f64 *u);

/**
 * @brief Performs the EKF Update step (Measurement correction).
 * Implements the numerically stable Joseph Form for the covariance update.
 */
arm_status EKF_Update(EKF_Handle_t *ekf, arm_matrix_instance_f64 *z_meas);

#endif /* EKF_H_ */
