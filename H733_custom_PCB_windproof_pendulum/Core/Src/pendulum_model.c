/*
 * pendulum_model.c
 *
 *  Created on: Mar 9, 2026
 *      Author: andrewchisholm
 */


#include "pendulum.h"
#include "ekf.h"

// 1. Create the Global EKF Handle
EKF_Handle_t hpendulum_ekf;

// 2. Allocate Static Buffers (Hidden from other files via 'static')
static float64_t x_data[PEND_STATE_DIM];
static float64_t P_data[PEND_STATE_DIM * PEND_STATE_DIM];
static float64_t Q_data[PEND_STATE_DIM * PEND_STATE_DIM];
static float64_t R_data[PEND_MEAS_DIM * PEND_MEAS_DIM];

// Scratchpads
static float64_t F_data[PEND_STATE_DIM * PEND_STATE_DIM];
static float64_t H_data[PEND_MEAS_DIM * PEND_STATE_DIM];
static float64_t K_data[PEND_STATE_DIM * PEND_MEAS_DIM];
static float64_t I_KH_data[PEND_STATE_DIM * PEND_STATE_DIM];
static float64_t t_nn_data[PEND_STATE_DIM * PEND_STATE_DIM];
static float64_t t_nm_data[PEND_STATE_DIM * PEND_MEAS_DIM];
static float64_t t_mn_data[PEND_MEAS_DIM * PEND_STATE_DIM];
static float64_t t_mm_data[PEND_MEAS_DIM * PEND_MEAS_DIM];

void PENDULUM_Model_Init(void) {
    // Link Pointers to the Handle
    hpendulum_ekf.x.pData = x_data;     hpendulum_ekf.x.numRows = PEND_STATE_DIM; hpendulum_ekf.x.numCols = 1;
    hpendulum_ekf.P.pData = P_data;     hpendulum_ekf.P.numRows = PEND_STATE_DIM; hpendulum_ekf.P.numCols = PEND_STATE_DIM;
    hpendulum_ekf.Q.pData = Q_data;     hpendulum_ekf.Q.numRows = PEND_STATE_DIM; hpendulum_ekf.Q.numCols = PEND_STATE_DIM;
    hpendulum_ekf.R.pData = R_data;     hpendulum_ekf.R.numRows = PEND_MEAS_DIM;  hpendulum_ekf.R.numCols = PEND_MEAS_DIM;

    hpendulum_ekf.F.pData = F_data;     hpendulum_ekf.H.pData = H_data;
    hpendulum_ekf.K.pData = K_data;     hpendulum_ekf.I_KH.pData = I_KH_data;

    hpendulum_ekf.temp_nn.pData = t_nn_data; hpendulum_ekf.temp_nm.pData = t_nm_data;
    hpendulum_ekf.temp_mn.pData = t_mn_data; hpendulum_ekf.temp_mm.pData = t_mm_data;

    // Assign Physics Callbacks from pendulum.c
    hpendulum_ekf.predict_f = Pendulum_Predict_F;
    hpendulum_ekf.jacob_F   = Pendulum_Jacobian_F;
    hpendulum_ekf.obs_h     = Pendulum_Obs_H;
    hpendulum_ekf.jacob_H   = Pendulum_Jacobian_H;

    // Call the generic EKF Dimension Init
    EKF_Init(&hpendulum_ekf, PEND_STATE_DIM, PEND_MEAS_DIM, PEND_INPUT_DIM);

    // Initialize Covariances (Initial Guesses)
    memset(P_data, 0, sizeof(P_data));
    P_data[0] = 0.1; P_data[4] = 0.1; P_data[8] = 0.1; // Initial uncertainty

    // Tuning: Process Noise (How much we trust the physics)
    memset(Q_data, 0, sizeof(Q_data));
    Q_data[0] = 1e-4; Q_data[4] = 1e-3; Q_data[8] = 1e-2;

    // Tuning: Measurement Noise (How much we trust the sensors)
    memset(R_data, 0, sizeof(R_data));
    R_data[0] = 1e-5; // Trust Encoder (Position)
    R_data[3] = 1e-3; // Trust Gyro (Velocity)
}
