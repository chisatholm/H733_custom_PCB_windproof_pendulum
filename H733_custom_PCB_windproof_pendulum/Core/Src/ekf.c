/*
 * ekf.c
 *
 *  Created on: Mar 1, 2026
 *      Author: andrewchisholm
 */
#include "ekf.h"

/**
 * @brief Initialize matrix instances with user-provided buffers.
 * This function "connects" the EKF handle to the static arrays defined in your model.
 */
void EKF_Init(EKF_Handle_t *ekf, uint16_t n, uint16_t m, uint16_t u_dim) {
    ekf->n = n;
    ekf->m = m;
    ekf->u_dim = u_dim;
    // The actual data pointers for ekf->x.pData, ekf->P.pData, etc.,
    // must be assigned in pendulum_model.c before calling this.
}

/**
 * @brief Prediction Step: x = f(x, u) and P = F*P*F' + Q
 */
arm_status EKF_Predict(EKF_Handle_t *ekf, arm_matrix_instance_f64 *u) {
    arm_status status;

    // 1. x_predict = f(x, u)
    ekf->predict_f(&ekf->x, &ekf->x, u);

    // 2. F = Jacobian of f
    ekf->jacob_F(&ekf->F, &ekf->x, u);

    // 3. P = F * P * F' + Q
    // temp_nn = F * P
    status = arm_mat_mult_f64(&ekf->F, &ekf->P, &ekf->temp_nn);
    if (status != ARM_MATH_SUCCESS) return status;

    // P = (temp_nn * F') + Q
    // Note: Using temp_nm as a temporary for F' (it's n x n here)
    arm_matrix_instance_f64 Ft = ekf->temp_nm;
    status = arm_mat_trans_f64(&ekf->F, &Ft);
    if (status != ARM_MATH_SUCCESS) return status;

    status = arm_mat_mult_f64(&ekf->temp_nn, &Ft, &ekf->P);
    if (status != ARM_MATH_SUCCESS) return status;

    status = arm_mat_add_f64(&ekf->P, &ekf->Q, &ekf->P);

    return status;
}

/**
 * @brief Update Step: Joseph Form P = (I - KH)P(I - KH)' + KRK'
 */
arm_status EKF_Update(EKF_Handle_t *ekf, arm_matrix_instance_f64 *z_meas) {
    arm_status status;

    // 1. H = Jacobian of h
    ekf->jacob_H(&ekf->H, &ekf->x);

    // 2. Innovation: y = z - h(x)
    // Reuse temp_mm for innovation vector (m x 1)
    float64_t z_pred_data[ekf->m];
    arm_matrix_instance_f64 z_pred = {ekf->m, 1, z_pred_data};
    ekf->obs_h(&z_pred, &ekf->x);

    float64_t y_data[ekf->m];
    arm_matrix_instance_f64 y = {ekf->m, 1, y_data};
    status = arm_mat_sub_f64(z_meas, &z_pred, &y);

    // 3. Innovation Covariance: S = HPH' + R
    // temp_mn = H * P
    status = arm_mat_mult_f64(&ekf->H, &ekf->P, &ekf->temp_mn);

    // temp_mm = (H*P) * H'
    arm_matrix_instance_f64 Ht = ekf->temp_nm; // Reuse nm as H'
    arm_mat_trans_f64(&ekf->H, &Ht);
    status = arm_mat_mult_f64(&ekf->temp_mn, &Ht, &ekf->temp_mm);

    // S = temp_mm + R
    status = arm_mat_add_f64(&ekf->temp_mm, &ekf->R, &ekf->temp_mm);

    // 4. Kalman Gain: K = PH'S^-1
    // Invert S (temp_mm)
    float64_t S_inv_data[ekf->m * ekf->m];
    arm_matrix_instance_f64 S_inv = {ekf->m, ekf->m, S_inv_data};
    status = arm_mat_inverse_f64(&ekf->temp_mm, &S_inv);
    if (status != ARM_MATH_SUCCESS) return status;

    // K = (P * H') * S_inv -> temp_nm * S_inv
    arm_matrix_instance_f64 PHt = ekf->temp_nm;
    arm_mat_mult_f64(&ekf->P, &Ht, &PHt);
    status = arm_mat_mult_f64(&PHt, &S_inv, &ekf->K);

    // 5. Update State: x = x + Ky
    float64_t Ky_data[ekf->n];
    arm_matrix_instance_f64 Ky = {ekf->n, 1, Ky_data};
    arm_mat_mult_f64(&ekf->K, &y, &Ky);
    arm_mat_add_f64(&ekf->x, &Ky, &ekf->x);

    // 6. Joseph Form Covariance Update: P = (I-KH)P(I-KH)' + KRK'
    // I_KH = I - K*H
    float64_t identity_data[ekf->n * ekf->n];
    memset(identity_data, 0, sizeof(identity_data));
    for(int i=0; i<ekf->n; i++) identity_data[i*ekf->n + i] = 1.0;
    arm_matrix_instance_f64 I = {ekf->n, ekf->n, identity_data};

    arm_mat_mult_f64(&ekf->K, &ekf->H, &ekf->temp_nn); // temp_nn = KH
    arm_mat_sub_f64(&I, &ekf->temp_nn, &ekf->I_KH);    // I_KH = I - KH

    // P = I_KH * P * I_KH'
    arm_mat_mult_f64(&ekf->I_KH, &ekf->P, &ekf->temp_nn); // temp_nn = (I-KH)P
    arm_matrix_instance_f64 I_KHt = ekf->temp_nm;         // Reuse temp_nm
    arm_mat_trans_f64(&ekf->I_KH, &I_KHt);
    arm_mat_mult_f64(&ekf->temp_nn, &I_KHt, &ekf->P);    // P = (I-KH)P(I-KH)'

    // P = P + K*R*K'
    arm_mat_mult_f64(&ekf->K, &ekf->R, &ekf->temp_nm);   // temp_nm = KR
    arm_matrix_instance_f64 Kt = ekf->temp_mn;
    arm_mat_trans_f64(&ekf->K, &Kt);
    arm_mat_mult_f64(&ekf->temp_nm, &Kt, &ekf->temp_nn); // temp_nn = KRK'
    arm_mat_add_f64(&ekf->P, &ekf->temp_nn, &ekf->P);

    // 7. Force Symmetry
    arm_mat_trans_f64(&ekf->P, &ekf->temp_nn);
    for(int i=0; i < (ekf->n * ekf->n); i++) {
        ekf->P.pData[i] = (ekf->P.pData[i] + ekf->temp_nn.pData[i]) * 0.5;
    }

    // 8. Covariance Clamping (The "Over-confidence" Guard)
	// Prevents variances from dropping below a threshold (e.g., 1e-12)
	const float64_t P_MIN = 1e-12;
	for(int i = 0; i < ekf->n; i++) {
		int diag_idx = i * ekf->n + i;
		if (ekf->P.pData[diag_idx] < P_MIN) {
			ekf->P.pData[diag_idx] = P_MIN;
		}
	}

    return ARM_MATH_SUCCESS;
}

