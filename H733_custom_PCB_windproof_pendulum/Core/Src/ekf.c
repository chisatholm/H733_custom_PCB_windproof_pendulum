/*
 * ekf.c
 *
 * Created on: Mar 1, 2026
 * Author: andrewchisholm
 */
#include "ekf.h"

/**
 * @brief Workaround for missing arm_mat_add_f64 in some CMSIS-DSP f64 libraries.
 * Performs element-wise addition: pDst = pSrcA + pSrcB
 */
static arm_status manual_mat_add_f64(const arm_matrix_instance_f64 *pSrcA,
                                     const arm_matrix_instance_f64 *pSrcB,
                                           arm_matrix_instance_f64 *pDst)
{
    if ((pSrcA->numRows != pSrcB->numRows) || (pSrcA->numCols != pSrcB->numCols) ||
        (pSrcA->numRows != pDst->numRows)  || (pSrcA->numCols != pDst->numCols)) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    uint32_t totalElements = pSrcA->numRows * pSrcA->numCols;
    float64_t *pA = pSrcA->pData;
    float64_t *pB = pSrcB->pData;
    float64_t *pOut = pDst->pData;

    for (uint32_t i = 0; i < totalElements; i++) {
        pOut[i] = pA[i] + pB[i];
    }

    return ARM_MATH_SUCCESS;
}

/**
 * @brief Initialize matrix instances with user-provided buffers.
 */
void EKF_Init(EKF_Handle_t *ekf, uint16_t n, uint16_t m, uint16_t u_dim) {
    ekf->n = n;
    ekf->m = m;
    ekf->u_dim = u_dim;
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
    status = arm_mat_mult_f64(&ekf->F, &ekf->P, &ekf->temp_nn);
    if (status != ARM_MATH_SUCCESS) return status;

    arm_matrix_instance_f64 Ft = ekf->temp_nm;
    status = arm_mat_trans_f64(&ekf->F, &Ft);
    if (status != ARM_MATH_SUCCESS) return status;

    status = arm_mat_mult_f64(&ekf->temp_nn, &Ft, &ekf->P);
    if (status != ARM_MATH_SUCCESS) return status;

    // Use manual add workaround
    status = manual_mat_add_f64(&ekf->P, &ekf->Q, &ekf->P);

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
    float64_t z_pred_data[ekf->m];
    arm_matrix_instance_f64 z_pred = {ekf->m, 1, z_pred_data};
    ekf->obs_h(&z_pred, &ekf->x);

    float64_t y_data[ekf->m];
    arm_matrix_instance_f64 y = {ekf->m, 1, y_data};
    status = arm_mat_sub_f64(z_meas, &z_pred, &y);
    if (status != ARM_MATH_SUCCESS) return status;

    // 3. Innovation Covariance: S = HPH' + R
    status = arm_mat_mult_f64(&ekf->H, &ekf->P, &ekf->temp_mn);
    if (status != ARM_MATH_SUCCESS) return status;

    arm_matrix_instance_f64 Ht = ekf->temp_nm;
    arm_mat_trans_f64(&ekf->H, &Ht);
    status = arm_mat_mult_f64(&ekf->temp_mn, &Ht, &ekf->temp_mm);
    if (status != ARM_MATH_SUCCESS) return status;

    // S = temp_mm + R (Manual add)
    status = manual_mat_add_f64(&ekf->temp_mm, &ekf->R, &ekf->temp_mm);
    if (status != ARM_MATH_SUCCESS) return status;

    // 4. Kalman Gain: K = PH'S^-1
    float64_t S_inv_data[ekf->m * ekf->m];
    arm_matrix_instance_f64 S_inv = {ekf->m, ekf->m, S_inv_data};
    status = arm_mat_inverse_f64(&ekf->temp_mm, &S_inv);
    if (status != ARM_MATH_SUCCESS) return status;

    arm_matrix_instance_f64 PHt = ekf->temp_nm;
    status = arm_mat_mult_f64(&ekf->P, &Ht, &PHt);
    if (status != ARM_MATH_SUCCESS) return status;
    status = arm_mat_mult_f64(&PHt, &S_inv, &ekf->K);
    if (status != ARM_MATH_SUCCESS) return status;

    // 5. Update State: x = x + Ky
    float64_t Ky_data[ekf->n];
    arm_matrix_instance_f64 Ky = {ekf->n, 1, Ky_data};
    status = arm_mat_mult_f64(&ekf->K, &y, &Ky);
    if (status != ARM_MATH_SUCCESS) return status;
    status = manual_mat_add_f64(&ekf->x, &Ky, &ekf->x); // Manual add
    if (status != ARM_MATH_SUCCESS) return status;

    // 6. Joseph Form Covariance Update: P = (I-KH)P(I-KH)' + KRK'
    float64_t identity_data[ekf->n * ekf->n];
    memset(identity_data, 0, sizeof(identity_data));
    for(int i=0; i<ekf->n; i++) identity_data[i*ekf->n + i] = 1.0;
    arm_matrix_instance_f64 I = {ekf->n, ekf->n, identity_data};

    status = arm_mat_mult_f64(&ekf->K, &ekf->H, &ekf->temp_nn);
    if (status != ARM_MATH_SUCCESS) return status;
    status = arm_mat_sub_f64(&I, &ekf->temp_nn, &ekf->I_KH);
    if (status != ARM_MATH_SUCCESS) return status;

    status = arm_mat_mult_f64(&ekf->I_KH, &ekf->P, &ekf->temp_nn);
    if (status != ARM_MATH_SUCCESS) return status;
    arm_matrix_instance_f64 I_KHt = ekf->temp_nm;
    arm_mat_trans_f64(&ekf->I_KH, &I_KHt);
    status = arm_mat_mult_f64(&ekf->temp_nn, &I_KHt, &ekf->P);
    if (status != ARM_MATH_SUCCESS) return status;

    // P = P + K*R*K'
    status = arm_mat_mult_f64(&ekf->K, &ekf->R, &ekf->temp_nm);
    if (status != ARM_MATH_SUCCESS) return status;
    arm_matrix_instance_f64 Kt = ekf->temp_mn;
    arm_mat_trans_f64(&ekf->K, &Kt);
    status = arm_mat_mult_f64(&ekf->temp_nm, &Kt, &ekf->temp_nn);
    if (status != ARM_MATH_SUCCESS) return status;
    status = manual_mat_add_f64(&ekf->P, &ekf->temp_nn, &ekf->P); // Manual add
    if (status != ARM_MATH_SUCCESS) return status;

    // 7. Force Symmetry (Manual element access is safe here)
    arm_mat_trans_f64(&ekf->P, &ekf->temp_nn);
    for(int i=0; i < (ekf->n * ekf->n); i++) {
        ekf->P.pData[i] = (ekf->P.pData[i] + ekf->temp_nn.pData[i]) * 0.5;
    }

    // 8. Covariance Clamping
    const float64_t P_MIN = 1e-12;
    for(int i = 0; i < ekf->n; i++) {
        int diag_idx = i * ekf->n + i;
        if (ekf->P.pData[diag_idx] < P_MIN) {
            ekf->P.pData[diag_idx] = P_MIN;
        }
    }

    return ARM_MATH_SUCCESS;
}
