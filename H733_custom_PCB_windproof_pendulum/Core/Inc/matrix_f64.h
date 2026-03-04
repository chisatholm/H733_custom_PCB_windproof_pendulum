/*
 * matrix_f64.h
 *
 *  Created on: Mar 3, 2026
 *      Author: andrewchisholm
 */

#ifndef INC_MATRIX_F64_H_
#define INC_MATRIX_F64_H_

#include "arm_math.h"

/* CMSIS-DSP Double Precision Matrix Function Prototypes */
/* These silence compiler warnings when linking libarm_cortexM7lfsp_math.a */

arm_status arm_mat_init_f64(
    arm_matrix_instance_f64 * S,
    uint16_t nRows,
    uint16_t nColumns,
    float64_t * pData);

arm_status arm_mat_mult_f64(
    const arm_matrix_instance_f64 * pSrcA,
    const arm_matrix_instance_f64 * pSrcB,
    arm_matrix_instance_f64 * pDst);

arm_status arm_mat_trans_f64(
    const arm_matrix_instance_f64 * pSrc,
    arm_matrix_instance_f64 * pDst);

arm_status arm_mat_add_f64(
    const arm_matrix_instance_f64 * pSrcA,
    const arm_matrix_instance_f64 * pSrcB,
    arm_matrix_instance_f64 * pDst);

arm_status arm_mat_sub_f64(
    const arm_matrix_instance_f64 * pSrcA,
    const arm_matrix_instance_f64 * pSrcB,
    arm_matrix_instance_f64 * pDst);

arm_status arm_mat_inverse_f64(
    const arm_matrix_instance_f64 * src,
    arm_matrix_instance_f64 * dst);


#endif /* INC_MATRIX_F64_H_ */
