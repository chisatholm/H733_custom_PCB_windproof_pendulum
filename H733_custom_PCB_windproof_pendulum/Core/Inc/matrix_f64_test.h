/*
 * matrix_f64_test.h
 *
 *  Created on: Mar 3, 2026
 *      Author: andrewchisholm
 */

#ifndef MATRIX_F64_TEST_H
#define MATRIX_F64_TEST_H

#include "matrix_f64.h"

/* Shared Test Data - Reuse for ALL Tests (Live Expressions) */
extern float64_t test_A_data[9];    // 3x3 input matrix A
extern float64_t test_B_data[9];    // 3x3 input matrix B
extern float64_t test_C_data[9];    // 3x3 output matrix C
extern arm_status test_status;      // ARM_MATH_SUCCESS = 0

/* Individual Test Functions - Call One At A Time */
void test_mat_init_f64(void);
void test_mat_mult_f64(void);
void test_mat_trans_f64(void);
void test_mat_add_f64(void);
void test_mat_sub_f64(void);
void test_mat_inverse_f64(void);

/* Master Test Function */
void run_all_matrix_f64_tests(void);

#endif /* MATRIX_F64_TEST_H */

