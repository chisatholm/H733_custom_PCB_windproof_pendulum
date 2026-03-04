/*
 * matrix_f64_test.c
 *
 *  Created on: Mar 3, 2026
 *      Author: andrewchisholm
 */
#include "matrix_f64_test.h"

/* Shared Test Data - Reuse for ALL Tests (Live Expressions) */
float64_t test_A_data[9] = {1.000000000001, 2.12345678987654321, 3.0,  4.0, 5.0, 6.0,  7.0, 8.0, 9.0};
float64_t test_B_data[9] = {9.0, 8.0, 7.0,  6.0, 5.0, 4.0,  3.0, 2.0, 1.0};
float64_t test_C_data[9];
arm_status test_status;

/* Test Matrices */
static arm_matrix_instance_f64 A = {3, 3, test_A_data};
static arm_matrix_instance_f64 B = {3, 3, test_B_data};
static arm_matrix_instance_f64 C = {3, 3, test_C_data};

/* Individual Test Functions */
void test_mat_init_f64(void) {
    arm_matrix_instance_f64 S;
    test_status = arm_mat_init_f64(&S, 3, 3, test_C_data);
    // Copies A data to C, verifies init works
    for(int i = 0; i < 9; i++) {
        test_C_data[i] = test_A_data[i];
    }
}

void test_mat_mult_f64(void) {
    test_status = arm_mat_mult_f64(&A, &B, &C);
}

void test_mat_trans_f64(void) {
    test_status = arm_mat_trans_f64(&A, &C);
}

void test_mat_add_f64(void) {
    test_status = arm_mat_add_f64(&A, &B, &C);
}

void test_mat_sub_f64(void) {
    test_status = arm_mat_sub_f64(&A, &B, &C);
}

void test_mat_inverse_f64(void) {
    test_status = arm_mat_inverse_f64(&A, &C);
}

/* Master Test Function - Call from main.c */
void run_all_matrix_f64_tests(void) {
    test_mat_init_f64();
    test_mat_mult_f64();
    test_mat_trans_f64();
    test_mat_add_f64();
    test_mat_sub_f64();
    test_mat_inverse_f64();
}


