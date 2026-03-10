/*
 * pendulum_model.h
 *
 *  Created on: Mar 9, 2026
 *      Author: andrewchisholm
 */

#ifndef PENDULUM_MODEL_H_
#define PENDULUM_MODEL_H_

#include "ekf.h"

// Export the handle so the control loop in main.c or sensor_manager.c can access it
extern EKF_Handle_t hpendulum_ekf;

// The one-time setup function
void PENDULUM_Model_Init(void);

#endif
