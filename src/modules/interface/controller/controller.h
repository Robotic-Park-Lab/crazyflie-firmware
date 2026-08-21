/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * controller.h - Controller interface
 */
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "stabilizer_types.h"

typedef enum {
  ControllerTypeAutoSelect,
  ControllerTypePID,
  ControllerTypeMellinger,
  ControllerTypeINDI,
  ControllerTypeBrescianini,
  ControllerTypeLee,
#ifdef CONFIG_CONTROLLER_OOT
  ControllerTypeOot,
#endif
  ControllerType_COUNT,
} ControllerType;

void controllerInit(ControllerType controller);
bool controllerTest(void);
void controller(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);
ControllerType controllerGetType(void);
const char* controllerGetName();

// Event-based Control
typedef struct threshold_s{
  float co, ai, cn;
  float dt;
  float noise[20];
  float last_signal;
  int16_t count;
}threshold;
#ifdef CONFIG_CONTROLLER_OOT
// Event-based Control
threshold init_triggering(float co, float ai);
bool eval_threshold(threshold *trigger, float signal, float ref);
// Multi-Robot App
void update_agent_pose(float name, float x, float y, float z);
void add_new_agent(float name, float pose[], float d, float k);
void remove_neighbour(float name);
void update_distance(float name, float d);
void target_pose(float pose[]);
void enable_formation(void);
// Gimbal App
void attitud_cmd(float roll, float pitch, float yaw, float thrust);
void attituderate_cmd(float roll, float pitch, float yaw, float thrust);
void enable_relay(void);
float update_attitude_relay(float setpoint, float value);
void relay_params(uint8_t level, uint8_t angle, float cmd, float threshold);

// General
void controller_params(uint8_t level, uint8_t angle, float kp, float ki, float kd, float co);
void controllerOutOfTreeInit(void);
bool controllerOutOfTreeTest(void);
void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep);
#endif

#endif //__CONTROLLER_H__
