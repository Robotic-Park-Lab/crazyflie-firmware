/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 * position_estimator_pid.c: PID-based implementation of the position controller
 */

#include <math.h>
#include "num.h"

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_eventbased_controller.h"

struct pidInit_s {
  float kp;
  float ki;
  float kd;
};

struct pidAxis_s {
  PidObject pid;

  struct pidInit_s init;
    stab_mode_t previousMode;
  float setpoint;
  float output;
  uint16_t count;
  float last_hold;
  float co;
  float ai;
};

struct this_s {
  struct pidAxis_s pidVX;
  struct pidAxis_s pidVY;
  struct pidAxis_s pidVZ;

  struct pidAxis_s pidX;
  struct pidAxis_s pidY;
  struct pidAxis_s pidZ;

  uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
  uint16_t thrustMin;  // Minimum thrust value to output
};

// Maximum roll/pitch angle permited
static float rLimit  = 20;
static float pLimit  = 20;
static float rpLimitOverhead = 1.10f;
// Velocity maximums
static float xVelMax = 1.0f;
static float yVelMax = 1.0f;
static float zVelMax  = 1.0f;
static float velMaxOverhead = 1.10f;
static const float thrustScale = 1000.0f;

// Feedforward gains
static float kFFx = 0.0; // feedforward gain for x direction [deg / m/s]
static float kFFy = 0.0; // feedforward gain for y direction [deg / m/s]

#define DT (float)(1.0f/POSITION_RATE)
bool posEBFiltEnable = true;
bool velEBFiltEnable = true;
float posEBFiltCutoff = 20.0f;
float velEBFiltCutoff = 20.0f;
bool posEBZFiltEnable = true;
bool velEBZFiltEnable = true;
float posEBZFiltCutoff = 20.0f;
#ifdef IMPROVED_BARO_Z_HOLD
float velEBZFiltCutoff = 0.7f;
#else
float velEBZFiltCutoff = 20.0f;
#endif

#ifndef UNIT_TEST
static struct this_s this = {
  .pidVX = {
    .init = {
      .kp = 25.0f,
      .ki = 1.0f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
    .co = 0.01f,
    .ai = 0.01f,
  },

  .pidVY = {
    .init = {
      .kp = 25.0f,
      .ki = 1.0f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
    .co = 0.01f,
    .ai = 0.01f,
  },
  #ifdef IMPROVED_BARO_Z_HOLD
    .pidVZ = {
      .init = {
        .kp = 3.0f,
        .ki = 1.0f,
        .kd = 1.5f, //kd can be lowered for improved stability, but results in slower response time.
      },
      .pid.dt = DT,
      .co = 0.01f,
      .ai = 0.01f,
    },
  #else
    .pidVZ = {
      .init = {
        .kp = 25.0f,
        .ki = 15.0f,
        .kd = 0,
      },
      .pid.dt = DT,
      .co = 0.01f,
      .ai = 0.01f,
    },
  #endif
  .pidX = {
    .init = {
      .kp = 2.0f,
      .ki = 0.0f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
    .co = 0.01f,
    .ai = 0.01f,
  },

  .pidY = {
    .init = {
      .kp = 2.0f,
      .ki = 0.0f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
    .co = 0.01f,
    .ai = 0.01f,
  },

  .pidZ = {
    .init = {
      .kp = 2.0f,
      .ki = 0.5f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
    .co = 0.01f,
    .ai = 0.01f,
    .count = 1.0f,
    .last_hold = 0.0f,
  },
  #ifdef IMPROVED_BARO_Z_HOLD
    .thrustBase = 38000,
  #else
    .thrustBase = 36000,
  #endif
  .thrustMin  = 20000,
};
#endif

void positionEBControllerInit()
{
  pidInit(&this.pidX.pid, this.pidX.setpoint, this.pidX.init.kp, this.pidX.init.ki, this.pidX.init.kd,
      this.pidX.pid.dt, POSITION_RATE, posEBFiltCutoff, posEBFiltEnable);
  pidInit(&this.pidY.pid, this.pidY.setpoint, this.pidY.init.kp, this.pidY.init.ki, this.pidY.init.kd,
      this.pidY.pid.dt, POSITION_RATE, posEBFiltCutoff, posEBFiltEnable);
  pidInit(&this.pidZ.pid, this.pidZ.setpoint, this.pidZ.init.kp, this.pidZ.init.ki, this.pidZ.init.kd,
      this.pidZ.pid.dt, POSITION_RATE, posEBZFiltCutoff, posEBZFiltEnable);

  pidInit(&this.pidVX.pid, this.pidVX.setpoint, this.pidVX.init.kp, this.pidVX.init.ki, this.pidVX.init.kd,
      this.pidVX.pid.dt, POSITION_RATE, velEBFiltCutoff, velEBFiltEnable);
  pidInit(&this.pidVY.pid, this.pidVY.setpoint, this.pidVY.init.kp, this.pidVY.init.ki, this.pidVY.init.kd,
      this.pidVY.pid.dt, POSITION_RATE, velEBFiltCutoff, velEBFiltEnable);
  pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.init.kp, this.pidVZ.init.ki, this.pidVZ.init.kd,
      this.pidVZ.pid.dt, POSITION_RATE, velEBZFiltCutoff, velEBZFiltEnable);
}

static float runPid(float input, struct pidAxis_s *axis, float setpoint, float dt) {
  axis->setpoint = setpoint;
  axis->pid.dt = dt;

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate(&axis->pid, input, true);
}


float state_body_x, state_body_y, state_body_vx, state_body_vy;

void positionEBController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  this.pidX.pid.outputLimit = xVelMax * velMaxOverhead;
  this.pidY.pid.outputLimit = yVelMax * velMaxOverhead;
  // The ROS landing detector will prematurely trip if
  // this value is below 0.5
  this.pidZ.pid.outputLimit = fmaxf(zVelMax, 0.5f)  * velMaxOverhead;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);

  float setp_body_x = setpoint->position.x * cosyaw + setpoint->position.y * sinyaw;
  float setp_body_y = -setpoint->position.x * sinyaw + setpoint->position.y * cosyaw;

  state_body_x = state->position.x * cosyaw + state->position.y * sinyaw;
  state_body_y = -state->position.x * sinyaw + state->position.y * cosyaw;

  float globalvx = setpoint->velocity.x;
  float globalvy = setpoint->velocity.y;

  //X, Y
  if (setpoint->mode.x == modeAbs) {
    setpoint->velocity.x = runPid(state_body_x, &this.pidX, setp_body_x, DT);
  } else if (!setpoint->velocity_body) {
    setpoint->velocity.x = globalvx * cosyaw + globalvy * sinyaw;
  }
  if (setpoint->mode.y == modeAbs) {
    setpoint->velocity.y = runPid(state_body_y, &this.pidY, setp_body_y, DT);
  } else if (!setpoint->velocity_body) {
    setpoint->velocity.y = globalvy * cosyaw - globalvx * sinyaw;
  }
  if (setpoint->mode.z == modeAbs && this.pidZ.last_hold == 0.0f) {
    float error = this.pidZ.last_hold - (setpoint->position.z - state->position.z);
    if (fabsf(error) > this.pidZ.co){
      setpoint->velocity.z = runPid(state->position.z, &this.pidZ, setpoint->position.z, this.pidZ.count * DT);
      this.pidZ.count = 1.0;
      this.pidZ.last_hold = error;
    }else{
      this.pidZ.count += 1.0;
    }
    // setpoint->velocity.z = runPid(state->position.z, &this.pidZ, setpoint->position.z, DT);
  }

  velocityEBController(thrust, attitude, setpoint, state);
}

void velocityEBController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  this.pidVX.pid.outputLimit = pLimit * rpLimitOverhead;
  this.pidVY.pid.outputLimit = rLimit * rpLimitOverhead;
  // Set the output limit to the maximum thrust range
  this.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
  //this.pidVZ.pid.outputLimit = (this.thrustBase - this.thrustMin) / thrustScale;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  state_body_vx = state->velocity.x * cosyaw + state->velocity.y * sinyaw;
  state_body_vy = -state->velocity.x * sinyaw + state->velocity.y * cosyaw;

  // Roll and Pitch
  attitude->pitch = -runPid(state_body_vx, &this.pidVX, setpoint->velocity.x, DT) - kFFx*setpoint->velocity.x;
  attitude->roll = -runPid(state_body_vy, &this.pidVY, setpoint->velocity.y, DT) - kFFy*setpoint->velocity.y;

  attitude->roll  = constrain(attitude->roll,  -rLimit, rLimit);
  attitude->pitch = constrain(attitude->pitch, -pLimit, pLimit);

  // Thrust
  float thrustRaw = runPid(state->velocity.z, &this.pidVZ, setpoint->velocity.z, DT);
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw*thrustScale + this.thrustBase;
  // Check for minimum thrust
  if (*thrust < this.thrustMin) {
    *thrust = this.thrustMin;
  }
    // saturate
  *thrust = constrain(*thrust, 0, UINT16_MAX);
}

void positionEBControllerResetAllPID()
{
  pidReset(&this.pidX.pid);
  pidReset(&this.pidY.pid);
  pidReset(&this.pidZ.pid);
  pidReset(&this.pidVX.pid);
  pidReset(&this.pidVY.pid);
  pidReset(&this.pidVZ.pid);
}

void positionEBControllerResetAllfilters() {
  filterReset(&this.pidX.pid, POSITION_RATE, posEBFiltCutoff, posEBFiltEnable);
  filterReset(&this.pidY.pid, POSITION_RATE, posEBFiltCutoff, posEBFiltEnable);
  filterReset(&this.pidZ.pid, POSITION_RATE, posEBZFiltCutoff, posEBZFiltEnable);
  filterReset(&this.pidVX.pid, POSITION_RATE, velEBFiltCutoff, velEBFiltEnable);
  filterReset(&this.pidVY.pid, POSITION_RATE, velEBFiltCutoff, velEBFiltEnable);
  filterReset(&this.pidVZ.pid, POSITION_RATE, velEBZFiltCutoff, velEBZFiltEnable);
}

/**
 * Log variables of the PID position controller
 *
 * Note: rename to posCtrlPID ?
 */
LOG_GROUP_START(posEbCtl)

/**
 * @brief PID controller target desired body-yaw-aligned velocity x [m/s]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVX, &this.pidVX.pid.desired)
/**
 * @brief PID controller target desired body-yaw-aligned velocity y [m/s]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVY, &this.pidVY.pid.desired)
/**
 * @brief PID controller target desired velocity z [m/s]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVZ, &this.pidVZ.pid.desired)
/**
 * @brief PID controller target desired body-yaw-aligned position x [m]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetX, &this.pidX.pid.desired)
/**
 * @brief PID controller target desired body-yaw-aligned position y [m]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetY, &this.pidY.pid.desired)
/**
 * @brief PID controller target desired global position z [m]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetZ, &this.pidZ.pid.desired)

/**
 * @brief PID state body-yaw-aligned velocity x [m/s]
 *
 */
LOG_ADD(LOG_FLOAT, bodyVX, &state_body_vx)
/**
 * @brief PID state body-yaw-aligned velocity y [m/s]
 *
 */
LOG_ADD(LOG_FLOAT, bodyVY, &state_body_vy)
/**
 * @brief PID state body-yaw-aligned position x [m]
 *
 */
LOG_ADD(LOG_FLOAT, bodyX, &state_body_x)
/**
 * @brief PID state body-yaw-aligned position y [m]
 *
 */
LOG_ADD(LOG_FLOAT, bodyY, &state_body_y)

/**
 * @brief PID proportional output position y
 */
LOG_ADD(LOG_FLOAT, Yp, &this.pidY.pid.outP)
/**
 * @brief PID integral output position y
 */
LOG_ADD(LOG_FLOAT, Yi, &this.pidY.pid.outI)
/**
 * @brief PID derivative output position y
 */
LOG_ADD(LOG_FLOAT, Yd, &this.pidY.pid.outD)

/**
 * @brief PID proportional output position z
 */
LOG_ADD(LOG_FLOAT, Zp, &this.pidZ.pid.outP)
/**
 * @brief PID integral output position z
 */
LOG_ADD(LOG_FLOAT, Zi, &this.pidZ.pid.outI)
/**
 * @brief PID derivative output position z
 */
LOG_ADD(LOG_FLOAT, Zd, &this.pidZ.pid.outD)
/**
 * @brief Z count
 */
LOG_ADD(LOG_FLOAT, Zcount, &this.pidZ.count)

/**
 * @brief PID proportional output velocity x
 */
LOG_ADD(LOG_FLOAT, VXp, &this.pidVX.pid.outP)
/**
 * @brief PID integral output velocity x
 */
LOG_ADD(LOG_FLOAT, VXi, &this.pidVX.pid.outI)
/**
 * @brief PID derivative output velocity x
 */
LOG_ADD(LOG_FLOAT, VXd, &this.pidVX.pid.outD)

/**
 * @brief PID proportional output velocity z
 */
LOG_ADD(LOG_FLOAT, VZp, &this.pidVZ.pid.outP)
/**
 * @brief PID integral output velocity z
 */
LOG_ADD(LOG_FLOAT, VZi, &this.pidVZ.pid.outI)
/**
 * @brief PID integral output velocity z
 */
LOG_ADD(LOG_FLOAT, VZd, &this.pidVZ.pid.outD)

LOG_GROUP_STOP(posEbCtl)

/**
 * Tuning settings for the gains of the PID
 * controller for the velocity of the Crazyflie ¨
 * in the body-yaw-aligned X & Y and global Z directions.
 */
PARAM_GROUP_START(velEbCtlPid)
/**
 * @brief Proportional gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT, vxKp, &this.pidVX.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT, vxKi, &this.pidVX.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT, vxKd, &this.pidVX.pid.kd)
/**
 * @brief VX.Co
 */
PARAM_ADD(PARAM_FLOAT, vxCo, &this.pidVX.co)
/**
 * @brief VX.Ai
 */
PARAM_ADD(PARAM_FLOAT, vxAi, &this.pidVX.ai)

/**
 * @brief Proportional gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT, vyKp, &this.pidVY.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT, vyKi, &this.pidVY.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT, vyKd, &this.pidVY.pid.kd)
/**
 * @brief VY.Co
 */
PARAM_ADD(PARAM_FLOAT, vyCo, &this.pidVY.co)
/**
 * @brief VY.Ai
 */
PARAM_ADD(PARAM_FLOAT, vyAi, &this.pidVY.ai)

/**
 * @brief Proportional gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT, vzKp, &this.pidVZ.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT, vzKi, &this.pidVZ.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT, vzKd, &this.pidVZ.pid.kd)
/**
 * @brief VZ.Co
 */
PARAM_ADD(PARAM_FLOAT, vzCo, &this.pidVZ.co)
/**
 * @brief VZ.Ai
 */
PARAM_ADD(PARAM_FLOAT, vzAi, &this.pidVZ.ai)
/**
 * @brief Feed-forward gain for the velocity PID in the body-yaw-aligned X direction (in degrees per m/s)
 */
PARAM_ADD(PARAM_FLOAT, vxKFF, &kFFx)
/**
 * @brief Feed-forward gain for the velocity PID in the body-yaw-aligned Y direction (in degrees per m/s)
 */
PARAM_ADD(PARAM_FLOAT, vyKFF, &kFFy)

PARAM_GROUP_STOP(velEbCtlPid)

/**
 * Tuning settings for the gains of the PID
 * controller for the position of the Crazyflie ¨
 * in the body-yaw-aligned X & Y and global Z directions.
 */
PARAM_GROUP_START(posEbCtlPid)
/**
 * @brief Proportional gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT, xKp, &this.pidX.pid.kp)
/**
 * @brief Proportional gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT, xKi, &this.pidX.pid.ki)
/**
 * @brief Derivative gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT, xKd, &this.pidX.pid.kd)
/**
 * @brief X.Co
 */
PARAM_ADD(PARAM_FLOAT, xCo, &this.pidX.co)
/**
 * @brief X.Ai
 */
PARAM_ADD(PARAM_FLOAT, xAi, &this.pidX.ai)

/**
 * @brief Proportional gain for the position PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT, yKp, &this.pidY.pid.kp)
/**
 * @brief Integral gain for the position PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT, yKi, &this.pidY.pid.ki)
/**
 * @brief Derivative gain for the position PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT, yKd, &this.pidY.pid.kd)
/**
 * @brief Y.Co
 */
PARAM_ADD(PARAM_FLOAT, yCo, &this.pidY.co)
/**
 * @brief Y.Ai
 */
PARAM_ADD(PARAM_FLOAT, yAi, &this.pidY.ai)

/**
 * @brief Proportional gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT, zKp, &this.pidZ.pid.kp)
/**
 * @brief Integral gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT, zKi, &this.pidZ.pid.ki)
/**
 * @brief Derivative gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT, zKd, &this.pidZ.pid.kd)
/**
 * @brief Z.Co
 */
PARAM_ADD(PARAM_FLOAT, zCo, &this.pidZ.co)
/**
 * @brief Z.Ai
 */
PARAM_ADD(PARAM_FLOAT, zAi, &this.pidZ.ai)

/**
 * @brief Approx. thrust needed for hover
 */
PARAM_ADD(PARAM_UINT16, thrustBase, &this.thrustBase)
/**
 * @brief Min. thrust value to output
 */
PARAM_ADD(PARAM_UINT16, thrustMin, &this.thrustMin)

/**
 * @brief Roll absolute limit
 */
PARAM_ADD(PARAM_FLOAT, rLimit,  &rLimit)
/**
 * @brief Pitch absolute limit
 */
PARAM_ADD(PARAM_FLOAT, pLimit,  &pLimit)
/**
 * @brief Maximum body-yaw-aligned X velocity
 */
PARAM_ADD(PARAM_FLOAT, xVelMax, &xVelMax)
/**
 * @brief Maximum body-yaw-aligned Y velocity
 */
PARAM_ADD(PARAM_FLOAT, yVelMax, &yVelMax)
/**
 * @brief Maximum Z Velocity
 */
PARAM_ADD(PARAM_FLOAT, zVelMax,  &zVelMax)

PARAM_GROUP_STOP(posEbCtlPid)
