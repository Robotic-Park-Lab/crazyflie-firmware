/**
 *
 * Crazyflie Gimbal control firmware
 *
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "app.h"
#include "attitude_controller.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "platform_defaults.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "controller.h"
#include "controller_pid.h"

#define DEBUG_MODULE "GIMBAL"
#include "debug.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

bool gimbal = false;
bool rate = false;
bool relay = false;
bool rate_relay = false;
bool roll_relay = true;
bool attitude_relay = false;
bool relay_level = false;
bool tuned = false;
static int idx = 0;
static float relay_error = 0.0f;
static float relay_threshold = 0.2f;
static float relay_cmd = 10.0f;
static float rate_relay_threshold = 2.0f;
static float rate_relay_cmd = 5000.0f;
static float rate_cmd_eq[200];
static float attitude_cmd_eq[100];
static float pitch_rate_filt = 0.0f; 
static float pitch_rate[10];
static float cmd_eq = 0.0f;
static float rroll_cmd = 0.0f;
static float pitch_cmd = 0.0f;
static float roll_cmd = 0.0f;
static float rpitch_cmd = 0.0f;
static float ryaw_cmd = 0.0f;
static float rroll_count = 0.0f;
static float rpitch_count = 0.0f;
static float rroll_k = 0.0447f;
static float rroll_d = 0.0168f;
static float att_roll_count = 0.0f;
static float att_pitch_count = 0.0f;
static float roll_k = 0.0f;
static float roll_d = 0.0;
static float rele_Tu = 0.0f;
static float rele_Tu_max = 0.0f;
static float rele_a = 0.0f;
static float rele_a_max = 0.0f;
static float rele_a_min = 0.0f;
static float aux_pid = 0.0f;
static float aux_pid_rate = 0.0f;

static setpoint_t setpoint_cmd;
static state_t state_process;

void target_pose(float pose[]){}
void enable_formation(){}
void add_new_agent(float name, float pose[], float d, float k) {}
void remove_neighbour(float name){}
void update_distance(float name, float d){}
void update_agent_pose(float name, float x, float y, float z) {}
void controller_update(setpoint_t setpoint, const state_t *state) {}


static bool rateFiltEnable = 1; //ATTITUDE_RATE_LPF_ENABLE;
static float omxFiltCutoff = ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ;
static float omyFiltCutoff = ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ;
static float omzFiltCutoff = ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ;
static float attFiltCutoff = ATTITUDE_LPF_CUTOFF_FREQ;
static bool attFiltEnable = 1;// ATTITUDE_LPF_ENABLE;


static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

PidObject pidRollRate_rp = {
  .kp = 232.648f, // 213.9816f, // PID_ROLL_RATE_KP,
  .ki = 594.118f, // 558.4472f, // PID_ROLL_RATE_KI,
  .kd =   0.0f,   // 5.1245f, // PID_ROLL_RATE_KD,
  .kff = PID_ROLL_RATE_KFF,
  .iLimit = PID_ROLL_RATE_INTEGRATION_LIMIT,
  .outputLimit = 720.0f,
};

threshold roll_rate_threshold = {
  .ai = 0.1f,
  .cn = 0.0f,
  .co = 0.5f,
  .count = 1,
  .last_signal = 0.0f,
  .dt = 0.0f,
};

PidObject pidPitchRate_rp = {
  .kp = PID_PITCH_RATE_KP,
  .ki = PID_PITCH_RATE_KI,
  .kd = PID_PITCH_RATE_KD,
  .kff = PID_PITCH_RATE_KFF,
  .iLimit = PID_PITCH_RATE_INTEGRATION_LIMIT,
  .outputLimit = 720.0f,
};

threshold pitch_rate_threshold = {
  .ai = 0.1f,
  .cn = 0.0f,
  .co = 0.0f,
  .count = 1,
  .last_signal = 0.0f,
  .dt = 0.0f,
};

PidObject pidRoll_rp = {
  .kp = 2.0f, //PID_ROLL_KP,
  .ki = 0.0f, //PID_ROLL_KI,
  .kd = PID_ROLL_KD,
  .kff = PID_ROLL_KFF,
  .iLimit = PID_ROLL_INTEGRATION_LIMIT,
  .outputLimit = 50.0f,
};

threshold roll_threshold = {
  .ai = 0.1f,
  .cn = 0.0f,
  .co = 0.1f,
  .count = 1,
  .last_signal = 0.0f,
  .dt = 0.0f,
};

PidObject pidPitch_rp = {
  .kp = 2.0f, //PID_PITCH_KP,
  .ki = 0.0f, //PID_PITCH_KI,
  .kd = 0.0f, //PID_PITCH_KD,
  .kff = PID_PITCH_KFF,
  .iLimit = PID_PITCH_INTEGRATION_LIMIT,
  .outputLimit = 50.0f,
};

threshold pitch_threshold = {
  .ai = 0.1f,
  .cn = 0.0f,
  .co = 0.0f,
  .count = 1,
  .last_signal = 0.0f,
  .dt = 0.0f,
};

PidObject pidYawRate_rp = {
  .kp = PID_YAW_RATE_KP,
  .ki = PID_YAW_RATE_KI,
  .kd = PID_YAW_RATE_KD,
  .kff = PID_YAW_RATE_KFF,
};

PidObject pidYaw_rp = {
  .kp = PID_YAW_KP,
  .ki = PID_YAW_KI,
  .kd = PID_YAW_KD,
  .kff = PID_YAW_KFF,
};

threshold init_triggering(float co, float ai)
{
  threshold  trigger;

  trigger.co = co;
  trigger.ai = ai;
  trigger.count = 1;
  trigger.last_signal = 0.0f;
  trigger.dt = 0.0f;

  return trigger;
}

bool eval_threshold(threshold *trigger, float signal, float ref)
{
  if(trigger->co == 0.0f){
    trigger->count += 1;
    return true;
  }
  // Noise - Cn
  float mean = signal/20.0f;
  for(int i = 0; i<19; i++){
    trigger->noise[i+1] = trigger->noise[i];
    mean += trigger->noise[i]/20.0f;
  }
  trigger->noise[0] = signal;
  trigger->cn = 0.0;
  for(int i = 0; i<20; i++){
    if(fabsf(trigger->noise[i]-mean) > trigger->cn)
      trigger->cn = trigger->noise[i]-mean;
  }
    
  //trigger->cn = 0.0f;
  // a
  float a = trigger->ai * fabsf(signal - ref);
  if (a>trigger->ai)
    a = trigger->ai;
  // Threshold
  float th = trigger->co + a + trigger->cn;
  float inc = fabsf(fabsf(ref-signal) - trigger->last_signal);

  // Delta Error
  trigger->count += 1;
  if (inc >= th){
    trigger->last_signal = fabsf(ref-signal);
    return true;
  }

  if(trigger->count>500){
    trigger->last_signal = fabsf(ref-signal);
    return true;
  }
  return false;
}

void relay_params(uint8_t level, uint8_t angle, float cmd, float threshold)
{
  if(level)
  {
    attitude_relay = true;
    relay_threshold = threshold;
    relay_cmd = cmd;
  }else
  {
    rate_relay = true;
    rate_relay_threshold = threshold;
    rate_relay_cmd = cmd;
  }
  if(angle)
  {
    roll_relay = false;
  }else
  {
    roll_relay = true;
  }
}

void controller_params(uint8_t level, uint8_t angle, float kp, float ki, float kd, float co)
{
  if(level)
  {
    if(angle)
    {
      pidPitch_rp.kp = kp;
      pidPitch_rp.ki = ki;
      pidPitch_rp.kd = kd;
      pitch_threshold.co = co;
    }else
    {
      pidRoll_rp.kp = kp;
      pidRoll_rp.ki = ki;
      pidRoll_rp.kd = kd;
      roll_threshold.co = co;
    }
  }else
  {
    if(angle)
    {
      pidPitchRate_rp.kp = kp;
      pidPitchRate_rp.ki = ki;
      pidPitchRate_rp.kd = kd;
      pitch_rate_threshold.co = co;
    }else
    {
      pidRollRate_rp.kp = kp;
      pidRollRate_rp.ki = ki;
      pidRollRate_rp.kd = kd;
      roll_rate_threshold.co = co;
    }
  }
}

void attitud_cmd(float roll, float pitch, float yaw, float thrust){
  if(rate){
    rate = false;
    setpoint_cmd.mode.roll = modeAbs;
    setpoint_cmd.mode.pitch = modeAbs;
    setpoint_cmd.mode.yaw = modeAbs;
  }
  setpoint_cmd.attitude.roll  = roll;
  setpoint_cmd.attitude.pitch = pitch;
  setpoint_cmd.attitude.yaw = yaw;
  setpoint_cmd.thrust = thrust;
  commanderSetSetpoint(&setpoint_cmd, 3);
}

void attituderate_cmd(float roll, float pitch, float yaw, float thrust){
  rate = true;
  if(!rate){
    rate = true;
    setpoint_cmd.mode.roll = modeVelocity;
    setpoint_cmd.mode.pitch = modeVelocity;
    setpoint_cmd.mode.yaw = modeVelocity;
  }
  setpoint_cmd.attitudeRate.roll  = roll;
  setpoint_cmd.attitudeRate.pitch = pitch;
  setpoint_cmd.attitudeRate.yaw = yaw;
  setpoint_cmd.thrust = thrust;
  commanderSetSetpoint(&setpoint_cmd, 3);
}

float update_attitude_relay(float setpoint, float value){
  relay_error = setpoint - value;
  if(!relay_level){
    if(relay_error>relay_threshold){
      if(roll_relay){
        rele_Tu = roll_threshold.count*0.002f;
        roll_threshold.count = 0.0f;
      }else{
        rele_Tu = pitch_threshold.count*0.002f;
        pitch_threshold.count = 0.0f;
      }
      float aux = fabsf(rele_a_max) + fabsf(rele_a_min);
      if(rele_a == 0 || rele_a_max == 0.0f || rele_a_min == 0.0f){
        rele_a = aux*2.0f;
      }else{
        rele_a = aux ; //(aux+rele_a)/2.0f;
      }
      rele_Tu_max = 0.0f;
      rele_a_max = 0.0f;
      rele_a_min = 0.0f;
      relay_level = true;
      return relay_cmd+cmd_eq;
    }else{
      return -relay_cmd+cmd_eq;
    }
  }else{
    if(relay_error<-relay_threshold){
      relay_level = false;
      roll_threshold.count = 0.0f;
      return -relay_cmd+cmd_eq;
    }else{
      return relay_cmd+cmd_eq;
    }
  }
}

float update_rate_relay(float setpoint, float value){
  relay_error = setpoint - value;
  if(!relay_level){
    if(relay_error>rate_relay_threshold){
      if(roll_relay){
        rele_Tu = roll_rate_threshold.count*0.002f;
        roll_rate_threshold.count = 0.0f;
      }else{
        rele_Tu = pitch_rate_threshold.count*0.002f;
        pitch_rate_threshold.count = 0.0f;
      }
      float aux = fabsf(rele_a_max) + fabsf(rele_a_min);
      if(rele_a == 0 || rele_a_max == 0.0f || rele_a_min == 0.0f){
        rele_a = aux*2.0f;
      }else{
        rele_a = aux ; //(aux+rele_a)/2.0f;
      }
      rele_Tu_max = 0.0f;
      rele_a_max = 0.0f;
      rele_a_min = 0.0f;
      relay_level = true;
      return rate_relay_cmd+cmd_eq;
    }else{
      return -rate_relay_cmd+cmd_eq;
    }
  }else{
    if(relay_error<-rate_relay_threshold){
      relay_level = false;
      return -rate_relay_cmd+cmd_eq;
    }else{
      return rate_relay_cmd+cmd_eq;
    }
  }
}

void enable_relay(){
  relay_error = 0.0f;
  if(roll_relay){
    aux_pid = pidRoll_rp.integ;
    aux_pid_rate = pidRollRate_rp.integ;
    roll_threshold.count = 0.0f;
    roll_rate_threshold.count = 0.0f;
  }else{
    aux_pid = pidPitch_rp.integ;
    aux_pid_rate = pidPitchRate_rp.integ;
    pitch_threshold.count = 0.0f;
    pitch_rate_threshold.count = 0.0f;
  }
  if(!relay){
    relay = true;
    if(attitude_relay){
      for (size_t i = 0; i < 99; i++){
        cmd_eq = cmd_eq + attitude_cmd_eq[i];
      }
    }else{
      for (size_t i = 0; i < 199; i++){
        cmd_eq = cmd_eq + rate_cmd_eq[i];
      }
    }
  }else{
    relay = false;
    float Ku = (4.0f*relay_cmd*2.0f)/(3.1415f*sqrtf(powf(rele_a,2.0f)-powf(relay_threshold,2.0f)));
    rroll_d = rele_Tu_max/4.0f;
    rroll_k = 2.0f*3.1415f/(Ku*roll_d*4);
    tuned = true;
    cmd_eq = 0.0f;
    rele_Tu_max = 0.0f;
    rele_Tu = 0.0f;
    rele_a_max = 0.0f;
    rele_a_min = 0.0f;
    rele_a = 0.0f;

    if(rate_relay){
      if(roll_relay){
        // pidReset(&pidRollRate_rp);
        pidRollRate_rp.integ = aux_pid_rate;
      }else{
        // pidReset(&pidPitchRate_rp);
        pidPitchRate_rp.integ = aux_pid_rate;
      }
    }
    if(attitude_relay){
      //attitudeControllerResetAllPID();
      if(roll_relay){
        // pidReset(&pidRoll_rp);
        pidRoll_rp.integ = aux_pid;
      }else{
        // pidReset(&pidPitch_rp);
        pidPitch_rp.integ = aux_pid;
      }
    }
  }
}

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  
  DEBUG_PRINT("In progress ...\n");
  while(1) {
    vTaskDelay(M2T(2000));
  }
}

void controllerOutOfTreeInit() {
  // Call the PID controller instead in this example to make it possible to fly
  setpoint_cmd.mode.x = modeDisable;
  setpoint_cmd.mode.y = modeDisable;
  setpoint_cmd.mode.z = modeDisable;
  setpoint_cmd.mode.roll = modeAbs;
  setpoint_cmd.mode.pitch = modeDisable;
  setpoint_cmd.mode.yaw = modeDisable;
  controllerPidInit();

  pidInit(&pidRollRate_rp,  0, pidRollRate_rp.kp,  pidRollRate_rp.ki,  pidRollRate_rp.kd,
       pidRollRate_rp.kff,  0.002f, ATTITUDE_RATE, omxFiltCutoff, rateFiltEnable);
  pidInit(&pidPitchRate_rp, 0, pidPitchRate_rp.kp, pidPitchRate_rp.ki, pidPitchRate_rp.kd,
       pidPitchRate_rp.kff, 0.002f, ATTITUDE_RATE, omyFiltCutoff, rateFiltEnable);
  pidInit(&pidYawRate_rp,   0, pidYawRate_rp.kp,   pidYawRate_rp.ki,   pidYawRate_rp.kd,
       pidYawRate_rp.kff,   0.002f, ATTITUDE_RATE, omzFiltCutoff, rateFiltEnable);
  
  pidSetIntegralLimit(&pidRollRate_rp,  PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate_rp, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate_rp,   PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&pidRoll_rp,  0, pidRoll_rp.kp,  pidRoll_rp.ki,  pidRoll_rp.kd,
       pidRoll_rp.kff,  0.002f, ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&pidPitch_rp, 0, pidPitch_rp.kp, pidPitch_rp.ki, pidPitch_rp.kd,
       pidPitchRate_rp.kff, 0.002f, ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&pidYaw_rp,   0, pidYaw_rp.kp,   pidYaw_rp.ki,   pidYaw_rp.kd,
       pidYaw_rp.kff,   0.002f, ATTITUDE_RATE, attFiltCutoff, attFiltEnable);

  pidSetIntegralLimit(&pidRoll_rp,  PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch_rp, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw_rp,   PID_YAW_INTEGRATION_LIMIT);

}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t* setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t stabilizerStep) {
  // Implement your controller here...
  // controller_update(setpoint, state);

  actuatorThrust = setpoint_cmd.thrust+2000;
  // controllerPid(control, setpoint, sensors, state, tick);
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    attitudeDesired.roll = setpoint_cmd.attitude.roll;
    attitudeDesired.pitch = setpoint_cmd.attitude.pitch;
    //attitudeDesired.yaw = setpoint_cmd.attitude.yaw;

    pitch_rate_filt -= pitch_rate[9];
    for (size_t i = 9; i > 0; i--){
      pitch_rate[i] = pitch_rate[i-1];
    }
    pitch_rate[0] = -sensors->gyro.y/10.0f;
    pitch_rate_filt += pitch_rate[0];

    // ATTITUDE LEVEL
    if(attitude_relay && relay){
      if(roll_relay){
        if(eval_threshold(&pitch_threshold, state->attitude.pitch, attitudeDesired.pitch)){
          pidSetDesired(&pidPitch_rp, attitudeDesired.pitch);
          pidPitch_rp.dt = 0.002f * pitch_threshold.count;
          rateDesired.pitch = pidUpdate(&pidPitch_rp, state->attitude.pitch, true);
          pitch_threshold.count = 0;
        }else{
          rateDesired.pitch = pitch_cmd;
        }
        // Roll
        rateDesired.roll = update_attitude_relay(attitudeDesired.roll, state->attitude.roll);
        roll_threshold.count += 1.0;
        float aux = attitudeDesired.roll-state->attitude.roll;
        if(aux > rele_a_max)
        {
          rele_Tu_max = roll_threshold.count*0.002f;
          rele_a_max = aux;
        }
        if(aux < rele_a_min)
        {
          rele_Tu_max = roll_threshold.count*0.002f;
          rele_a_min = aux;
        }
      }else{
        if(eval_threshold(&roll_threshold, state->attitude.roll, attitudeDesired.roll)){
          pidSetDesired(&pidRoll_rp, attitudeDesired.roll);
          pidRoll_rp.dt = 0.002f * roll_threshold.count;
          rateDesired.roll = pidUpdate(&pidRoll_rp, state->attitude.roll, true);
          roll_threshold.count = 0;
        }else{
          rateDesired.roll = roll_cmd;
        }

        rateDesired.pitch = update_attitude_relay(attitudeDesired.pitch, state->attitude.pitch);
        pitch_threshold.count += 1.0;
        float aux = attitudeDesired.pitch-state->attitude.pitch;
        if(aux > rele_a_max)
        {
          rele_Tu_max = pitch_threshold.count*0.002f;
          rele_a_max = aux;
        }
        if(aux < rele_a_min)
        {
          rele_Tu_max = pitch_threshold.count*0.002f;
          rele_a_min = aux;
        }

      }

      // Yaw
      rateDesired.yaw = 0.0f;
      //rateDesired.yaw = update_attitude_relay(attitudeDesired.roll, state->attitude.roll);
      //attitudeControllerResetYawAttitudePID();
    }else{
      /*
      if(tuned){
        pidRoll_rp.kp = 4.0486f; // 3.7142f; // 6.5948f;
        pidRoll_rp.ki = 3.3264f; // 3.4661f; // 12.1417f;
        pidRoll_rp.kd = 0.0000f; // 0.2239f;
        roll_threshold.co = 0.0f;
        tuned = false;
      }
      */
      if(eval_threshold(&roll_threshold, state->attitude.roll, attitudeDesired.roll))// && roll_threshold.count>49)
      {
        pidSetDesired(&pidRoll_rp, attitudeDesired.roll);
        pidRoll_rp.dt = 0.002f * roll_threshold.count;
        rateDesired.roll = pidUpdate(&pidRoll_rp, state->attitude.roll, true);
        roll_threshold.count = 0;
      }
      else
      {
        rateDesired.roll = roll_cmd;
      }

      if(eval_threshold(&pitch_threshold, state->attitude.pitch, attitudeDesired.pitch))
      {
        pidSetDesired(&pidPitch_rp, attitudeDesired.pitch);
        pidPitch_rp.dt = 0.002f * pitch_threshold.count;
        rateDesired.pitch = pidUpdate(&pidPitch_rp, state->attitude.pitch, true);
        pitch_threshold.count = 0;
      }
      else
      {
        rateDesired.pitch = pitch_cmd;
      }
    }

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity || rate) {
      rateDesired.roll = setpoint_cmd.attitudeRate.roll; //setpoint->attitudeRate.roll;
      pidReset(&pidRoll_rp);
    }
    if (setpoint->mode.pitch == modeVelocity || rate) {
      rateDesired.pitch = setpoint_cmd.attitudeRate.pitch; // setpoint->attitudeRate.pitch;
      pidReset(&pidPitch_rp);
    }

    rateDesired.yaw = 0.0f;
    
    // RATE LEVEL
    if(rate_relay && relay){
      if(roll_relay){
        if(eval_threshold(&pitch_rate_threshold, pitch_rate_filt, rateDesired.pitch)){
          pidSetDesired(&pidPitchRate_rp, rateDesired.pitch);
          pidPitchRate_rp.dt = 0.002f * pitch_rate_threshold.count;
          control->pitch = saturateSignedInt16(pidUpdate(&pidPitchRate_rp, pitch_rate_filt, true));
          pitch_rate_threshold.count = 0;
        }else{
          control->pitch = rpitch_cmd;
        }

        control->roll = update_rate_relay(0.0f, sensors->gyro.x);
        roll_rate_threshold.count += 1.0;
        float aux = sensors->gyro.x;
        if(aux > rele_a_max){
          rele_a_max = aux;
          rele_Tu_max = roll_rate_threshold.count*0.002f;
        }
        if(aux < rele_a_min){
          rele_a_min = aux;
        }
      }else{
        if(eval_threshold(&roll_rate_threshold, sensors->gyro.x, rateDesired.roll)){
          pidSetDesired(&pidRollRate_rp, rateDesired.roll);
          pidRollRate_rp.dt = 0.002f * roll_rate_threshold.count;
          control->roll = saturateSignedInt16(pidUpdate(&pidRollRate_rp, sensors->gyro.x, true));
          roll_rate_threshold.count = 0;
        }else{
          control->roll = rroll_cmd;
        }

        control->pitch = update_rate_relay(0.0f, pitch_rate_filt);
        pitch_rate_threshold.count += 1.0;
        float aux = pitch_rate_filt;
        if(aux > rele_a_max){
          rele_a_max = aux;
        }
        if(aux < rele_a_min){
          rele_a_min = aux;
          rele_Tu_max = pitch_rate_threshold.count*0.002f;
        }
      }
      
    }else{
      if(eval_threshold(&roll_rate_threshold, sensors->gyro.x, rateDesired.roll)){
        pidSetDesired(&pidRollRate_rp, rateDesired.roll);
        pidRollRate_rp.dt = 0.002f * roll_rate_threshold.count;
        control->roll = saturateSignedInt16(pidUpdate(&pidRollRate_rp, sensors->gyro.x, true));
        roll_rate_threshold.count = 0;
      }else{
        control->roll = rroll_cmd;
      }
      if(eval_threshold(&pitch_rate_threshold, -sensors->gyro.y, rateDesired.pitch)){
        pidSetDesired(&pidPitchRate_rp, rateDesired.pitch);
        pidPitchRate_rp.dt = 0.002f * pitch_rate_threshold.count;
        control->pitch = saturateSignedInt16(pidUpdate(&pidPitchRate_rp, -sensors->gyro.y, true));
        pitch_rate_threshold.count = 0;
      }else{
        control->pitch = rpitch_cmd;
      }
    }
    control->yaw = saturateSignedInt16(0.0f);
    // control->yaw = -saturateSignedInt16(20000.0f); //-control->yaw;
  }
  state_process.attitude.roll = state->attitude.roll;
  state_process.attitude.pitch = state->attitude.pitch;
  state_process.attitude.yaw = state->attitude.yaw;
  control->thrust = actuatorThrust;
  roll_cmd = rateDesired.roll;
  pitch_cmd = rateDesired.pitch;
  rroll_cmd = control->roll;
  rpitch_cmd = control->pitch;
  ryaw_cmd = control->yaw;
  rroll_count = roll_rate_threshold.count;
  rpitch_count = pitch_rate_threshold.count;
  att_roll_count = roll_threshold.count;
  att_pitch_count = pitch_threshold.count;

  if(rate_relay){
    if(roll_relay){
      rate_cmd_eq[idx] = rroll_cmd/200.0f;
    }else{
      rate_cmd_eq[idx] = rpitch_cmd/200.0f;
    }
    if(idx == 199){
      idx = 0;
    }else{
      idx += 1;
    }
  }
  if(attitude_relay)
  {
    if(roll_relay){
      attitude_cmd_eq[idx] = roll_cmd/100.0f;
    }else{
      attitude_cmd_eq[idx] = pitch_cmd/100.0f;
    }
    if(idx == 99){
      idx = 0;
    }else{
      idx += 1;
    }
  }
  

}

/**
 */
LOG_GROUP_START(rele)
/**
 * @brief Setpoint_x
 */
LOG_ADD(LOG_FLOAT, roll_sp, &setpoint_cmd.attitude.roll)
/**
 * @brief Setpoint_y
 */
LOG_ADD(LOG_FLOAT, pitch_sp, &setpoint_cmd.attitude.pitch)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, yaw_sp, &setpoint_cmd.attitude.yaw)
/**
 * @brief Setpoint_x
 */
LOG_ADD(LOG_FLOAT, roll, &state_process.attitude.roll)
/**
 * @brief Setpoint_y
 */
LOG_ADD(LOG_FLOAT, pitch, &state_process.attitude.pitch)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, yaw, &state_process.attitude.yaw)
/**
 * @brief Setpoint_x
 */
LOG_ADD(LOG_FLOAT, rate_roll_sp, &rateDesired.roll)
/**
 * @brief Setpoint_y
 */
LOG_ADD(LOG_FLOAT, rate_pitch_sp, &rateDesired.pitch)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, rate_yaw_sp, &relay_error)
/**
 * @brief Pitch value (y)
 */
LOG_ADD(LOG_FLOAT, rate_pitch, &pitch_rate_filt)
/**
 * @brief Setpoint_x
 */
LOG_ADD(LOG_FLOAT, rate_roll_cmd, &rroll_cmd)
/**
 * @brief Setpoint_y
 */
LOG_ADD(LOG_FLOAT, rate_pitch_cmd, &rpitch_cmd)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, rate_yaw_cmd, &ryaw_cmd)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, rate_roll_count, &rroll_count)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, rate_pitch_count, &rpitch_count)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, roll_count, &att_roll_count)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, pitch_count, &att_pitch_count)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, rate_roll_kp, &pidRollRate_rp.kp)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, rate_roll_ki, &pidRollRate_rp.ki)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, roll_kp, &pidRoll_rp.kp)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, roll_ki, &pidRoll_rp.ki)

LOG_ADD(LOG_FLOAT, a, &rele_a)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, a_max, &rele_a_max)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, tu, &rele_Tu)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, tu_max, &rele_Tu_max)

/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, roll_model_k, &roll_k)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, roll_model_d, &roll_d)

LOG_GROUP_STOP(rele)

