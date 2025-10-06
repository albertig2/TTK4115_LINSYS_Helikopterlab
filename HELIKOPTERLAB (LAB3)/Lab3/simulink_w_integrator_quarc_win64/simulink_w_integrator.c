/*
 * simulink_w_integrator.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "simulink_w_integrator".
 *
 * Model version              : 11.33
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C source code generated on : Mon Oct  6 19:46:16 2025
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "simulink_w_integrator.h"
#include "simulink_w_integrator_private.h"

t_stream simulink_w_integrator_rtZt_stream = NULL;

/* Block signals (default storage) */
B_simulink_w_integrator_T simulink_w_integrator_B;

/* Continuous states */
X_simulink_w_integrator_T simulink_w_integrator_X;

/* Block states (default storage) */
DW_simulink_w_integrator_T simulink_w_integrator_DW;

/* Real-time model */
static RT_MODEL_simulink_w_integrato_T simulink_w_integrator_M_;
RT_MODEL_simulink_w_integrato_T *const simulink_w_integrator_M =
  &simulink_w_integrator_M_;
static void rate_monotonic_scheduler(void);
time_T rt_SimUpdateDiscreteEvents(
  int_T rtmNumSampTimes, void *rtmTimingData, int_T *rtmSampleHitPtr, int_T
  *rtmPerTaskSampleHits )
{
  rtmSampleHitPtr[1] = rtmStepTask(simulink_w_integrator_M, 1);
  rtmSampleHitPtr[2] = rtmStepTask(simulink_w_integrator_M, 2);
  UNUSED_PARAMETER(rtmNumSampTimes);
  UNUSED_PARAMETER(rtmTimingData);
  UNUSED_PARAMETER(rtmPerTaskSampleHits);
  return(-1);
}

/*
 *   This function updates active task flag for each subrate
 * and rate transition flags for tasks that exchange data.
 * The function assumes rate-monotonic multitasking scheduler.
 * The function must be called at model base rate so that
 * the generated code self-manages all its subrates and rate
 * transition flags.
 */
static void rate_monotonic_scheduler(void)
{
  /* To ensure a deterministic data transfer between two rates,
   * data is transferred at the priority of a fast task and the frequency
   * of the slow task.  The following flags indicate when the data transfer
   * happens.  That is, a rate interaction flag is set true when both rates
   * will run, and false otherwise.
   */

  /* tid 1 shares data with slower tid rate: 2 */
  if (simulink_w_integrator_M->Timing.TaskCounters.TID[1] == 0) {
    simulink_w_integrator_M->Timing.RateInteraction.TID1_2 =
      (simulink_w_integrator_M->Timing.TaskCounters.TID[2] == 0);

    /* update PerTaskSampleHits matrix for non-inline sfcn */
    simulink_w_integrator_M->Timing.perTaskSampleHits[5] =
      simulink_w_integrator_M->Timing.RateInteraction.TID1_2;
  }

  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (simulink_w_integrator_M->Timing.TaskCounters.TID[2])++;
  if ((simulink_w_integrator_M->Timing.TaskCounters.TID[2]) > 4) {/* Sample time: [0.01s, 0.0s] */
    simulink_w_integrator_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/*
 * This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = (ODE1_IntgData *)rtsiGetSolverData(si);
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 11;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  simulink_w_integrator_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model output function for TID0 */
void simulink_w_integrator_output0(void) /* Sample time: [0.0s, 0.0s] */
{
  /* local block i/o variables */
  t_stream_ptr rtb_StreamCall1_o1;
  real_T rtb_HILReadEncoderTimebase_o1;
  real_T rtb_HILReadEncoderTimebase_o2;
  real_T rtb_HILReadEncoderTimebase_o3;
  real32_T rtb_StreamRead1_o2[10];
  int32_T rtb_StreamFormattedWrite_o2;
  int32_T rtb_StreamCall1_o3;
  uint8_T rtb_StreamCall1_o2;
  boolean_T rtb_StreamRead1_o3;
  real_T tmp_2[9];
  real_T rtb_Sum1[5];
  real_T tmp[5];
  real_T rtb_Gain2[3];
  real_T rtb_Sum[2];
  real_T tmp_0[2];
  real_T tmp_1[2];
  real_T ay;
  real_T az;
  real_T euler_rates_tmp;
  real_T euler_rates_tmp_0;
  real_T rtb_Backgain;
  real_T rtb_Frontgain;
  real_T rtb_Sum3;
  real_T rtb_Sum4;
  int32_T i;
  int32_T i_0;
  int32_T tmp_3;
  if (rtmIsMajorTimeStep(simulink_w_integrator_M)) {
    /* set solver stop time */
    if (!(simulink_w_integrator_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&simulink_w_integrator_M->solverInfo,
                            ((simulink_w_integrator_M->Timing.clockTickH0 + 1) *
        simulink_w_integrator_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&simulink_w_integrator_M->solverInfo,
                            ((simulink_w_integrator_M->Timing.clockTick0 + 1) *
        simulink_w_integrator_M->Timing.stepSize0 +
        simulink_w_integrator_M->Timing.clockTickH0 *
        simulink_w_integrator_M->Timing.stepSize0 * 4294967296.0));
    }

    {                                  /* Sample time: [0.0s, 0.0s] */
      rate_monotonic_scheduler();
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(simulink_w_integrator_M)) {
    simulink_w_integrator_M->Timing.t[0] = rtsiGetT
      (&simulink_w_integrator_M->solverInfo);
  }

  /* RateTransition: '<S6>/Rate Transition: y' */
  if (rtmIsMajorTimeStep(simulink_w_integrator_M)) {
    /* S-Function (hil_read_encoder_timebase_block): '<S4>/HIL Read Encoder Timebase' */

    /* S-Function Block: simulink_w_integrator/Heli 3D/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
    {
      t_error result;
      result = hil_task_read_encoder
        (simulink_w_integrator_DW.HILReadEncoderTimebase_Task, 1,
         &simulink_w_integrator_DW.HILReadEncoderTimebase_Buffer[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
      } else {
        rtb_HILReadEncoderTimebase_o1 =
          simulink_w_integrator_DW.HILReadEncoderTimebase_Buffer[0];
        rtb_HILReadEncoderTimebase_o2 =
          simulink_w_integrator_DW.HILReadEncoderTimebase_Buffer[1];
        rtb_HILReadEncoderTimebase_o3 =
          simulink_w_integrator_DW.HILReadEncoderTimebase_Buffer[2];
      }
    }

    /* RateTransition: '<S6>/Rate Transition: x' */
    if (simulink_w_integrator_M->Timing.RateInteraction.TID1_2) {
      /* RateTransition: '<S6>/Rate Transition: x' */
      simulink_w_integrator_B.RateTransitionx =
        simulink_w_integrator_DW.RateTransitionx_Buffer0;
    }

    /* End of RateTransition: '<S6>/Rate Transition: x' */

    /* DeadZone: '<S6>/Dead Zone: x' */
    if (simulink_w_integrator_B.RateTransitionx >
        simulink_w_integrator_P.DeadZonex_End) {
      ay = simulink_w_integrator_B.RateTransitionx -
        simulink_w_integrator_P.DeadZonex_End;
    } else if (simulink_w_integrator_B.RateTransitionx >=
               simulink_w_integrator_P.DeadZonex_Start) {
      ay = 0.0;
    } else {
      ay = simulink_w_integrator_B.RateTransitionx -
        simulink_w_integrator_P.DeadZonex_Start;
    }

    /* End of DeadZone: '<S6>/Dead Zone: x' */

    /* Gain: '<S6>/Joystick_gain_x' incorporates:
     *  Gain: '<S6>/Gain: x'
     */
    simulink_w_integrator_B.Joystick_gain_x = simulink_w_integrator_P.Gainx_Gain
      * ay * simulink_w_integrator_P.Joystick_gain_x;
    if (simulink_w_integrator_M->Timing.RateInteraction.TID1_2) {
      /* RateTransition: '<S6>/Rate Transition: y' */
      simulink_w_integrator_B.RateTransitiony =
        simulink_w_integrator_DW.RateTransitiony_Buffer0;
    }

    /* DeadZone: '<S6>/Dead Zone: y' */
    if (simulink_w_integrator_B.RateTransitiony >
        simulink_w_integrator_P.DeadZoney_End) {
      ay = simulink_w_integrator_B.RateTransitiony -
        simulink_w_integrator_P.DeadZoney_End;
    } else if (simulink_w_integrator_B.RateTransitiony >=
               simulink_w_integrator_P.DeadZoney_Start) {
      ay = 0.0;
    } else {
      ay = simulink_w_integrator_B.RateTransitiony -
        simulink_w_integrator_P.DeadZoney_Start;
    }

    /* End of DeadZone: '<S6>/Dead Zone: y' */

    /* Gain: '<S6>/Joystick_gain_y' incorporates:
     *  Gain: '<S6>/Gain: y'
     */
    simulink_w_integrator_B.Joystick_gain_y = simulink_w_integrator_P.Gainy_Gain
      * ay * simulink_w_integrator_P.Joystick_gain_y;
  }

  /* End of RateTransition: '<S6>/Rate Transition: y' */

  /* Sum: '<S7>/Sum3' incorporates:
   *  Integrator: '<S3>/Integrator'
   */
  rtb_Sum3 = simulink_w_integrator_B.Joystick_gain_x -
    simulink_w_integrator_X.Integrator_CSTATE[0];

  /* Sum: '<S7>/Sum4' incorporates:
   *  Integrator: '<S3>/Integrator'
   */
  rtb_Sum4 = simulink_w_integrator_B.Joystick_gain_y -
    simulink_w_integrator_X.Integrator_CSTATE[3];

  /* SignalConversion generated from: '<S7>/Gain3' incorporates:
   *  Integrator: '<S3>/Integrator'
   *  Integrator: '<S7>/Integrator'
   *  Integrator: '<S7>/Integrator1'
   */
  tmp[2] = simulink_w_integrator_X.Integrator_CSTATE[3];
  tmp[3] = simulink_w_integrator_X.Integrator_CSTATE_m;
  tmp[4] = simulink_w_integrator_X.Integrator1_CSTATE;
  tmp[0] = simulink_w_integrator_X.Integrator_CSTATE[0];

  /* Gain: '<S7>/Gain' incorporates:
   *  SignalConversion generated from: '<S7>/Gain'
   */
  tmp_0[0] = simulink_w_integrator_P.F[0] * rtb_Sum3 +
    simulink_w_integrator_P.F[2] * rtb_Sum4;

  /* SignalConversion generated from: '<S7>/Gain3' incorporates:
   *  Integrator: '<S3>/Integrator'
   */
  tmp[1] = simulink_w_integrator_X.Integrator_CSTATE[1];

  /* Gain: '<S7>/Gain' incorporates:
   *  SignalConversion generated from: '<S7>/Gain'
   */
  tmp_0[1] = simulink_w_integrator_P.F[1] * rtb_Sum3;
  tmp_0[1] += simulink_w_integrator_P.F[3] * rtb_Sum4;
  for (i = 0; i < 2; i++) {
    /* Gain: '<S7>/Gain3' */
    tmp_1[i] = 0.0;
    for (i_0 = 0; i_0 < 5; i_0++) {
      tmp_1[i] += simulink_w_integrator_P.K_LQR[(i_0 << 1) + i] * tmp[i_0];
    }

    /* End of Gain: '<S7>/Gain3' */

    /* Sum: '<S7>/Sum' */
    rtb_Sum[i] = tmp_0[i] - tmp_1[i];
  }

  /* Sum: '<Root>/Sum' incorporates:
   *  Constant: '<Root>/Constant'
   */
  rtb_Backgain = rtb_Sum[0] + simulink_w_integrator_P.Vs_0;

  /* Sum: '<S1>/Add' */
  rtb_Frontgain = rtb_Backgain - rtb_Sum[1];

  /* Sum: '<S1>/Subtract' */
  rtb_Backgain += rtb_Sum[1];

  /* Gain: '<S1>/Back gain' */
  rtb_Backgain *= simulink_w_integrator_P.Backgain_Gain;

  /* Gain: '<S1>/Front gain' */
  rtb_Frontgain *= simulink_w_integrator_P.Frontgain_Gain;

  /* Gain: '<S3>/C_est' incorporates:
   *  Integrator: '<S3>/Integrator'
   */
  for (i = 0; i < 5; i++) {
    rtb_Sum1[i] = 0.0;
    for (i_0 = 0; i_0 < 5; i_0++) {
      rtb_Sum1[i] += simulink_w_integrator_P.C_est[5 * i_0 + i] *
        simulink_w_integrator_X.Integrator_CSTATE[i_0];
    }
  }

  /* End of Gain: '<S3>/C_est' */
  if (rtmIsMajorTimeStep(simulink_w_integrator_M)) {
    /* S-Function (stream_call_block): '<S11>/Stream Call1' incorporates:
     *  Constant: '<S11>/Constant'
     *  S-Function (string_constant_block): '<S11>/String Constant'
     */

    /* S-Function Block: simulink_w_integrator/IMU system/IMU/Stream Call1 (stream_call_block) */
    {
      t_error result = 0;
      t_boolean close_flag = (simulink_w_integrator_P.Constant_Value_m != 0);
      rtb_StreamCall1_o1 = NULL;
      switch (simulink_w_integrator_DW.StreamCall1_State) {
       case STREAM_CALL_STATE_NOT_CONNECTED:
        {
          if (!close_flag) {
            /* Make sure URI is null-terminated */
            if (string_length((char *)
                              simulink_w_integrator_P.StringConstant_Value, 255)
                == 255) {
              rtmSetErrorStatus(simulink_w_integrator_M,
                                "URI passed to Stream Call block is not null-terminated!");
              result = -QERR_STRING_NOT_TERMINATED;
            } else {
              result = stream_connect((char *)
                simulink_w_integrator_P.StringConstant_Value,
                simulink_w_integrator_P.StreamCall1_NonBlocking != 0,
                simulink_w_integrator_P.StreamCall1_SendBufferSize,
                simulink_w_integrator_P.StreamCall1_ReceiveBufferSize,
                &simulink_w_integrator_DW.StreamCall1_Stream);
              if (result == 0) {
                simulink_w_integrator_DW.StreamCall1_State =
                  STREAM_CALL_STATE_CONNECTED;
                stream_set_byte_order
                  (simulink_w_integrator_DW.StreamCall1_Stream,
                   (t_stream_byte_order)
                   simulink_w_integrator_P.StreamCall1_Endian);
                rtb_StreamCall1_o1 =
                  &simulink_w_integrator_DW.StreamCall1_Stream;
              } else if (result == -QERR_WOULD_BLOCK) {
                simulink_w_integrator_DW.StreamCall1_State =
                  STREAM_CALL_STATE_CONNECTING;
                result = 0;
              }
            }
          }
          break;
        }

       case STREAM_CALL_STATE_CONNECTING:
        {
          if (!close_flag) {
            const t_timeout timeout = { 0, 0, false };/* zero seconds */

            result = stream_poll(simulink_w_integrator_DW.StreamCall1_Stream,
                                 &timeout, STREAM_POLL_CONNECT);
            if (result > 0) {
              simulink_w_integrator_DW.StreamCall1_State =
                STREAM_CALL_STATE_CONNECTED;
              stream_set_byte_order(simulink_w_integrator_DW.StreamCall1_Stream,
                                    (t_stream_byte_order)
                                    simulink_w_integrator_P.StreamCall1_Endian);
              rtb_StreamCall1_o1 = &simulink_w_integrator_DW.StreamCall1_Stream;
              result = 0;
              break;
            } else if (result == 0) {
              break;
            }
          }

          /* Fall through deliberately */
        }

       case STREAM_CALL_STATE_CONNECTED:
        {
          rtb_StreamCall1_o1 = &simulink_w_integrator_DW.StreamCall1_Stream;
          if (!close_flag) {
            break;
          }

          /* Fall through deliberately */
        }

       default:
        {
          t_error close_result = stream_close
            (simulink_w_integrator_DW.StreamCall1_Stream);
          if (close_result == 0) {
            simulink_w_integrator_DW.StreamCall1_State =
              STREAM_CALL_STATE_NOT_CONNECTED;
            simulink_w_integrator_DW.StreamCall1_Stream = NULL;
            rtb_StreamCall1_o1 = NULL;
          } else if (result == 0) {
            result = close_result;
          }
          break;
        }
      }

      rtb_StreamCall1_o2 = simulink_w_integrator_DW.StreamCall1_State;
      rtb_StreamCall1_o3 = (int32_T) result;
    }

    /* S-Function (stream_formatted_write_block): '<S11>/Stream Formatted Write' incorporates:
     *  Constant: '<S11>/Constant1'
     */
    {
      t_error result;
      if (rtb_StreamCall1_o1 != NULL) {
        result = stream_print_utf8_char_array(*rtb_StreamCall1_o1,
          simulink_w_integrator_P.StreamFormattedWrite_MaxUnits,
          &rtb_StreamFormattedWrite_o2, "%c\n"
          , (char) simulink_w_integrator_P.Constant1_Value
          );
        if (result > 0) {
          result = stream_flush(*rtb_StreamCall1_o1);
        }

        if (result == -QERR_WOULD_BLOCK) {
          result = 0;
        }
      }
    }

    /* S-Function (stream_read_block): '<S11>/Stream Read1' incorporates:
     *  Constant: '<S11>/Constant1'
     *  S-Function (stream_call_block): '<S11>/Stream Call1'
     *  S-Function (stream_formatted_write_block): '<S11>/Stream Formatted Write'
     */

    /* S-Function Block: simulink_w_integrator/IMU system/IMU/Stream Read1 (stream_read_block) */
    {
      t_error result;
      memset(&rtb_StreamRead1_o2[0], 0, 10 * sizeof(real32_T));
      if (((t_stream_ptr)rtb_StreamCall1_o1) != NULL) {
        result = stream_receive_unit_array(*((t_stream_ptr)rtb_StreamCall1_o1),
          &rtb_StreamRead1_o2[0], sizeof(real32_T), 10);
        rtb_StreamRead1_o3 = (result > 0);
        if (result > 0 || result == -QERR_WOULD_BLOCK) {
          result = 0;
        }
      } else {
        rtb_StreamRead1_o3 = false;
        result = 0;
      }
    }

    for (i = 0; i < 10; i++) {
      /* Switch: '<S11>/Switch' */
      if (rtb_StreamRead1_o3) {
        /* Switch: '<S11>/Switch' incorporates:
         *  DataTypeConversion: '<S11>/Data Type Conversion'
         */
        simulink_w_integrator_B.Switch[i] = rtb_StreamRead1_o2[i];
      } else {
        /* Switch: '<S11>/Switch' incorporates:
         *  Memory: '<S11>/Memory'
         */
        simulink_w_integrator_B.Switch[i] =
          simulink_w_integrator_DW.Memory_PreviousInput[i];
      }

      /* End of Switch: '<S11>/Switch' */
    }

    /* Gain: '<S11>/Gain2' */
    for (i = 0; i < 3; i++) {
      rtb_Gain2[i] = simulink_w_integrator_P.Gain2_Gain[i + 6] *
        simulink_w_integrator_B.Switch[2] +
        (simulink_w_integrator_P.Gain2_Gain[i + 3] *
         simulink_w_integrator_B.Switch[1] +
         simulink_w_integrator_P.Gain2_Gain[i] * simulink_w_integrator_B.Switch
         [0]);
    }

    /* End of Gain: '<S11>/Gain2' */

    /* MATLAB Function: '<S5>/MATLAB Function' */
    ay = rtb_Gain2[1];
    az = rtb_Gain2[2];

    /* MATLAB Function 'IMU system/MATLAB Function': '<S12>:1' */
    if (rtb_Gain2[2] == 0.0) {
      /* '<S12>:1:2' */
      /* '<S12>:1:3' */
      az = 0.01;
      if (rtb_Gain2[1] == 0.0) {
        /* '<S12>:1:4' */
        /* '<S12>:1:5' */
        ay = 0.01;
      }
    }

    /* Sum: '<S5>/Sum' incorporates:
     *  Constant: '<S5>/Constant'
     *  MATLAB Function: '<S5>/MATLAB Function'
     */
    /* '<S12>:1:8' */
    /* '<S12>:1:9' */
    simulink_w_integrator_B.Sum = atan(ay / az) +
      simulink_w_integrator_P.Constant_Value;

    /* Sum: '<S5>/Sum1' incorporates:
     *  Constant: '<S5>/Constant1'
     *  MATLAB Function: '<S5>/MATLAB Function'
     */
    simulink_w_integrator_B.Sum1 = atan(rtb_Gain2[0] / sqrt(ay * ay + az * az))
      + simulink_w_integrator_P.Constant1_Value_n;

    /* MATLAB Function: '<S5>/Gyro vector to [pitch rate, elevation rate, travle rate]' incorporates:
     *  SignalConversion generated from: '<S10>/ SFunction '
     */
    /* MATLAB Function 'IMU system/Gyro vector to [pitch rate, elevation rate, travle rate]': '<S10>:1' */
    /* '<S10>:1:3' */
    /* '<S10>:1:4' */
    /* '<S10>:1:8' */
    /* '<S10>:1:11' */
    ay = tan(simulink_w_integrator_B.Sum1);
    az = sin(simulink_w_integrator_B.Sum);
    euler_rates_tmp = cos(simulink_w_integrator_B.Sum);
    euler_rates_tmp_0 = cos(simulink_w_integrator_B.Sum1);
    tmp_2[0] = 1.0;
    tmp_2[3] = az * ay;
    tmp_2[6] = euler_rates_tmp * ay;
    tmp_2[1] = 0.0;
    tmp_2[4] = euler_rates_tmp;
    tmp_2[7] = -az;
    tmp_2[2] = 0.0;
    tmp_2[5] = az / euler_rates_tmp_0;
    tmp_2[8] = euler_rates_tmp / euler_rates_tmp_0;

    /* Gain: '<S11>/Gain1' */
    for (i = 0; i < 3; i++) {
      rtb_Gain2[i] = simulink_w_integrator_P.Gain1_Gain[i + 6] *
        simulink_w_integrator_B.Switch[5] +
        (simulink_w_integrator_P.Gain1_Gain[i + 3] *
         simulink_w_integrator_B.Switch[4] +
         simulink_w_integrator_P.Gain1_Gain[i] * simulink_w_integrator_B.Switch
         [3]);
    }

    /* End of Gain: '<S11>/Gain1' */

    /* MATLAB Function: '<S5>/Gyro vector to [pitch rate, elevation rate, travle rate]' */
    for (i = 0; i < 3; i++) {
      simulink_w_integrator_B.euler_rates[i] = 0.0;
      simulink_w_integrator_B.euler_rates[i] += tmp_2[i] * rtb_Gain2[0];
      simulink_w_integrator_B.euler_rates[i] += tmp_2[i + 3] * rtb_Gain2[1];
      simulink_w_integrator_B.euler_rates[i] += tmp_2[i + 6] * rtb_Gain2[2];
    }
  }

  /* Sum: '<S3>/Sum1' */
  tmp[0] = simulink_w_integrator_B.Sum - rtb_Sum1[0];
  tmp[1] = simulink_w_integrator_B.euler_rates[0] - rtb_Sum1[1];
  tmp[2] = simulink_w_integrator_B.Sum1 - rtb_Sum1[2];
  tmp[3] = simulink_w_integrator_B.euler_rates[1] - rtb_Sum1[3];
  tmp[4] = simulink_w_integrator_B.euler_rates[2] - rtb_Sum1[4];
  for (i = 0; i < 5; i++) {
    rtb_Sum1[i] = tmp[i];
  }

  /* End of Sum: '<S3>/Sum1' */
  for (i = 0; i < 5; i++) {
    /* Gain: '<S3>/Gain1' */
    tmp[i] = 0.0;

    /* Sum: '<S3>/Sum' */
    ay = 0.0;
    for (i_0 = 0; i_0 < 5; i_0++) {
      /* Gain: '<S3>/Gain1' incorporates:
       *  Gain: '<S3>/A_est'
       *  Sum: '<S3>/Sum'
       */
      tmp_3 = 5 * i_0 + i;

      /* Sum: '<S3>/Sum' incorporates:
       *  Gain: '<S3>/A_est'
       *  Integrator: '<S3>/Integrator'
       */
      ay += simulink_w_integrator_P.A_est[tmp_3] *
        simulink_w_integrator_X.Integrator_CSTATE[i_0];

      /* Gain: '<S3>/Gain1' */
      tmp[i] += simulink_w_integrator_P.L[tmp_3] * rtb_Sum1[i_0];
    }

    /* Sum: '<S3>/Sum' incorporates:
     *  Gain: '<S3>/A_est'
     *  Gain: '<S3>/B_est'
     */
    simulink_w_integrator_B.Sum_p[i] = (tmp[i] +
      (simulink_w_integrator_P.B_est[i + 5] * rtb_Sum[1] +
       simulink_w_integrator_P.B_est[i] * rtb_Sum[0])) + ay;
  }

  if (rtmIsMajorTimeStep(simulink_w_integrator_M)) {
    /* Gain: '<S4>/Elevation: Count to rad' */
    simulink_w_integrator_B.ElevationCounttorad =
      simulink_w_integrator_P.ElevationCounttorad_Gain *
      rtb_HILReadEncoderTimebase_o3;

    /* Gain: '<S4>/Pitch: Count to rad' */
    simulink_w_integrator_B.PitchCounttorad =
      simulink_w_integrator_P.PitchCounttorad_Gain *
      rtb_HILReadEncoderTimebase_o2;

    /* Gain: '<S4>/Travel: Count to rad' */
    simulink_w_integrator_B.TravelCounttorad =
      simulink_w_integrator_P.TravelCounttorad_Gain *
      rtb_HILReadEncoderTimebase_o1;
  }

  /* Saturate: '<S4>/Front motor: Saturation' */
  if (rtb_Frontgain > simulink_w_integrator_P.FrontmotorSaturation_UpperSat) {
    /* Saturate: '<S4>/Front motor: Saturation' */
    simulink_w_integrator_B.FrontmotorSaturation =
      simulink_w_integrator_P.FrontmotorSaturation_UpperSat;
  } else if (rtb_Frontgain <
             simulink_w_integrator_P.FrontmotorSaturation_LowerSat) {
    /* Saturate: '<S4>/Front motor: Saturation' */
    simulink_w_integrator_B.FrontmotorSaturation =
      simulink_w_integrator_P.FrontmotorSaturation_LowerSat;
  } else {
    /* Saturate: '<S4>/Front motor: Saturation' */
    simulink_w_integrator_B.FrontmotorSaturation = rtb_Frontgain;
  }

  /* End of Saturate: '<S4>/Front motor: Saturation' */

  /* Saturate: '<S4>/Back motor: Saturation' */
  if (rtb_Backgain > simulink_w_integrator_P.BackmotorSaturation_UpperSat) {
    /* Saturate: '<S4>/Back motor: Saturation' */
    simulink_w_integrator_B.BackmotorSaturation =
      simulink_w_integrator_P.BackmotorSaturation_UpperSat;
  } else if (rtb_Backgain < simulink_w_integrator_P.BackmotorSaturation_LowerSat)
  {
    /* Saturate: '<S4>/Back motor: Saturation' */
    simulink_w_integrator_B.BackmotorSaturation =
      simulink_w_integrator_P.BackmotorSaturation_LowerSat;
  } else {
    /* Saturate: '<S4>/Back motor: Saturation' */
    simulink_w_integrator_B.BackmotorSaturation = rtb_Backgain;
  }

  /* End of Saturate: '<S4>/Back motor: Saturation' */
  if (rtmIsMajorTimeStep(simulink_w_integrator_M)) {
    /* S-Function (hil_write_analog_block): '<S4>/HIL Write Analog' */

    /* S-Function Block: simulink_w_integrator/Heli 3D/HIL Write Analog (hil_write_analog_block) */
    {
      t_error result;
      simulink_w_integrator_DW.HILWriteAnalog_Buffer[0] =
        simulink_w_integrator_B.FrontmotorSaturation;
      simulink_w_integrator_DW.HILWriteAnalog_Buffer[1] =
        simulink_w_integrator_B.BackmotorSaturation;
      result = hil_write_analog(simulink_w_integrator_DW.HILInitialize_Card,
        simulink_w_integrator_P.HILWriteAnalog_channels, 2,
        &simulink_w_integrator_DW.HILWriteAnalog_Buffer[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
      }
    }

    /* S-Function (stop_with_error_block): '<S11>/Stop with Call Error' */

    /* S-Function Block: simulink_w_integrator/IMU system/IMU/Stop with Call Error (stop_with_error_block) */
    {
      if (rtb_StreamCall1_o3 < 0) {
        msg_get_error_messageA(NULL, rtb_StreamCall1_o3, _rt_error_message,
          sizeof(_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }
    }
  }

  /* Sum: '<S7>/Sum1' incorporates:
   *  Integrator: '<S3>/Integrator'
   */
  simulink_w_integrator_B.Sum1_o = rtb_Sum3 -
    simulink_w_integrator_X.Integrator_CSTATE[0];

  /* Sum: '<S7>/Sum2' incorporates:
   *  Integrator: '<S3>/Integrator'
   */
  simulink_w_integrator_B.Sum2 = rtb_Sum4 -
    simulink_w_integrator_X.Integrator_CSTATE[3];

  /* Integrator: '<S9>/Integrator' */
  /* Limited  Integrator  */
  if (simulink_w_integrator_X.Integrator_CSTATE_n >=
      simulink_w_integrator_P.Integrator_UpperSat) {
    simulink_w_integrator_X.Integrator_CSTATE_n =
      simulink_w_integrator_P.Integrator_UpperSat;
  } else {
    if (simulink_w_integrator_X.Integrator_CSTATE_n <=
        simulink_w_integrator_P.Integrator_LowerSat) {
      simulink_w_integrator_X.Integrator_CSTATE_n =
        simulink_w_integrator_P.Integrator_LowerSat;
    }
  }

  /* End of Integrator: '<S9>/Integrator' */
  if (rtmIsMajorTimeStep(simulink_w_integrator_M)) {
    /* Gain: '<S9>/K_ei' incorporates:
     *  Sum: '<S2>/Sum'
     */
    simulink_w_integrator_B.K_ei = simulink_w_integrator_P.K_ei_Gain * 0.0;
  }
}

/* Model update function for TID0 */
void simulink_w_integrator_update0(void) /* Sample time: [0.0s, 0.0s] */
{
  if (rtmIsMajorTimeStep(simulink_w_integrator_M)) {
    /* Update for Memory: '<S11>/Memory' */
    memcpy(&simulink_w_integrator_DW.Memory_PreviousInput[0],
           &simulink_w_integrator_B.Switch[0], 10U * sizeof(real_T));
  }

  if (rtmIsMajorTimeStep(simulink_w_integrator_M)) {
    rt_ertODEUpdateContinuousStates(&simulink_w_integrator_M->solverInfo);
  }

  /* Update absolute time */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++simulink_w_integrator_M->Timing.clockTick0)) {
    ++simulink_w_integrator_M->Timing.clockTickH0;
  }

  simulink_w_integrator_M->Timing.t[0] = rtsiGetSolverStopTime
    (&simulink_w_integrator_M->solverInfo);

  /* Update absolute time */
  /* The "clockTick1" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick1"
   * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick1 and the high bits
   * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++simulink_w_integrator_M->Timing.clockTick1)) {
    ++simulink_w_integrator_M->Timing.clockTickH1;
  }

  simulink_w_integrator_M->Timing.t[1] =
    simulink_w_integrator_M->Timing.clockTick1 *
    simulink_w_integrator_M->Timing.stepSize1 +
    simulink_w_integrator_M->Timing.clockTickH1 *
    simulink_w_integrator_M->Timing.stepSize1 * 4294967296.0;
}

/* Derivatives for root system: '<Root>' */
void simulink_w_integrator_derivatives(void)
{
  XDot_simulink_w_integrator_T *_rtXdot;
  int32_T i;
  boolean_T lsat;
  boolean_T usat;
  _rtXdot = ((XDot_simulink_w_integrator_T *) simulink_w_integrator_M->derivs);

  /* Derivatives for Integrator: '<S3>/Integrator' */
  for (i = 0; i < 5; i++) {
    _rtXdot->Integrator_CSTATE[i] = simulink_w_integrator_B.Sum_p[i];
  }

  /* End of Derivatives for Integrator: '<S3>/Integrator' */

  /* Derivatives for Integrator: '<S7>/Integrator' */
  _rtXdot->Integrator_CSTATE_m = simulink_w_integrator_B.Sum1_o;

  /* Derivatives for Integrator: '<S7>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = simulink_w_integrator_B.Sum2;

  /* Derivatives for TransferFcn: '<S4>/Elevation: Transfer Fcn' */
  _rtXdot->ElevationTransferFcn_CSTATE = 0.0;
  _rtXdot->ElevationTransferFcn_CSTATE +=
    simulink_w_integrator_P.ElevationTransferFcn_A *
    simulink_w_integrator_X.ElevationTransferFcn_CSTATE;
  _rtXdot->ElevationTransferFcn_CSTATE +=
    simulink_w_integrator_B.ElevationCounttorad;

  /* Derivatives for TransferFcn: '<S4>/Pitch: Transfer Fcn' */
  _rtXdot->PitchTransferFcn_CSTATE = 0.0;
  _rtXdot->PitchTransferFcn_CSTATE += simulink_w_integrator_P.PitchTransferFcn_A
    * simulink_w_integrator_X.PitchTransferFcn_CSTATE;
  _rtXdot->PitchTransferFcn_CSTATE += simulink_w_integrator_B.PitchCounttorad;

  /* Derivatives for TransferFcn: '<S4>/Travel: Transfer Fcn' */
  _rtXdot->TravelTransferFcn_CSTATE = 0.0;
  _rtXdot->TravelTransferFcn_CSTATE +=
    simulink_w_integrator_P.TravelTransferFcn_A *
    simulink_w_integrator_X.TravelTransferFcn_CSTATE;
  _rtXdot->TravelTransferFcn_CSTATE += simulink_w_integrator_B.TravelCounttorad;

  /* Derivatives for Integrator: '<S9>/Integrator' */
  lsat = (simulink_w_integrator_X.Integrator_CSTATE_n <=
          simulink_w_integrator_P.Integrator_LowerSat);
  usat = (simulink_w_integrator_X.Integrator_CSTATE_n >=
          simulink_w_integrator_P.Integrator_UpperSat);
  if (((!lsat) && (!usat)) || (lsat && (simulink_w_integrator_B.K_ei > 0.0)) ||
      (usat && (simulink_w_integrator_B.K_ei < 0.0))) {
    _rtXdot->Integrator_CSTATE_n = simulink_w_integrator_B.K_ei;
  } else {
    /* in saturation */
    _rtXdot->Integrator_CSTATE_n = 0.0;
  }

  /* End of Derivatives for Integrator: '<S9>/Integrator' */
}

/* Model output function for TID2 */
void simulink_w_integrator_output2(void) /* Sample time: [0.01s, 0.0s] */
{
  /* local block i/o variables */
  real_T rtb_GameController_o4;
  real_T rtb_GameController_o5;

  /* S-Function (game_controller_block): '<S6>/Game Controller' */

  /* S-Function Block: simulink_w_integrator/Joystick/Game Controller (game_controller_block) */
  {
    if (simulink_w_integrator_P.GameController_Enabled) {
      t_game_controller_states state;
      t_boolean new_data;
      t_error result;
      result = game_controller_poll
        (simulink_w_integrator_DW.GameController_Controller, &state, &new_data);
      if (result < 0) {
        new_data = false;
      }

      rtb_GameController_o4 = state.x;
      rtb_GameController_o5 = state.y;
    } else {
      rtb_GameController_o4 = 0;
      rtb_GameController_o5 = 0;
    }
  }

  /* RateTransition: '<S6>/Rate Transition: x' */
  simulink_w_integrator_DW.RateTransitionx_Buffer0 = rtb_GameController_o4;

  /* RateTransition: '<S6>/Rate Transition: y' */
  simulink_w_integrator_DW.RateTransitiony_Buffer0 = rtb_GameController_o5;
}

/* Model update function for TID2 */
void simulink_w_integrator_update2(void) /* Sample time: [0.01s, 0.0s] */
{
  /* Update absolute time */
  /* The "clockTick2" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick2"
   * and "Timing.stepSize2". Size of "clockTick2" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick2 and the high bits
   * Timing.clockTickH2. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++simulink_w_integrator_M->Timing.clockTick2)) {
    ++simulink_w_integrator_M->Timing.clockTickH2;
  }

  simulink_w_integrator_M->Timing.t[2] =
    simulink_w_integrator_M->Timing.clockTick2 *
    simulink_w_integrator_M->Timing.stepSize2 +
    simulink_w_integrator_M->Timing.clockTickH2 *
    simulink_w_integrator_M->Timing.stepSize2 * 4294967296.0;
}

/* Model output wrapper function for compatibility with a static main program */
void simulink_w_integrator_output(int_T tid)
{
  switch (tid) {
   case 0 :
    simulink_w_integrator_output0();
    break;

   case 2 :
    simulink_w_integrator_output2();
    break;

   default :
    break;
  }
}

/* Model update wrapper function for compatibility with a static main program */
void simulink_w_integrator_update(int_T tid)
{
  switch (tid) {
   case 0 :
    simulink_w_integrator_update0();
    break;

   case 2 :
    simulink_w_integrator_update2();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void simulink_w_integrator_initialize(void)
{
  /* Start for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: simulink_w_integrator/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("q8_usb", "0",
                      &simulink_w_integrator_DW.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
      return;
    }

    is_switching = false;
    result = hil_set_card_specific_options
      (simulink_w_integrator_DW.HILInitialize_Card,
       "update_rate=normal;decimation=1", 32);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
      return;
    }

    result = hil_watchdog_clear(simulink_w_integrator_DW.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
      return;
    }

    if ((simulink_w_integrator_P.HILInitialize_AIPStart && !is_switching) ||
        (simulink_w_integrator_P.HILInitialize_AIPEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AIMinimums =
          &simulink_w_integrator_DW.HILInitialize_AIMinimums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AIMinimums[i1] = (simulink_w_integrator_P.HILInitialize_AILow);
        }
      }

      {
        int_T i1;
        real_T *dw_AIMaximums =
          &simulink_w_integrator_DW.HILInitialize_AIMaximums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AIMaximums[i1] = simulink_w_integrator_P.HILInitialize_AIHigh;
        }
      }

      result = hil_set_analog_input_ranges
        (simulink_w_integrator_DW.HILInitialize_Card,
         simulink_w_integrator_P.HILInitialize_AIChannels, 8U,
         &simulink_w_integrator_DW.HILInitialize_AIMinimums[0],
         &simulink_w_integrator_DW.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_w_integrator_P.HILInitialize_AOPStart && !is_switching) ||
        (simulink_w_integrator_P.HILInitialize_AOPEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOMinimums =
          &simulink_w_integrator_DW.HILInitialize_AOMinimums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOMinimums[i1] = (simulink_w_integrator_P.HILInitialize_AOLow);
        }
      }

      {
        int_T i1;
        real_T *dw_AOMaximums =
          &simulink_w_integrator_DW.HILInitialize_AOMaximums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOMaximums[i1] = simulink_w_integrator_P.HILInitialize_AOHigh;
        }
      }

      result = hil_set_analog_output_ranges
        (simulink_w_integrator_DW.HILInitialize_Card,
         simulink_w_integrator_P.HILInitialize_AOChannels, 8U,
         &simulink_w_integrator_DW.HILInitialize_AOMinimums[0],
         &simulink_w_integrator_DW.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_w_integrator_P.HILInitialize_AOStart && !is_switching) ||
        (simulink_w_integrator_P.HILInitialize_AOEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOVoltages =
          &simulink_w_integrator_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = simulink_w_integrator_P.HILInitialize_AOInitial;
        }
      }

      result = hil_write_analog(simulink_w_integrator_DW.HILInitialize_Card,
        simulink_w_integrator_P.HILInitialize_AOChannels, 8U,
        &simulink_w_integrator_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }
    }

    if (simulink_w_integrator_P.HILInitialize_AOReset) {
      {
        int_T i1;
        real_T *dw_AOVoltages =
          &simulink_w_integrator_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = simulink_w_integrator_P.HILInitialize_AOWatchdog;
        }
      }

      result = hil_watchdog_set_analog_expiration_state
        (simulink_w_integrator_DW.HILInitialize_Card,
         simulink_w_integrator_P.HILInitialize_AOChannels, 8U,
         &simulink_w_integrator_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_w_integrator_P.HILInitialize_EIPStart && !is_switching) ||
        (simulink_w_integrator_P.HILInitialize_EIPEnter && is_switching)) {
      {
        int_T i1;
        int32_T *dw_QuadratureModes =
          &simulink_w_integrator_DW.HILInitialize_QuadratureModes[0];
        for (i1=0; i1 < 8; i1++) {
          dw_QuadratureModes[i1] =
            simulink_w_integrator_P.HILInitialize_EIQuadrature;
        }
      }

      result = hil_set_encoder_quadrature_mode
        (simulink_w_integrator_DW.HILInitialize_Card,
         simulink_w_integrator_P.HILInitialize_EIChannels, 8U,
         (t_encoder_quadrature_mode *)
         &simulink_w_integrator_DW.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_w_integrator_P.HILInitialize_EIStart && !is_switching) ||
        (simulink_w_integrator_P.HILInitialize_EIEnter && is_switching)) {
      {
        int_T i1;
        int32_T *dw_InitialEICounts =
          &simulink_w_integrator_DW.HILInitialize_InitialEICounts[0];
        for (i1=0; i1 < 8; i1++) {
          dw_InitialEICounts[i1] =
            simulink_w_integrator_P.HILInitialize_EIInitial;
        }
      }

      result = hil_set_encoder_counts
        (simulink_w_integrator_DW.HILInitialize_Card,
         simulink_w_integrator_P.HILInitialize_EIChannels, 8U,
         &simulink_w_integrator_DW.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_w_integrator_P.HILInitialize_POPStart && !is_switching) ||
        (simulink_w_integrator_P.HILInitialize_POPEnter && is_switching)) {
      uint32_T num_duty_cycle_modes = 0;
      uint32_T num_frequency_modes = 0;

      {
        int_T i1;
        int32_T *dw_POModeValues =
          &simulink_w_integrator_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POModeValues[i1] = simulink_w_integrator_P.HILInitialize_POModes;
        }
      }

      result = hil_set_pwm_mode(simulink_w_integrator_DW.HILInitialize_Card,
        simulink_w_integrator_P.HILInitialize_POChannels, 8U, (t_pwm_mode *)
        &simulink_w_integrator_DW.HILInitialize_POModeValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }

      {
        int_T i1;
        const uint32_T *p_HILInitialize_POChannels =
          simulink_w_integrator_P.HILInitialize_POChannels;
        int32_T *dw_POModeValues =
          &simulink_w_integrator_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          if (dw_POModeValues[i1] == PWM_DUTY_CYCLE_MODE || dw_POModeValues[i1] ==
              PWM_ONE_SHOT_MODE || dw_POModeValues[i1] == PWM_TIME_MODE ||
              dw_POModeValues[i1] == PWM_RAW_MODE) {
            simulink_w_integrator_DW.HILInitialize_POSortedChans[num_duty_cycle_modes]
              = (p_HILInitialize_POChannels[i1]);
            simulink_w_integrator_DW.HILInitialize_POSortedFreqs[num_duty_cycle_modes]
              = simulink_w_integrator_P.HILInitialize_POFrequency;
            num_duty_cycle_modes++;
          } else {
            simulink_w_integrator_DW.HILInitialize_POSortedChans[7U -
              num_frequency_modes] = (p_HILInitialize_POChannels[i1]);
            simulink_w_integrator_DW.HILInitialize_POSortedFreqs[7U -
              num_frequency_modes] =
              simulink_w_integrator_P.HILInitialize_POFrequency;
            num_frequency_modes++;
          }
        }
      }

      if (num_duty_cycle_modes > 0) {
        result = hil_set_pwm_frequency
          (simulink_w_integrator_DW.HILInitialize_Card,
           &simulink_w_integrator_DW.HILInitialize_POSortedChans[0],
           num_duty_cycle_modes,
           &simulink_w_integrator_DW.HILInitialize_POSortedFreqs[0]);
        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
          return;
        }
      }

      if (num_frequency_modes > 0) {
        result = hil_set_pwm_duty_cycle
          (simulink_w_integrator_DW.HILInitialize_Card,
           &simulink_w_integrator_DW.HILInitialize_POSortedChans[num_duty_cycle_modes],
           num_frequency_modes,
           &simulink_w_integrator_DW.HILInitialize_POSortedFreqs[num_duty_cycle_modes]);
        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
          return;
        }
      }

      {
        int_T i1;
        int32_T *dw_POModeValues =
          &simulink_w_integrator_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POModeValues[i1] =
            simulink_w_integrator_P.HILInitialize_POConfiguration;
        }
      }

      {
        int_T i1;
        int32_T *dw_POAlignValues =
          &simulink_w_integrator_DW.HILInitialize_POAlignValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POAlignValues[i1] =
            simulink_w_integrator_P.HILInitialize_POAlignment;
        }
      }

      {
        int_T i1;
        int32_T *dw_POPolarityVals =
          &simulink_w_integrator_DW.HILInitialize_POPolarityVals[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POPolarityVals[i1] =
            simulink_w_integrator_P.HILInitialize_POPolarity;
        }
      }

      result = hil_set_pwm_configuration
        (simulink_w_integrator_DW.HILInitialize_Card,
         simulink_w_integrator_P.HILInitialize_POChannels, 8U,
         (t_pwm_configuration *)
         &simulink_w_integrator_DW.HILInitialize_POModeValues[0],
         (t_pwm_alignment *)
         &simulink_w_integrator_DW.HILInitialize_POAlignValues[0],
         (t_pwm_polarity *)
         &simulink_w_integrator_DW.HILInitialize_POPolarityVals[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }

      {
        int_T i1;
        real_T *dw_POSortedFreqs =
          &simulink_w_integrator_DW.HILInitialize_POSortedFreqs[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POSortedFreqs[i1] = simulink_w_integrator_P.HILInitialize_POLeading;
        }
      }

      {
        int_T i1;
        real_T *dw_POValues = &simulink_w_integrator_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_w_integrator_P.HILInitialize_POTrailing;
        }
      }

      result = hil_set_pwm_deadband(simulink_w_integrator_DW.HILInitialize_Card,
        simulink_w_integrator_P.HILInitialize_POChannels, 8U,
        &simulink_w_integrator_DW.HILInitialize_POSortedFreqs[0],
        &simulink_w_integrator_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_w_integrator_P.HILInitialize_POStart && !is_switching) ||
        (simulink_w_integrator_P.HILInitialize_POEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_POValues = &simulink_w_integrator_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_w_integrator_P.HILInitialize_POInitial;
        }
      }

      result = hil_write_pwm(simulink_w_integrator_DW.HILInitialize_Card,
        simulink_w_integrator_P.HILInitialize_POChannels, 8U,
        &simulink_w_integrator_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }
    }

    if (simulink_w_integrator_P.HILInitialize_POReset) {
      {
        int_T i1;
        real_T *dw_POValues = &simulink_w_integrator_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_w_integrator_P.HILInitialize_POWatchdog;
        }
      }

      result = hil_watchdog_set_pwm_expiration_state
        (simulink_w_integrator_DW.HILInitialize_Card,
         simulink_w_integrator_P.HILInitialize_POChannels, 8U,
         &simulink_w_integrator_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for S-Function (hil_read_encoder_timebase_block): '<S4>/HIL Read Encoder Timebase' */

  /* S-Function Block: simulink_w_integrator/Heli 3D/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
  {
    t_error result;
    result = hil_task_create_encoder_reader
      (simulink_w_integrator_DW.HILInitialize_Card,
       simulink_w_integrator_P.HILReadEncoderTimebase_SamplesI,
       simulink_w_integrator_P.HILReadEncoderTimebase_Channels, 3,
       &simulink_w_integrator_DW.HILReadEncoderTimebase_Task);
    if (result >= 0) {
      result = hil_task_set_buffer_overflow_mode
        (simulink_w_integrator_DW.HILReadEncoderTimebase_Task,
         (t_buffer_overflow_mode)
         (simulink_w_integrator_P.HILReadEncoderTimebase_Overflow - 1));
    }

    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
    }
  }

  /* Start for RateTransition: '<S6>/Rate Transition: x' */
  simulink_w_integrator_B.RateTransitionx =
    simulink_w_integrator_P.RateTransitionx_InitialConditio;

  /* Start for RateTransition: '<S6>/Rate Transition: y' */
  simulink_w_integrator_B.RateTransitiony =
    simulink_w_integrator_P.RateTransitiony_InitialConditio;

  /* Start for S-Function (stream_call_block): '<S11>/Stream Call1' incorporates:
   *  Constant: '<S11>/Constant'
   *  S-Function (string_constant_block): '<S11>/String Constant'
   */

  /* S-Function Block: simulink_w_integrator/IMU system/IMU/Stream Call1 (stream_call_block) */
  {
    simulink_w_integrator_DW.StreamCall1_State = STREAM_CALL_STATE_NOT_CONNECTED;
    simulink_w_integrator_DW.StreamCall1_Stream = NULL;
  }

  /* Start for S-Function (game_controller_block): '<S6>/Game Controller' */

  /* S-Function Block: simulink_w_integrator/Joystick/Game Controller (game_controller_block) */
  {
    if (simulink_w_integrator_P.GameController_Enabled) {
      t_double deadzone[6];
      t_double saturation[6];
      t_int index;
      t_error result;
      for (index = 0; index < 6; index++) {
        deadzone[index] = -1;
      }

      for (index = 0; index < 6; index++) {
        saturation[index] = -1;
      }

      result = game_controller_open
        (simulink_w_integrator_P.GameController_ControllerNumber,
         simulink_w_integrator_P.GameController_BufferSize, deadzone, saturation,
         simulink_w_integrator_P.GameController_AutoCenter, 0, 1.0,
         &simulink_w_integrator_DW.GameController_Controller);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
      }
    }
  }

  {
    int32_T i;

    /* InitializeConditions for RateTransition: '<S6>/Rate Transition: x' */
    simulink_w_integrator_DW.RateTransitionx_Buffer0 =
      simulink_w_integrator_P.RateTransitionx_InitialConditio;

    /* InitializeConditions for Integrator: '<S3>/Integrator' */
    for (i = 0; i < 5; i++) {
      simulink_w_integrator_X.Integrator_CSTATE[i] =
        simulink_w_integrator_P.Integrator_IC;
    }

    /* End of InitializeConditions for Integrator: '<S3>/Integrator' */

    /* InitializeConditions for RateTransition: '<S6>/Rate Transition: y' */
    simulink_w_integrator_DW.RateTransitiony_Buffer0 =
      simulink_w_integrator_P.RateTransitiony_InitialConditio;

    /* InitializeConditions for Integrator: '<S7>/Integrator' */
    simulink_w_integrator_X.Integrator_CSTATE_m =
      simulink_w_integrator_P.Integrator_IC_j;

    /* InitializeConditions for Integrator: '<S7>/Integrator1' */
    simulink_w_integrator_X.Integrator1_CSTATE =
      simulink_w_integrator_P.Integrator1_IC;

    /* InitializeConditions for Memory: '<S11>/Memory' */
    memcpy(&simulink_w_integrator_DW.Memory_PreviousInput[0],
           &simulink_w_integrator_P.Memory_InitialCondition[0], 10U * sizeof
           (real_T));

    /* InitializeConditions for TransferFcn: '<S4>/Elevation: Transfer Fcn' */
    simulink_w_integrator_X.ElevationTransferFcn_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S4>/Pitch: Transfer Fcn' */
    simulink_w_integrator_X.PitchTransferFcn_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S4>/Travel: Transfer Fcn' */
    simulink_w_integrator_X.TravelTransferFcn_CSTATE = 0.0;

    /* InitializeConditions for Integrator: '<S9>/Integrator' */
    simulink_w_integrator_X.Integrator_CSTATE_n =
      simulink_w_integrator_P.Integrator_IC_k;
  }
}

/* Model terminate function */
void simulink_w_integrator_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: simulink_w_integrator/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    t_uint32 num_final_analog_outputs = 0;
    t_uint32 num_final_pwm_outputs = 0;
    hil_task_stop_all(simulink_w_integrator_DW.HILInitialize_Card);
    hil_monitor_stop_all(simulink_w_integrator_DW.HILInitialize_Card);
    is_switching = false;
    if ((simulink_w_integrator_P.HILInitialize_AOTerminate && !is_switching) ||
        (simulink_w_integrator_P.HILInitialize_AOExit && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOVoltages =
          &simulink_w_integrator_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = simulink_w_integrator_P.HILInitialize_AOFinal;
        }
      }

      num_final_analog_outputs = 8U;
    } else {
      num_final_analog_outputs = 0;
    }

    if ((simulink_w_integrator_P.HILInitialize_POTerminate && !is_switching) ||
        (simulink_w_integrator_P.HILInitialize_POExit && is_switching)) {
      {
        int_T i1;
        real_T *dw_POValues = &simulink_w_integrator_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_w_integrator_P.HILInitialize_POFinal;
        }
      }

      num_final_pwm_outputs = 8U;
    } else {
      num_final_pwm_outputs = 0;
    }

    if (0
        || num_final_analog_outputs > 0
        || num_final_pwm_outputs > 0
        ) {
      /* Attempt to write the final outputs atomically (due to firmware issue in old Q2-USB). Otherwise write channels individually */
      result = hil_write(simulink_w_integrator_DW.HILInitialize_Card
                         , simulink_w_integrator_P.HILInitialize_AOChannels,
                         num_final_analog_outputs
                         , simulink_w_integrator_P.HILInitialize_POChannels,
                         num_final_pwm_outputs
                         , NULL, 0
                         , NULL, 0
                         , &simulink_w_integrator_DW.HILInitialize_AOVoltages[0]
                         , &simulink_w_integrator_DW.HILInitialize_POValues[0]
                         , (t_boolean *) NULL
                         , NULL
                         );
      if (result == -QERR_HIL_WRITE_NOT_SUPPORTED) {
        t_error local_result;
        result = 0;

        /* The hil_write operation is not supported by this card. Write final outputs for each channel type */
        if (num_final_analog_outputs > 0) {
          local_result = hil_write_analog
            (simulink_w_integrator_DW.HILInitialize_Card,
             simulink_w_integrator_P.HILInitialize_AOChannels,
             num_final_analog_outputs,
             &simulink_w_integrator_DW.HILInitialize_AOVoltages[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_pwm_outputs > 0) {
          local_result = hil_write_pwm
            (simulink_w_integrator_DW.HILInitialize_Card,
             simulink_w_integrator_P.HILInitialize_POChannels,
             num_final_pwm_outputs,
             &simulink_w_integrator_DW.HILInitialize_POValues[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(simulink_w_integrator_M, _rt_error_message);
        }
      }
    }

    hil_task_delete_all(simulink_w_integrator_DW.HILInitialize_Card);
    hil_monitor_delete_all(simulink_w_integrator_DW.HILInitialize_Card);
    hil_close(simulink_w_integrator_DW.HILInitialize_Card);
    simulink_w_integrator_DW.HILInitialize_Card = NULL;
  }

  /* Terminate for S-Function (stream_call_block): '<S11>/Stream Call1' incorporates:
   *  Constant: '<S11>/Constant'
   *  S-Function (string_constant_block): '<S11>/String Constant'
   */

  /* S-Function Block: simulink_w_integrator/IMU system/IMU/Stream Call1 (stream_call_block) */
  {
    if (simulink_w_integrator_DW.StreamCall1_Stream != NULL) {
      stream_close(simulink_w_integrator_DW.StreamCall1_Stream);
      simulink_w_integrator_DW.StreamCall1_Stream = NULL;
    }
  }

  /* Terminate for S-Function (game_controller_block): '<S6>/Game Controller' */

  /* S-Function Block: simulink_w_integrator/Joystick/Game Controller (game_controller_block) */
  {
    if (simulink_w_integrator_P.GameController_Enabled) {
      game_controller_close(simulink_w_integrator_DW.GameController_Controller);
      simulink_w_integrator_DW.GameController_Controller = NULL;
    }
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/

/* Solver interface called by GRT_Main */
#ifndef USE_GENERATED_SOLVER

void rt_ODECreateIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEDestroyIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEUpdateContinuousStates(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

#endif

void MdlOutputs(int_T tid)
{
  if (tid == 1)
    tid = 0;
  simulink_w_integrator_output(tid);
}

void MdlUpdate(int_T tid)
{
  if (tid == 1)
    tid = 0;
  simulink_w_integrator_update(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  simulink_w_integrator_initialize();
}

void MdlTerminate(void)
{
  simulink_w_integrator_terminate();
}

/* Registration function */
RT_MODEL_simulink_w_integrato_T *simulink_w_integrator(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  simulink_w_integrator_P.Integrator_UpperSat = rtInf;
  simulink_w_integrator_P.Integrator_LowerSat = rtMinusInf;

  /* initialize real-time model */
  (void) memset((void *)simulink_w_integrator_M, 0,
                sizeof(RT_MODEL_simulink_w_integrato_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&simulink_w_integrator_M->solverInfo,
                          &simulink_w_integrator_M->Timing.simTimeStep);
    rtsiSetTPtr(&simulink_w_integrator_M->solverInfo, &rtmGetTPtr
                (simulink_w_integrator_M));
    rtsiSetStepSizePtr(&simulink_w_integrator_M->solverInfo,
                       &simulink_w_integrator_M->Timing.stepSize0);
    rtsiSetdXPtr(&simulink_w_integrator_M->solverInfo,
                 &simulink_w_integrator_M->derivs);
    rtsiSetContStatesPtr(&simulink_w_integrator_M->solverInfo, (real_T **)
                         &simulink_w_integrator_M->contStates);
    rtsiSetNumContStatesPtr(&simulink_w_integrator_M->solverInfo,
      &simulink_w_integrator_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&simulink_w_integrator_M->solverInfo,
      &simulink_w_integrator_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&simulink_w_integrator_M->solverInfo,
      &simulink_w_integrator_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&simulink_w_integrator_M->solverInfo,
      &simulink_w_integrator_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&simulink_w_integrator_M->solverInfo,
                          (&rtmGetErrorStatus(simulink_w_integrator_M)));
    rtsiSetRTModelPtr(&simulink_w_integrator_M->solverInfo,
                      simulink_w_integrator_M);
  }

  rtsiSetSimTimeStep(&simulink_w_integrator_M->solverInfo, MAJOR_TIME_STEP);
  simulink_w_integrator_M->intgData.f[0] = simulink_w_integrator_M->odeF[0];
  simulink_w_integrator_M->contStates = ((real_T *) &simulink_w_integrator_X);
  rtsiSetSolverData(&simulink_w_integrator_M->solverInfo, (void *)
                    &simulink_w_integrator_M->intgData);
  rtsiSetSolverName(&simulink_w_integrator_M->solverInfo,"ode1");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = simulink_w_integrator_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    mdlTsMap[2] = 2;
    simulink_w_integrator_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    simulink_w_integrator_M->Timing.sampleTimes =
      (&simulink_w_integrator_M->Timing.sampleTimesArray[0]);
    simulink_w_integrator_M->Timing.offsetTimes =
      (&simulink_w_integrator_M->Timing.offsetTimesArray[0]);

    /* task periods */
    simulink_w_integrator_M->Timing.sampleTimes[0] = (0.0);
    simulink_w_integrator_M->Timing.sampleTimes[1] = (0.002);
    simulink_w_integrator_M->Timing.sampleTimes[2] = (0.01);

    /* task offsets */
    simulink_w_integrator_M->Timing.offsetTimes[0] = (0.0);
    simulink_w_integrator_M->Timing.offsetTimes[1] = (0.0);
    simulink_w_integrator_M->Timing.offsetTimes[2] = (0.0);
  }

  rtmSetTPtr(simulink_w_integrator_M, &simulink_w_integrator_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = simulink_w_integrator_M->Timing.sampleHitArray;
    int_T *mdlPerTaskSampleHits =
      simulink_w_integrator_M->Timing.perTaskSampleHitsArray;
    simulink_w_integrator_M->Timing.perTaskSampleHits = (&mdlPerTaskSampleHits[0]);
    mdlSampleHits[0] = 1;
    simulink_w_integrator_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(simulink_w_integrator_M, -1);
  simulink_w_integrator_M->Timing.stepSize0 = 0.002;
  simulink_w_integrator_M->Timing.stepSize1 = 0.002;
  simulink_w_integrator_M->Timing.stepSize2 = 0.01;
  simulink_w_integrator_M->solverInfoPtr = (&simulink_w_integrator_M->solverInfo);
  simulink_w_integrator_M->Timing.stepSize = (0.002);
  rtsiSetFixedStepSize(&simulink_w_integrator_M->solverInfo, 0.002);
  rtsiSetSolverMode(&simulink_w_integrator_M->solverInfo,
                    SOLVER_MODE_MULTITASKING);

  /* block I/O */
  simulink_w_integrator_M->blockIO = ((void *) &simulink_w_integrator_B);

  {
    int32_T i;
    for (i = 0; i < 10; i++) {
      simulink_w_integrator_B.Switch[i] = 0.0;
    }

    for (i = 0; i < 5; i++) {
      simulink_w_integrator_B.Sum_p[i] = 0.0;
    }

    simulink_w_integrator_B.RateTransitionx = 0.0;
    simulink_w_integrator_B.Joystick_gain_x = 0.0;
    simulink_w_integrator_B.RateTransitiony = 0.0;
    simulink_w_integrator_B.Joystick_gain_y = 0.0;
    simulink_w_integrator_B.Sum = 0.0;
    simulink_w_integrator_B.Sum1 = 0.0;
    simulink_w_integrator_B.ElevationCounttorad = 0.0;
    simulink_w_integrator_B.PitchCounttorad = 0.0;
    simulink_w_integrator_B.TravelCounttorad = 0.0;
    simulink_w_integrator_B.FrontmotorSaturation = 0.0;
    simulink_w_integrator_B.BackmotorSaturation = 0.0;
    simulink_w_integrator_B.Sum1_o = 0.0;
    simulink_w_integrator_B.Sum2 = 0.0;
    simulink_w_integrator_B.K_ei = 0.0;
    simulink_w_integrator_B.euler_rates[0] = 0.0;
    simulink_w_integrator_B.euler_rates[1] = 0.0;
    simulink_w_integrator_B.euler_rates[2] = 0.0;
  }

  /* parameters */
  simulink_w_integrator_M->defaultParam = ((real_T *)&simulink_w_integrator_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &simulink_w_integrator_X;
    simulink_w_integrator_M->contStates = (x);
    (void) memset((void *)&simulink_w_integrator_X, 0,
                  sizeof(X_simulink_w_integrator_T));
  }

  /* states (dwork) */
  simulink_w_integrator_M->dwork = ((void *) &simulink_w_integrator_DW);
  (void) memset((void *)&simulink_w_integrator_DW, 0,
                sizeof(DW_simulink_w_integrator_T));

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_w_integrator_DW.HILInitialize_AIMinimums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_w_integrator_DW.HILInitialize_AIMaximums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_w_integrator_DW.HILInitialize_AOMinimums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_w_integrator_DW.HILInitialize_AOMaximums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_w_integrator_DW.HILInitialize_AOVoltages[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_w_integrator_DW.HILInitialize_FilterFrequency[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_w_integrator_DW.HILInitialize_POSortedFreqs[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_w_integrator_DW.HILInitialize_POValues[i] = 0.0;
    }
  }

  simulink_w_integrator_DW.RateTransitionx_Buffer0 = 0.0;
  simulink_w_integrator_DW.RateTransitiony_Buffer0 = 0.0;

  {
    int32_T i;
    for (i = 0; i < 10; i++) {
      simulink_w_integrator_DW.Memory_PreviousInput[i] = 0.0;
    }
  }

  simulink_w_integrator_DW.HILWriteAnalog_Buffer[0] = 0.0;
  simulink_w_integrator_DW.HILWriteAnalog_Buffer[1] = 0.0;

  /* Initialize Sizes */
  simulink_w_integrator_M->Sizes.numContStates = (11);/* Number of continuous states */
  simulink_w_integrator_M->Sizes.numPeriodicContStates = (0);
                                      /* Number of periodic continuous states */
  simulink_w_integrator_M->Sizes.numY = (0);/* Number of model outputs */
  simulink_w_integrator_M->Sizes.numU = (0);/* Number of model inputs */
  simulink_w_integrator_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  simulink_w_integrator_M->Sizes.numSampTimes = (3);/* Number of sample times */
  simulink_w_integrator_M->Sizes.numBlocks = (69);/* Number of blocks */
  simulink_w_integrator_M->Sizes.numBlockIO = (17);/* Number of block outputs */
  simulink_w_integrator_M->Sizes.numBlockPrms = (537);/* Sum of parameter "widths" */
  return simulink_w_integrator_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
