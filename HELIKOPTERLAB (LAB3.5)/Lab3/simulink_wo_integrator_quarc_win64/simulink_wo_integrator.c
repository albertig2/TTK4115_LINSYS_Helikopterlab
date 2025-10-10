/*
 * simulink_wo_integrator.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "simulink_wo_integrator".
 *
 * Model version              : 11.32
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C source code generated on : Mon Oct  6 16:40:58 2025
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "simulink_wo_integrator.h"
#include "simulink_wo_integrator_private.h"
#include "simulink_wo_integrator_dt.h"

t_stream simulink_wo_integrator_rtZt_stream = NULL;

/* Block signals (default storage) */
B_simulink_wo_integrator_T simulink_wo_integrator_B;

/* Continuous states */
X_simulink_wo_integrator_T simulink_wo_integrator_X;

/* Block states (default storage) */
DW_simulink_wo_integrator_T simulink_wo_integrator_DW;

/* Real-time model */
static RT_MODEL_simulink_wo_integrat_T simulink_wo_integrator_M_;
RT_MODEL_simulink_wo_integrat_T *const simulink_wo_integrator_M =
  &simulink_wo_integrator_M_;
static void rate_monotonic_scheduler(void);

/*
 * Writes out MAT-file header.  Returns success or failure.
 * Returns:
 *      0 - success
 *      1 - failure
 */
int_T rt_WriteMat4FileHeader(FILE *fp, int32_T m, int32_T n, const char *name)
{
  typedef enum { ELITTLE_ENDIAN, EBIG_ENDIAN } ByteOrder;

  int16_T one = 1;
  ByteOrder byteOrder = (*((int8_T *)&one)==1) ? ELITTLE_ENDIAN : EBIG_ENDIAN;
  int32_T type = (byteOrder == ELITTLE_ENDIAN) ? 0: 1000;
  int32_T imagf = 0;
  int32_T name_len = (int32_T)strlen(name) + 1;
  if ((fwrite(&type, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&m, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&n, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&imagf, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&name_len, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(name, sizeof(char), name_len, fp) == 0)) {
    return(1);
  } else {
    return(0);
  }
}                                      /* end rt_WriteMat4FileHeader */

time_T rt_SimUpdateDiscreteEvents(
  int_T rtmNumSampTimes, void *rtmTimingData, int_T *rtmSampleHitPtr, int_T
  *rtmPerTaskSampleHits )
{
  rtmSampleHitPtr[1] = rtmStepTask(simulink_wo_integrator_M, 1);
  rtmSampleHitPtr[2] = rtmStepTask(simulink_wo_integrator_M, 2);
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
  if (simulink_wo_integrator_M->Timing.TaskCounters.TID[1] == 0) {
    simulink_wo_integrator_M->Timing.RateInteraction.TID1_2 =
      (simulink_wo_integrator_M->Timing.TaskCounters.TID[2] == 0);

    /* update PerTaskSampleHits matrix for non-inline sfcn */
    simulink_wo_integrator_M->Timing.perTaskSampleHits[5] =
      simulink_wo_integrator_M->Timing.RateInteraction.TID1_2;
  }

  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (simulink_wo_integrator_M->Timing.TaskCounters.TID[2])++;
  if ((simulink_wo_integrator_M->Timing.TaskCounters.TID[2]) > 4) {/* Sample time: [0.01s, 0.0s] */
    simulink_wo_integrator_M->Timing.TaskCounters.TID[2] = 0;
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
  int_T nXc = 4;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  simulink_wo_integrator_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * Output and update for atomic system:
 *    '<Root>/Gyro vector to [pitch rate, elevation rate, travle rate]'
 *    '<S5>/Gyro vector to [pitch rate, elevation rate, travle rate]'
 */
void Gyrovectortopitchrateelevat(const real_T rtu_gyro_vec[3], real_T
  rtu_euler_angles, real_T rtu_euler_angles_o, B_Gyrovectortopitchrateelevat_T
  *localB)
{
  real_T tmp[9];
  real_T euler_rates_tmp;
  real_T euler_rates_tmp_0;
  real_T euler_rates_tmp_1;
  real_T euler_rates_tmp_2;
  int32_T i;

  /* SignalConversion generated from: '<S3>/ SFunction ' */
  /* MATLAB Function 'Gyro vector to [pitch rate, elevation rate, travle rate]': '<S3>:1' */
  /* '<S3>:1:3' */
  /* '<S3>:1:4' */
  /* '<S3>:1:8' */
  /* '<S3>:1:11' */
  euler_rates_tmp = tan(rtu_euler_angles_o);
  euler_rates_tmp_0 = sin(rtu_euler_angles);
  euler_rates_tmp_1 = cos(rtu_euler_angles);
  euler_rates_tmp_2 = cos(rtu_euler_angles_o);
  tmp[0] = 1.0;
  tmp[3] = euler_rates_tmp_0 * euler_rates_tmp;
  tmp[6] = euler_rates_tmp_1 * euler_rates_tmp;
  tmp[1] = 0.0;
  tmp[4] = euler_rates_tmp_1;
  tmp[7] = -euler_rates_tmp_0;
  tmp[2] = 0.0;
  tmp[5] = euler_rates_tmp_0 / euler_rates_tmp_2;
  tmp[8] = euler_rates_tmp_1 / euler_rates_tmp_2;
  for (i = 0; i < 3; i++) {
    localB->euler_rates[i] = 0.0;
    localB->euler_rates[i] += tmp[i] * rtu_gyro_vec[0];
    localB->euler_rates[i] += tmp[i + 3] * rtu_gyro_vec[1];
    localB->euler_rates[i] += tmp[i + 6] * rtu_gyro_vec[2];
  }
}

/* Model output function for TID0 */
void simulink_wo_integrator_output0(void) /* Sample time: [0.0s, 0.0s] */
{
  /* local block i/o variables */
  t_stream_ptr rtb_StreamCall1_o1;
  real_T rtb_HILReadEncoderTimebase_o1;
  real_T rtb_HILReadEncoderTimebase_o2;
  real_T rtb_TmpSignalConversionAtToFile[6];
  real_T rtb_BackmotorSaturation;
  real_T rtb_Sum;
  real32_T rtb_StreamRead1_o2[10];
  int32_T rtb_StreamFormattedWrite_o2;
  int32_T rtb_StreamCall1_o3;
  boolean_T rtb_StreamRead1_o3;
  real_T ay;
  real_T az;
  int32_T i;
  if (rtmIsMajorTimeStep(simulink_wo_integrator_M)) {
    /* set solver stop time */
    if (!(simulink_wo_integrator_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&simulink_wo_integrator_M->solverInfo,
                            ((simulink_wo_integrator_M->Timing.clockTickH0 + 1) *
        simulink_wo_integrator_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&simulink_wo_integrator_M->solverInfo,
                            ((simulink_wo_integrator_M->Timing.clockTick0 + 1) *
        simulink_wo_integrator_M->Timing.stepSize0 +
        simulink_wo_integrator_M->Timing.clockTickH0 *
        simulink_wo_integrator_M->Timing.stepSize0 * 4294967296.0));
    }

    {                                  /* Sample time: [0.0s, 0.0s] */
      rate_monotonic_scheduler();
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(simulink_wo_integrator_M)) {
    simulink_wo_integrator_M->Timing.t[0] = rtsiGetT
      (&simulink_wo_integrator_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(simulink_wo_integrator_M)) {
    /* S-Function (hil_read_encoder_timebase_block): '<S4>/HIL Read Encoder Timebase' */

    /* S-Function Block: simulink_wo_integrator/Heli 3D/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
    {
      t_error result;
      result = hil_task_read_encoder
        (simulink_wo_integrator_DW.HILReadEncoderTimebase_Task, 1,
         &simulink_wo_integrator_DW.HILReadEncoderTimebase_Buffer[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
      } else {
        rtb_HILReadEncoderTimebase_o1 =
          simulink_wo_integrator_DW.HILReadEncoderTimebase_Buffer[0];
        rtb_HILReadEncoderTimebase_o2 =
          simulink_wo_integrator_DW.HILReadEncoderTimebase_Buffer[1];
        rtb_BackmotorSaturation =
          simulink_wo_integrator_DW.HILReadEncoderTimebase_Buffer[2];
      }
    }

    /* S-Function (stream_call_block): '<S11>/Stream Call1' incorporates:
     *  Constant: '<S11>/Constant'
     *  S-Function (string_constant_block): '<S11>/String Constant'
     */

    /* S-Function Block: simulink_wo_integrator/IMU system/IMU/Stream Call1 (stream_call_block) */
    {
      t_error result = 0;
      t_boolean close_flag = (simulink_wo_integrator_P.Constant_Value != 0);
      rtb_StreamCall1_o1 = NULL;
      switch (simulink_wo_integrator_DW.StreamCall1_State) {
       case STREAM_CALL_STATE_NOT_CONNECTED:
        {
          if (!close_flag) {
            /* Make sure URI is null-terminated */
            if (string_length((char *)
                              simulink_wo_integrator_P.StringConstant_Value, 255)
                == 255) {
              rtmSetErrorStatus(simulink_wo_integrator_M,
                                "URI passed to Stream Call block is not null-terminated!");
              result = -QERR_STRING_NOT_TERMINATED;
            } else {
              result = stream_connect((char *)
                simulink_wo_integrator_P.StringConstant_Value,
                simulink_wo_integrator_P.StreamCall1_NonBlocking != 0,
                simulink_wo_integrator_P.StreamCall1_SendBufferSize,
                simulink_wo_integrator_P.StreamCall1_ReceiveBufferSize,
                &simulink_wo_integrator_DW.StreamCall1_Stream);
              if (result == 0) {
                simulink_wo_integrator_DW.StreamCall1_State =
                  STREAM_CALL_STATE_CONNECTED;
                stream_set_byte_order
                  (simulink_wo_integrator_DW.StreamCall1_Stream,
                   (t_stream_byte_order)
                   simulink_wo_integrator_P.StreamCall1_Endian);
                rtb_StreamCall1_o1 =
                  &simulink_wo_integrator_DW.StreamCall1_Stream;
              } else if (result == -QERR_WOULD_BLOCK) {
                simulink_wo_integrator_DW.StreamCall1_State =
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

            result = stream_poll(simulink_wo_integrator_DW.StreamCall1_Stream,
                                 &timeout, STREAM_POLL_CONNECT);
            if (result > 0) {
              simulink_wo_integrator_DW.StreamCall1_State =
                STREAM_CALL_STATE_CONNECTED;
              stream_set_byte_order(simulink_wo_integrator_DW.StreamCall1_Stream,
                                    (t_stream_byte_order)
                                    simulink_wo_integrator_P.StreamCall1_Endian);
              rtb_StreamCall1_o1 = &simulink_wo_integrator_DW.StreamCall1_Stream;
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
          rtb_StreamCall1_o1 = &simulink_wo_integrator_DW.StreamCall1_Stream;
          if (!close_flag) {
            break;
          }

          /* Fall through deliberately */
        }

       default:
        {
          t_error close_result = stream_close
            (simulink_wo_integrator_DW.StreamCall1_Stream);
          if (close_result == 0) {
            simulink_wo_integrator_DW.StreamCall1_State =
              STREAM_CALL_STATE_NOT_CONNECTED;
            simulink_wo_integrator_DW.StreamCall1_Stream = NULL;
            rtb_StreamCall1_o1 = NULL;
          } else if (result == 0) {
            result = close_result;
          }
          break;
        }
      }

      simulink_wo_integrator_B.StreamCall1_o2 =
        simulink_wo_integrator_DW.StreamCall1_State;
      rtb_StreamCall1_o3 = (int32_T) result;
    }

    /* S-Function (stream_formatted_write_block): '<S11>/Stream Formatted Write' incorporates:
     *  Constant: '<S11>/Constant1'
     */
    {
      t_error result;
      if (rtb_StreamCall1_o1 != NULL) {
        result = stream_print_utf8_char_array(*rtb_StreamCall1_o1,
          simulink_wo_integrator_P.StreamFormattedWrite_MaxUnits,
          &rtb_StreamFormattedWrite_o2, "%c\n"
          , (char) simulink_wo_integrator_P.Constant1_Value
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

    /* S-Function Block: simulink_wo_integrator/IMU system/IMU/Stream Read1 (stream_read_block) */
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
        simulink_wo_integrator_B.Switch[i] = rtb_StreamRead1_o2[i];
      } else {
        /* Switch: '<S11>/Switch' incorporates:
         *  Memory: '<S11>/Memory'
         */
        simulink_wo_integrator_B.Switch[i] =
          simulink_wo_integrator_DW.Memory_PreviousInput[i];
      }

      /* End of Switch: '<S11>/Switch' */
    }

    for (i = 0; i < 3; i++) {
      /* Gain: '<S11>/Gain2' */
      simulink_wo_integrator_B.Gain2[i] = 0.0;
      simulink_wo_integrator_B.Gain2[i] += simulink_wo_integrator_P.Gain2_Gain[i]
        * simulink_wo_integrator_B.Switch[0];
      simulink_wo_integrator_B.Gain2[i] += simulink_wo_integrator_P.Gain2_Gain[i
        + 3] * simulink_wo_integrator_B.Switch[1];
      simulink_wo_integrator_B.Gain2[i] += simulink_wo_integrator_P.Gain2_Gain[i
        + 6] * simulink_wo_integrator_B.Switch[2];
    }

    /* RateTransition: '<S6>/Rate Transition: x' */
    if (simulink_wo_integrator_M->Timing.RateInteraction.TID1_2) {
      /* RateTransition: '<S6>/Rate Transition: x' */
      simulink_wo_integrator_B.RateTransitionx =
        simulink_wo_integrator_DW.RateTransitionx_Buffer0;
    }

    /* End of RateTransition: '<S6>/Rate Transition: x' */

    /* DeadZone: '<S6>/Dead Zone: x' */
    if (simulink_wo_integrator_B.RateTransitionx >
        simulink_wo_integrator_P.DeadZonex_End) {
      /* Sum: '<S2>/Sum' */
      rtb_Sum = simulink_wo_integrator_B.RateTransitionx -
        simulink_wo_integrator_P.DeadZonex_End;
    } else if (simulink_wo_integrator_B.RateTransitionx >=
               simulink_wo_integrator_P.DeadZonex_Start) {
      /* Sum: '<S2>/Sum' */
      rtb_Sum = 0.0;
    } else {
      /* Sum: '<S2>/Sum' */
      rtb_Sum = simulink_wo_integrator_B.RateTransitionx -
        simulink_wo_integrator_P.DeadZonex_Start;
    }

    /* End of DeadZone: '<S6>/Dead Zone: x' */

    /* Gain: '<S6>/Joystick_gain_x' incorporates:
     *  Gain: '<S6>/Gain: x'
     */
    simulink_wo_integrator_B.Joystick_gain_x =
      simulink_wo_integrator_P.Gainx_Gain * rtb_Sum *
      simulink_wo_integrator_P.Joystick_gain_x;

    /* RateTransition: '<S6>/Rate Transition: y' */
    if (simulink_wo_integrator_M->Timing.RateInteraction.TID1_2) {
      /* RateTransition: '<S6>/Rate Transition: y' */
      simulink_wo_integrator_B.RateTransitiony =
        simulink_wo_integrator_DW.RateTransitiony_Buffer0;
    }

    /* End of RateTransition: '<S6>/Rate Transition: y' */

    /* DeadZone: '<S6>/Dead Zone: y' */
    if (simulink_wo_integrator_B.RateTransitiony >
        simulink_wo_integrator_P.DeadZoney_End) {
      /* Sum: '<S2>/Sum' */
      rtb_Sum = simulink_wo_integrator_B.RateTransitiony -
        simulink_wo_integrator_P.DeadZoney_End;
    } else if (simulink_wo_integrator_B.RateTransitiony >=
               simulink_wo_integrator_P.DeadZoney_Start) {
      /* Sum: '<S2>/Sum' */
      rtb_Sum = 0.0;
    } else {
      /* Sum: '<S2>/Sum' */
      rtb_Sum = simulink_wo_integrator_B.RateTransitiony -
        simulink_wo_integrator_P.DeadZoney_Start;
    }

    /* End of DeadZone: '<S6>/Dead Zone: y' */

    /* Gain: '<S6>/Joystick_gain_y' incorporates:
     *  Gain: '<S6>/Gain: y'
     */
    simulink_wo_integrator_B.Joystick_gain_y =
      simulink_wo_integrator_P.Gainy_Gain * rtb_Sum *
      simulink_wo_integrator_P.Joystick_gain_y;

    /* Gain: '<S7>/Gain' incorporates:
     *  SignalConversion generated from: '<S7>/Gain'
     */
    simulink_wo_integrator_B.Gain[0] = 0.0;
    simulink_wo_integrator_B.Gain[0] += simulink_wo_integrator_P.F[0] *
      simulink_wo_integrator_B.Joystick_gain_x;
    simulink_wo_integrator_B.Gain[0] += simulink_wo_integrator_P.F[2] *
      simulink_wo_integrator_B.Joystick_gain_y;
    simulink_wo_integrator_B.Gain[1] = 0.0;
    simulink_wo_integrator_B.Gain[1] += simulink_wo_integrator_P.F[1] *
      simulink_wo_integrator_B.Joystick_gain_x;
    simulink_wo_integrator_B.Gain[1] += simulink_wo_integrator_P.F[3] *
      simulink_wo_integrator_B.Joystick_gain_y;

    /* Gain: '<S4>/Pitch: Count to rad' */
    simulink_wo_integrator_B.PitchCounttorad =
      simulink_wo_integrator_P.PitchCounttorad_Gain *
      rtb_HILReadEncoderTimebase_o2;

    /* Gain: '<S4>/Elevation: Count to rad' */
    simulink_wo_integrator_B.ElevationCounttorad =
      simulink_wo_integrator_P.ElevationCounttorad_Gain *
      rtb_BackmotorSaturation;
  }

  /* TransferFcn: '<S4>/Pitch: Transfer Fcn' */
  simulink_wo_integrator_B.PitchTransferFcn = 0.0;
  simulink_wo_integrator_B.PitchTransferFcn +=
    simulink_wo_integrator_P.PitchTransferFcn_C *
    simulink_wo_integrator_X.PitchTransferFcn_CSTATE;
  simulink_wo_integrator_B.PitchTransferFcn +=
    simulink_wo_integrator_P.PitchTransferFcn_D *
    simulink_wo_integrator_B.PitchCounttorad;

  /* TransferFcn: '<S4>/Elevation: Transfer Fcn' */
  simulink_wo_integrator_B.ElevationTransferFcn = 0.0;
  simulink_wo_integrator_B.ElevationTransferFcn +=
    simulink_wo_integrator_P.ElevationTransferFcn_C *
    simulink_wo_integrator_X.ElevationTransferFcn_CSTATE;
  simulink_wo_integrator_B.ElevationTransferFcn +=
    simulink_wo_integrator_P.ElevationTransferFcn_D *
    simulink_wo_integrator_B.ElevationCounttorad;
  for (i = 0; i < 2; i++) {
    /* Sum: '<S7>/Sum' incorporates:
     *  Gain: '<S7>/Gain3'
     *  SignalConversion generated from: '<S7>/Gain3'
     */
    simulink_wo_integrator_B.Sum[i] = simulink_wo_integrator_B.Gain[i] -
      ((simulink_wo_integrator_P.K[i + 2] *
        simulink_wo_integrator_B.PitchTransferFcn + simulink_wo_integrator_P.K[i]
        * simulink_wo_integrator_B.PitchCounttorad) +
       simulink_wo_integrator_P.K[i + 4] *
       simulink_wo_integrator_B.ElevationTransferFcn);
  }

  if (rtmIsMajorTimeStep(simulink_wo_integrator_M)) {
    /* Gain: '<S4>/Travel: Count to rad' */
    simulink_wo_integrator_B.TravelCounttorad =
      simulink_wo_integrator_P.TravelCounttorad_Gain *
      rtb_HILReadEncoderTimebase_o1;

    /* Sum: '<S4>/Sum' incorporates:
     *  Constant: '<S4>/Constant'
     */
    simulink_wo_integrator_B.Sum_m =
      simulink_wo_integrator_B.ElevationCounttorad -
      simulink_wo_integrator_P.Constant_Value_p;
  }

  /* TransferFcn: '<S4>/Travel: Transfer Fcn' */
  simulink_wo_integrator_B.TravelTransferFcn = 0.0;
  simulink_wo_integrator_B.TravelTransferFcn +=
    simulink_wo_integrator_P.TravelTransferFcn_C *
    simulink_wo_integrator_X.TravelTransferFcn_CSTATE;
  simulink_wo_integrator_B.TravelTransferFcn +=
    simulink_wo_integrator_P.TravelTransferFcn_D *
    simulink_wo_integrator_B.TravelCounttorad;

  /* Sum: '<Root>/Sum' incorporates:
   *  Constant: '<Root>/Constant'
   */
  simulink_wo_integrator_B.Sum_o = simulink_wo_integrator_B.Sum[0] +
    simulink_wo_integrator_P.Vs_0;
  if (rtmIsMajorTimeStep(simulink_wo_integrator_M)) {
    for (i = 0; i < 3; i++) {
      /* Gain: '<S11>/Gain1' */
      simulink_wo_integrator_B.Gain1[i] = 0.0;
      simulink_wo_integrator_B.Gain1[i] += simulink_wo_integrator_P.Gain1_Gain[i]
        * simulink_wo_integrator_B.Switch[3];
      simulink_wo_integrator_B.Gain1[i] += simulink_wo_integrator_P.Gain1_Gain[i
        + 3] * simulink_wo_integrator_B.Switch[4];
      simulink_wo_integrator_B.Gain1[i] += simulink_wo_integrator_P.Gain1_Gain[i
        + 6] * simulink_wo_integrator_B.Switch[5];
    }

    /* MATLAB Function: '<Root>/Gyro vector to [pitch rate, elevation rate, travle rate]' */
    Gyrovectortopitchrateelevat(simulink_wo_integrator_B.Gain1,
      simulink_wo_integrator_B.PitchCounttorad, simulink_wo_integrator_B.Sum_m,
      &simulink_wo_integrator_B.sf_Gyrovectortopitchrateelevati);

    /* MATLAB Function: '<S5>/MATLAB Function' */
    ay = simulink_wo_integrator_B.Gain2[1];
    az = simulink_wo_integrator_B.Gain2[2];

    /* MATLAB Function 'IMU system/MATLAB Function': '<S12>:1' */
    if (simulink_wo_integrator_B.Gain2[2] == 0.0) {
      /* '<S12>:1:2' */
      /* '<S12>:1:3' */
      az = 0.01;
      if (simulink_wo_integrator_B.Gain2[1] == 0.0) {
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
    simulink_wo_integrator_B.Sum_c = atan(ay / az) +
      simulink_wo_integrator_P.Constant_Value_b;

    /* Sum: '<S5>/Sum1' incorporates:
     *  Constant: '<S5>/Constant1'
     *  MATLAB Function: '<S5>/MATLAB Function'
     */
    simulink_wo_integrator_B.Sum1 = atan(simulink_wo_integrator_B.Gain2[0] /
      sqrt(ay * ay + az * az)) + simulink_wo_integrator_P.Constant1_Value_e;

    /* MATLAB Function: '<S5>/Gyro vector to [pitch rate, elevation rate, travle rate]' */
    Gyrovectortopitchrateelevat(simulink_wo_integrator_B.Gain1,
      simulink_wo_integrator_B.Sum_c, simulink_wo_integrator_B.Sum1,
      &simulink_wo_integrator_B.sf_Gyrovectortopitchrateeleva_h);

    /* SignalConversion generated from: '<Root>/To File' */
    rtb_TmpSignalConversionAtToFile[0] =
      simulink_wo_integrator_B.sf_Gyrovectortopitchrateelevati.euler_rates[0];
    rtb_TmpSignalConversionAtToFile[1] =
      simulink_wo_integrator_B.sf_Gyrovectortopitchrateelevati.euler_rates[1];
    rtb_TmpSignalConversionAtToFile[2] =
      simulink_wo_integrator_B.sf_Gyrovectortopitchrateelevati.euler_rates[2];
    rtb_TmpSignalConversionAtToFile[3] =
      simulink_wo_integrator_B.sf_Gyrovectortopitchrateeleva_h.euler_rates[0];
    rtb_TmpSignalConversionAtToFile[4] =
      simulink_wo_integrator_B.sf_Gyrovectortopitchrateeleva_h.euler_rates[1];
    rtb_TmpSignalConversionAtToFile[5] =
      simulink_wo_integrator_B.sf_Gyrovectortopitchrateeleva_h.euler_rates[2];

    /* ToFile: '<Root>/To File' */
    if (rtmIsMajorTimeStep(simulink_wo_integrator_M)) {
      if (rtmIsMajorTimeStep(simulink_wo_integrator_M) ) {
        {
          if (!(++simulink_wo_integrator_DW.ToFile_IWORK.Decimation % 1) &&
              (simulink_wo_integrator_DW.ToFile_IWORK.Count * (6 + 1)) + 1 <
              100000000 ) {
            FILE *fp = (FILE *) simulink_wo_integrator_DW.ToFile_PWORK.FilePtr;
            if (fp != (NULL)) {
              real_T u[6 + 1];
              simulink_wo_integrator_DW.ToFile_IWORK.Decimation = 0;
              u[0] = simulink_wo_integrator_M->Timing.t[1];
              u[1] = rtb_TmpSignalConversionAtToFile[0];
              u[2] = rtb_TmpSignalConversionAtToFile[1];
              u[3] = rtb_TmpSignalConversionAtToFile[2];
              u[4] = rtb_TmpSignalConversionAtToFile[3];
              u[5] = rtb_TmpSignalConversionAtToFile[4];
              u[6] = rtb_TmpSignalConversionAtToFile[5];
              if (fwrite(u, sizeof(real_T), 6 + 1, fp) != 6 + 1) {
                rtmSetErrorStatus(simulink_wo_integrator_M,
                                  "Error writing to MAT-file enc_rates_vs_imu_rates_func_block_w_offset_added.mat");
                return;
              }

              if (((++simulink_wo_integrator_DW.ToFile_IWORK.Count) * (6 + 1))+1
                  >= 100000000) {
                (void)fprintf(stdout,
                              "*** The ToFile block will stop logging data before\n"
                              "    the simulation has ended, because it has reached\n"
                              "    the maximum number of elements (100000000)\n"
                              "    allowed in MAT-file enc_rates_vs_imu_rates_func_block_w_offset_added.mat.\n");
              }
            }
          }
        }
      }
    }

    /* Saturate: '<S4>/Front motor: Saturation' */
    if (0.0 > simulink_wo_integrator_P.FrontmotorSaturation_UpperSat) {
      /* Sum: '<S2>/Sum' */
      rtb_Sum = simulink_wo_integrator_P.FrontmotorSaturation_UpperSat;
    } else if (0.0 < simulink_wo_integrator_P.FrontmotorSaturation_LowerSat) {
      /* Sum: '<S2>/Sum' */
      rtb_Sum = simulink_wo_integrator_P.FrontmotorSaturation_LowerSat;
    } else {
      /* Sum: '<S2>/Sum' */
      rtb_Sum = 0.0;
    }

    /* End of Saturate: '<S4>/Front motor: Saturation' */

    /* Saturate: '<S4>/Back motor: Saturation' */
    if (0.0 > simulink_wo_integrator_P.BackmotorSaturation_UpperSat) {
      /* Saturate: '<S4>/Back motor: Saturation' */
      rtb_BackmotorSaturation =
        simulink_wo_integrator_P.BackmotorSaturation_UpperSat;
    } else if (0.0 < simulink_wo_integrator_P.BackmotorSaturation_LowerSat) {
      /* Saturate: '<S4>/Back motor: Saturation' */
      rtb_BackmotorSaturation =
        simulink_wo_integrator_P.BackmotorSaturation_LowerSat;
    } else {
      /* Saturate: '<S4>/Back motor: Saturation' */
      rtb_BackmotorSaturation = 0.0;
    }

    /* End of Saturate: '<S4>/Back motor: Saturation' */

    /* S-Function (hil_write_analog_block): '<S4>/HIL Write Analog' */

    /* S-Function Block: simulink_wo_integrator/Heli 3D/HIL Write Analog (hil_write_analog_block) */
    {
      t_error result;
      simulink_wo_integrator_DW.HILWriteAnalog_Buffer[0] = rtb_Sum;
      simulink_wo_integrator_DW.HILWriteAnalog_Buffer[1] =
        rtb_BackmotorSaturation;
      result = hil_write_analog(simulink_wo_integrator_DW.HILInitialize_Card,
        simulink_wo_integrator_P.HILWriteAnalog_channels, 2,
        &simulink_wo_integrator_DW.HILWriteAnalog_Buffer[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
      }
    }

    /* S-Function (stop_with_error_block): '<S11>/Stop with Call Error' */

    /* S-Function Block: simulink_wo_integrator/IMU system/IMU/Stop with Call Error (stop_with_error_block) */
    {
      if (rtb_StreamCall1_o3 < 0) {
        msg_get_error_messageA(NULL, rtb_StreamCall1_o3, _rt_error_message,
          sizeof(_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }
    }
  }

  /* Integrator: '<S9>/Integrator' */
  /* Limited  Integrator  */
  if (simulink_wo_integrator_X.Integrator_CSTATE >=
      simulink_wo_integrator_P.Integrator_UpperSat) {
    simulink_wo_integrator_X.Integrator_CSTATE =
      simulink_wo_integrator_P.Integrator_UpperSat;
  } else {
    if (simulink_wo_integrator_X.Integrator_CSTATE <=
        simulink_wo_integrator_P.Integrator_LowerSat) {
      simulink_wo_integrator_X.Integrator_CSTATE =
        simulink_wo_integrator_P.Integrator_LowerSat;
    }
  }

  /* End of Integrator: '<S9>/Integrator' */
  if (rtmIsMajorTimeStep(simulink_wo_integrator_M)) {
    /* Sum: '<S2>/Sum' */
    rtb_Sum = 0.0;

    /* Gain: '<S9>/K_ei' */
    simulink_wo_integrator_B.K_ei = simulink_wo_integrator_P.K_ei_Gain * rtb_Sum;

    /* ToFile: '<Root>/To File1' */
    if (rtmIsMajorTimeStep(simulink_wo_integrator_M)) {
      if (rtmIsMajorTimeStep(simulink_wo_integrator_M) ) {
        {
          if (!(++simulink_wo_integrator_DW.ToFile1_IWORK.Decimation % 1) &&
              (simulink_wo_integrator_DW.ToFile1_IWORK.Count * (1 + 1)) + 1 <
              100000000 ) {
            FILE *fp = (FILE *) simulink_wo_integrator_DW.ToFile1_PWORK.FilePtr;
            if (fp != (NULL)) {
              real_T u[1 + 1];
              simulink_wo_integrator_DW.ToFile1_IWORK.Decimation = 0;
              u[0] = simulink_wo_integrator_M->Timing.t[1];
              u[1] = 0.0;
              if (fwrite(u, sizeof(real_T), 1 + 1, fp) != 1 + 1) {
                rtmSetErrorStatus(simulink_wo_integrator_M,
                                  "Error writing to MAT-file lab2_Q_1_1_20_R_1_0.5.mat");
                return;
              }

              if (((++simulink_wo_integrator_DW.ToFile1_IWORK.Count) * (1 + 1))+
                  1 >= 100000000) {
                (void)fprintf(stdout,
                              "*** The ToFile block will stop logging data before\n"
                              "    the simulation has ended, because it has reached\n"
                              "    the maximum number of elements (100000000)\n"
                              "    allowed in MAT-file lab2_Q_1_1_20_R_1_0.5.mat.\n");
              }
            }
          }
        }
      }
    }
  }
}

/* Model update function for TID0 */
void simulink_wo_integrator_update0(void) /* Sample time: [0.0s, 0.0s] */
{
  if (rtmIsMajorTimeStep(simulink_wo_integrator_M)) {
    /* Update for Memory: '<S11>/Memory' */
    memcpy(&simulink_wo_integrator_DW.Memory_PreviousInput[0],
           &simulink_wo_integrator_B.Switch[0], 10U * sizeof(real_T));
  }

  if (rtmIsMajorTimeStep(simulink_wo_integrator_M)) {
    rt_ertODEUpdateContinuousStates(&simulink_wo_integrator_M->solverInfo);
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
  if (!(++simulink_wo_integrator_M->Timing.clockTick0)) {
    ++simulink_wo_integrator_M->Timing.clockTickH0;
  }

  simulink_wo_integrator_M->Timing.t[0] = rtsiGetSolverStopTime
    (&simulink_wo_integrator_M->solverInfo);

  /* Update absolute time */
  /* The "clockTick1" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick1"
   * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick1 and the high bits
   * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++simulink_wo_integrator_M->Timing.clockTick1)) {
    ++simulink_wo_integrator_M->Timing.clockTickH1;
  }

  simulink_wo_integrator_M->Timing.t[1] =
    simulink_wo_integrator_M->Timing.clockTick1 *
    simulink_wo_integrator_M->Timing.stepSize1 +
    simulink_wo_integrator_M->Timing.clockTickH1 *
    simulink_wo_integrator_M->Timing.stepSize1 * 4294967296.0;
}

/* Derivatives for root system: '<Root>' */
void simulink_wo_integrator_derivatives(void)
{
  XDot_simulink_wo_integrator_T *_rtXdot;
  boolean_T lsat;
  boolean_T usat;
  _rtXdot = ((XDot_simulink_wo_integrator_T *) simulink_wo_integrator_M->derivs);

  /* Derivatives for TransferFcn: '<S4>/Pitch: Transfer Fcn' */
  _rtXdot->PitchTransferFcn_CSTATE = 0.0;
  _rtXdot->PitchTransferFcn_CSTATE +=
    simulink_wo_integrator_P.PitchTransferFcn_A *
    simulink_wo_integrator_X.PitchTransferFcn_CSTATE;
  _rtXdot->PitchTransferFcn_CSTATE += simulink_wo_integrator_B.PitchCounttorad;

  /* Derivatives for TransferFcn: '<S4>/Elevation: Transfer Fcn' */
  _rtXdot->ElevationTransferFcn_CSTATE = 0.0;
  _rtXdot->ElevationTransferFcn_CSTATE +=
    simulink_wo_integrator_P.ElevationTransferFcn_A *
    simulink_wo_integrator_X.ElevationTransferFcn_CSTATE;
  _rtXdot->ElevationTransferFcn_CSTATE +=
    simulink_wo_integrator_B.ElevationCounttorad;

  /* Derivatives for TransferFcn: '<S4>/Travel: Transfer Fcn' */
  _rtXdot->TravelTransferFcn_CSTATE = 0.0;
  _rtXdot->TravelTransferFcn_CSTATE +=
    simulink_wo_integrator_P.TravelTransferFcn_A *
    simulink_wo_integrator_X.TravelTransferFcn_CSTATE;
  _rtXdot->TravelTransferFcn_CSTATE += simulink_wo_integrator_B.TravelCounttorad;

  /* Derivatives for Integrator: '<S9>/Integrator' */
  lsat = (simulink_wo_integrator_X.Integrator_CSTATE <=
          simulink_wo_integrator_P.Integrator_LowerSat);
  usat = (simulink_wo_integrator_X.Integrator_CSTATE >=
          simulink_wo_integrator_P.Integrator_UpperSat);
  if (((!lsat) && (!usat)) || (lsat && (simulink_wo_integrator_B.K_ei > 0.0)) ||
      (usat && (simulink_wo_integrator_B.K_ei < 0.0))) {
    _rtXdot->Integrator_CSTATE = simulink_wo_integrator_B.K_ei;
  } else {
    /* in saturation */
    _rtXdot->Integrator_CSTATE = 0.0;
  }

  /* End of Derivatives for Integrator: '<S9>/Integrator' */
}

/* Model output function for TID2 */
void simulink_wo_integrator_output2(void) /* Sample time: [0.01s, 0.0s] */
{
  /* local block i/o variables */
  real_T rtb_GameController_o4;
  real_T rtb_GameController_o5;

  /* S-Function (game_controller_block): '<S6>/Game Controller' */

  /* S-Function Block: simulink_wo_integrator/Joystick/Game Controller (game_controller_block) */
  {
    if (simulink_wo_integrator_P.GameController_Enabled) {
      t_game_controller_states state;
      t_boolean new_data;
      t_error result;
      result = game_controller_poll
        (simulink_wo_integrator_DW.GameController_Controller, &state, &new_data);
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
  simulink_wo_integrator_DW.RateTransitionx_Buffer0 = rtb_GameController_o4;

  /* RateTransition: '<S6>/Rate Transition: y' */
  simulink_wo_integrator_DW.RateTransitiony_Buffer0 = rtb_GameController_o5;
}

/* Model update function for TID2 */
void simulink_wo_integrator_update2(void) /* Sample time: [0.01s, 0.0s] */
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
  if (!(++simulink_wo_integrator_M->Timing.clockTick2)) {
    ++simulink_wo_integrator_M->Timing.clockTickH2;
  }

  simulink_wo_integrator_M->Timing.t[2] =
    simulink_wo_integrator_M->Timing.clockTick2 *
    simulink_wo_integrator_M->Timing.stepSize2 +
    simulink_wo_integrator_M->Timing.clockTickH2 *
    simulink_wo_integrator_M->Timing.stepSize2 * 4294967296.0;
}

/* Model output wrapper function for compatibility with a static main program */
void simulink_wo_integrator_output(int_T tid)
{
  switch (tid) {
   case 0 :
    simulink_wo_integrator_output0();
    break;

   case 2 :
    simulink_wo_integrator_output2();
    break;

   default :
    break;
  }
}

/* Model update wrapper function for compatibility with a static main program */
void simulink_wo_integrator_update(int_T tid)
{
  switch (tid) {
   case 0 :
    simulink_wo_integrator_update0();
    break;

   case 2 :
    simulink_wo_integrator_update2();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void simulink_wo_integrator_initialize(void)
{
  /* Start for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: simulink_wo_integrator/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("q8_usb", "0",
                      &simulink_wo_integrator_DW.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
      return;
    }

    is_switching = false;
    result = hil_set_card_specific_options
      (simulink_wo_integrator_DW.HILInitialize_Card,
       "update_rate=normal;decimation=1", 32);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
      return;
    }

    result = hil_watchdog_clear(simulink_wo_integrator_DW.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
      return;
    }

    if ((simulink_wo_integrator_P.HILInitialize_AIPStart && !is_switching) ||
        (simulink_wo_integrator_P.HILInitialize_AIPEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AIMinimums =
          &simulink_wo_integrator_DW.HILInitialize_AIMinimums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AIMinimums[i1] = (simulink_wo_integrator_P.HILInitialize_AILow);
        }
      }

      {
        int_T i1;
        real_T *dw_AIMaximums =
          &simulink_wo_integrator_DW.HILInitialize_AIMaximums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AIMaximums[i1] = simulink_wo_integrator_P.HILInitialize_AIHigh;
        }
      }

      result = hil_set_analog_input_ranges
        (simulink_wo_integrator_DW.HILInitialize_Card,
         simulink_wo_integrator_P.HILInitialize_AIChannels, 8U,
         &simulink_wo_integrator_DW.HILInitialize_AIMinimums[0],
         &simulink_wo_integrator_DW.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_wo_integrator_P.HILInitialize_AOPStart && !is_switching) ||
        (simulink_wo_integrator_P.HILInitialize_AOPEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOMinimums =
          &simulink_wo_integrator_DW.HILInitialize_AOMinimums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOMinimums[i1] = (simulink_wo_integrator_P.HILInitialize_AOLow);
        }
      }

      {
        int_T i1;
        real_T *dw_AOMaximums =
          &simulink_wo_integrator_DW.HILInitialize_AOMaximums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOMaximums[i1] = simulink_wo_integrator_P.HILInitialize_AOHigh;
        }
      }

      result = hil_set_analog_output_ranges
        (simulink_wo_integrator_DW.HILInitialize_Card,
         simulink_wo_integrator_P.HILInitialize_AOChannels, 8U,
         &simulink_wo_integrator_DW.HILInitialize_AOMinimums[0],
         &simulink_wo_integrator_DW.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_wo_integrator_P.HILInitialize_AOStart && !is_switching) ||
        (simulink_wo_integrator_P.HILInitialize_AOEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOVoltages =
          &simulink_wo_integrator_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = simulink_wo_integrator_P.HILInitialize_AOInitial;
        }
      }

      result = hil_write_analog(simulink_wo_integrator_DW.HILInitialize_Card,
        simulink_wo_integrator_P.HILInitialize_AOChannels, 8U,
        &simulink_wo_integrator_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }
    }

    if (simulink_wo_integrator_P.HILInitialize_AOReset) {
      {
        int_T i1;
        real_T *dw_AOVoltages =
          &simulink_wo_integrator_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = simulink_wo_integrator_P.HILInitialize_AOWatchdog;
        }
      }

      result = hil_watchdog_set_analog_expiration_state
        (simulink_wo_integrator_DW.HILInitialize_Card,
         simulink_wo_integrator_P.HILInitialize_AOChannels, 8U,
         &simulink_wo_integrator_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_wo_integrator_P.HILInitialize_EIPStart && !is_switching) ||
        (simulink_wo_integrator_P.HILInitialize_EIPEnter && is_switching)) {
      {
        int_T i1;
        int32_T *dw_QuadratureModes =
          &simulink_wo_integrator_DW.HILInitialize_QuadratureModes[0];
        for (i1=0; i1 < 8; i1++) {
          dw_QuadratureModes[i1] =
            simulink_wo_integrator_P.HILInitialize_EIQuadrature;
        }
      }

      result = hil_set_encoder_quadrature_mode
        (simulink_wo_integrator_DW.HILInitialize_Card,
         simulink_wo_integrator_P.HILInitialize_EIChannels, 8U,
         (t_encoder_quadrature_mode *)
         &simulink_wo_integrator_DW.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_wo_integrator_P.HILInitialize_EIStart && !is_switching) ||
        (simulink_wo_integrator_P.HILInitialize_EIEnter && is_switching)) {
      {
        int_T i1;
        int32_T *dw_InitialEICounts =
          &simulink_wo_integrator_DW.HILInitialize_InitialEICounts[0];
        for (i1=0; i1 < 8; i1++) {
          dw_InitialEICounts[i1] =
            simulink_wo_integrator_P.HILInitialize_EIInitial;
        }
      }

      result = hil_set_encoder_counts
        (simulink_wo_integrator_DW.HILInitialize_Card,
         simulink_wo_integrator_P.HILInitialize_EIChannels, 8U,
         &simulink_wo_integrator_DW.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_wo_integrator_P.HILInitialize_POPStart && !is_switching) ||
        (simulink_wo_integrator_P.HILInitialize_POPEnter && is_switching)) {
      uint32_T num_duty_cycle_modes = 0;
      uint32_T num_frequency_modes = 0;

      {
        int_T i1;
        int32_T *dw_POModeValues =
          &simulink_wo_integrator_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POModeValues[i1] = simulink_wo_integrator_P.HILInitialize_POModes;
        }
      }

      result = hil_set_pwm_mode(simulink_wo_integrator_DW.HILInitialize_Card,
        simulink_wo_integrator_P.HILInitialize_POChannels, 8U, (t_pwm_mode *)
        &simulink_wo_integrator_DW.HILInitialize_POModeValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }

      {
        int_T i1;
        const uint32_T *p_HILInitialize_POChannels =
          simulink_wo_integrator_P.HILInitialize_POChannels;
        int32_T *dw_POModeValues =
          &simulink_wo_integrator_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          if (dw_POModeValues[i1] == PWM_DUTY_CYCLE_MODE || dw_POModeValues[i1] ==
              PWM_ONE_SHOT_MODE || dw_POModeValues[i1] == PWM_TIME_MODE ||
              dw_POModeValues[i1] == PWM_RAW_MODE) {
            simulink_wo_integrator_DW.HILInitialize_POSortedChans[num_duty_cycle_modes]
              = (p_HILInitialize_POChannels[i1]);
            simulink_wo_integrator_DW.HILInitialize_POSortedFreqs[num_duty_cycle_modes]
              = simulink_wo_integrator_P.HILInitialize_POFrequency;
            num_duty_cycle_modes++;
          } else {
            simulink_wo_integrator_DW.HILInitialize_POSortedChans[7U -
              num_frequency_modes] = (p_HILInitialize_POChannels[i1]);
            simulink_wo_integrator_DW.HILInitialize_POSortedFreqs[7U -
              num_frequency_modes] =
              simulink_wo_integrator_P.HILInitialize_POFrequency;
            num_frequency_modes++;
          }
        }
      }

      if (num_duty_cycle_modes > 0) {
        result = hil_set_pwm_frequency
          (simulink_wo_integrator_DW.HILInitialize_Card,
           &simulink_wo_integrator_DW.HILInitialize_POSortedChans[0],
           num_duty_cycle_modes,
           &simulink_wo_integrator_DW.HILInitialize_POSortedFreqs[0]);
        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
          return;
        }
      }

      if (num_frequency_modes > 0) {
        result = hil_set_pwm_duty_cycle
          (simulink_wo_integrator_DW.HILInitialize_Card,
           &simulink_wo_integrator_DW.HILInitialize_POSortedChans[num_duty_cycle_modes],
           num_frequency_modes,
           &simulink_wo_integrator_DW.HILInitialize_POSortedFreqs[num_duty_cycle_modes]);
        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
          return;
        }
      }

      {
        int_T i1;
        int32_T *dw_POModeValues =
          &simulink_wo_integrator_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POModeValues[i1] =
            simulink_wo_integrator_P.HILInitialize_POConfiguration;
        }
      }

      {
        int_T i1;
        int32_T *dw_POAlignValues =
          &simulink_wo_integrator_DW.HILInitialize_POAlignValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POAlignValues[i1] =
            simulink_wo_integrator_P.HILInitialize_POAlignment;
        }
      }

      {
        int_T i1;
        int32_T *dw_POPolarityVals =
          &simulink_wo_integrator_DW.HILInitialize_POPolarityVals[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POPolarityVals[i1] =
            simulink_wo_integrator_P.HILInitialize_POPolarity;
        }
      }

      result = hil_set_pwm_configuration
        (simulink_wo_integrator_DW.HILInitialize_Card,
         simulink_wo_integrator_P.HILInitialize_POChannels, 8U,
         (t_pwm_configuration *)
         &simulink_wo_integrator_DW.HILInitialize_POModeValues[0],
         (t_pwm_alignment *)
         &simulink_wo_integrator_DW.HILInitialize_POAlignValues[0],
         (t_pwm_polarity *)
         &simulink_wo_integrator_DW.HILInitialize_POPolarityVals[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }

      {
        int_T i1;
        real_T *dw_POSortedFreqs =
          &simulink_wo_integrator_DW.HILInitialize_POSortedFreqs[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POSortedFreqs[i1] =
            simulink_wo_integrator_P.HILInitialize_POLeading;
        }
      }

      {
        int_T i1;
        real_T *dw_POValues = &simulink_wo_integrator_DW.HILInitialize_POValues
          [0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_wo_integrator_P.HILInitialize_POTrailing;
        }
      }

      result = hil_set_pwm_deadband(simulink_wo_integrator_DW.HILInitialize_Card,
        simulink_wo_integrator_P.HILInitialize_POChannels, 8U,
        &simulink_wo_integrator_DW.HILInitialize_POSortedFreqs[0],
        &simulink_wo_integrator_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_wo_integrator_P.HILInitialize_POStart && !is_switching) ||
        (simulink_wo_integrator_P.HILInitialize_POEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_POValues = &simulink_wo_integrator_DW.HILInitialize_POValues
          [0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_wo_integrator_P.HILInitialize_POInitial;
        }
      }

      result = hil_write_pwm(simulink_wo_integrator_DW.HILInitialize_Card,
        simulink_wo_integrator_P.HILInitialize_POChannels, 8U,
        &simulink_wo_integrator_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }
    }

    if (simulink_wo_integrator_P.HILInitialize_POReset) {
      {
        int_T i1;
        real_T *dw_POValues = &simulink_wo_integrator_DW.HILInitialize_POValues
          [0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_wo_integrator_P.HILInitialize_POWatchdog;
        }
      }

      result = hil_watchdog_set_pwm_expiration_state
        (simulink_wo_integrator_DW.HILInitialize_Card,
         simulink_wo_integrator_P.HILInitialize_POChannels, 8U,
         &simulink_wo_integrator_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for S-Function (hil_read_encoder_timebase_block): '<S4>/HIL Read Encoder Timebase' */

  /* S-Function Block: simulink_wo_integrator/Heli 3D/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
  {
    t_error result;
    result = hil_task_create_encoder_reader
      (simulink_wo_integrator_DW.HILInitialize_Card,
       simulink_wo_integrator_P.HILReadEncoderTimebase_SamplesI,
       simulink_wo_integrator_P.HILReadEncoderTimebase_Channels, 3,
       &simulink_wo_integrator_DW.HILReadEncoderTimebase_Task);
    if (result >= 0) {
      result = hil_task_set_buffer_overflow_mode
        (simulink_wo_integrator_DW.HILReadEncoderTimebase_Task,
         (t_buffer_overflow_mode)
         (simulink_wo_integrator_P.HILReadEncoderTimebase_Overflow - 1));
    }

    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
    }
  }

  /* Start for S-Function (stream_call_block): '<S11>/Stream Call1' incorporates:
   *  Constant: '<S11>/Constant'
   *  S-Function (string_constant_block): '<S11>/String Constant'
   */

  /* S-Function Block: simulink_wo_integrator/IMU system/IMU/Stream Call1 (stream_call_block) */
  {
    simulink_wo_integrator_DW.StreamCall1_State =
      STREAM_CALL_STATE_NOT_CONNECTED;
    simulink_wo_integrator_DW.StreamCall1_Stream = NULL;
  }

  /* Start for RateTransition: '<S6>/Rate Transition: x' */
  simulink_wo_integrator_B.RateTransitionx =
    simulink_wo_integrator_P.RateTransitionx_InitialConditio;

  /* Start for RateTransition: '<S6>/Rate Transition: y' */
  simulink_wo_integrator_B.RateTransitiony =
    simulink_wo_integrator_P.RateTransitiony_InitialConditio;

  /* Start for ToFile: '<Root>/To File' */
  {
    FILE *fp = (NULL);
    char fileName[509] = "enc_rates_vs_imu_rates_func_block_w_offset_added.mat";
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(simulink_wo_integrator_M,
                        "Error creating .mat file enc_rates_vs_imu_rates_func_block_w_offset_added.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp, 6 + 1, 0, "ans")) {
      rtmSetErrorStatus(simulink_wo_integrator_M,
                        "Error writing mat file header to file enc_rates_vs_imu_rates_func_block_w_offset_added.mat");
      return;
    }

    simulink_wo_integrator_DW.ToFile_IWORK.Count = 0;
    simulink_wo_integrator_DW.ToFile_IWORK.Decimation = -1;
    simulink_wo_integrator_DW.ToFile_PWORK.FilePtr = fp;
  }

  /* Start for ToFile: '<Root>/To File1' */
  {
    FILE *fp = (NULL);
    char fileName[509] = "lab2_Q_1_1_20_R_1_0.5.mat";
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(simulink_wo_integrator_M,
                        "Error creating .mat file lab2_Q_1_1_20_R_1_0.5.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp, 1 + 1, 0, "ans")) {
      rtmSetErrorStatus(simulink_wo_integrator_M,
                        "Error writing mat file header to file lab2_Q_1_1_20_R_1_0.5.mat");
      return;
    }

    simulink_wo_integrator_DW.ToFile1_IWORK.Count = 0;
    simulink_wo_integrator_DW.ToFile1_IWORK.Decimation = -1;
    simulink_wo_integrator_DW.ToFile1_PWORK.FilePtr = fp;
  }

  /* Start for S-Function (game_controller_block): '<S6>/Game Controller' */

  /* S-Function Block: simulink_wo_integrator/Joystick/Game Controller (game_controller_block) */
  {
    if (simulink_wo_integrator_P.GameController_Enabled) {
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
        (simulink_wo_integrator_P.GameController_ControllerNumber,
         simulink_wo_integrator_P.GameController_BufferSize, deadzone,
         saturation, simulink_wo_integrator_P.GameController_AutoCenter, 0, 1.0,
         &simulink_wo_integrator_DW.GameController_Controller);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
      }
    }
  }

  /* InitializeConditions for Memory: '<S11>/Memory' */
  memcpy(&simulink_wo_integrator_DW.Memory_PreviousInput[0],
         &simulink_wo_integrator_P.Memory_InitialCondition[0], 10U * sizeof
         (real_T));

  /* InitializeConditions for RateTransition: '<S6>/Rate Transition: x' */
  simulink_wo_integrator_DW.RateTransitionx_Buffer0 =
    simulink_wo_integrator_P.RateTransitionx_InitialConditio;

  /* InitializeConditions for RateTransition: '<S6>/Rate Transition: y' */
  simulink_wo_integrator_DW.RateTransitiony_Buffer0 =
    simulink_wo_integrator_P.RateTransitiony_InitialConditio;

  /* InitializeConditions for TransferFcn: '<S4>/Pitch: Transfer Fcn' */
  simulink_wo_integrator_X.PitchTransferFcn_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S4>/Elevation: Transfer Fcn' */
  simulink_wo_integrator_X.ElevationTransferFcn_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S4>/Travel: Transfer Fcn' */
  simulink_wo_integrator_X.TravelTransferFcn_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S9>/Integrator' */
  simulink_wo_integrator_X.Integrator_CSTATE =
    simulink_wo_integrator_P.Integrator_IC;
}

/* Model terminate function */
void simulink_wo_integrator_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: simulink_wo_integrator/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    t_uint32 num_final_analog_outputs = 0;
    t_uint32 num_final_pwm_outputs = 0;
    hil_task_stop_all(simulink_wo_integrator_DW.HILInitialize_Card);
    hil_monitor_stop_all(simulink_wo_integrator_DW.HILInitialize_Card);
    is_switching = false;
    if ((simulink_wo_integrator_P.HILInitialize_AOTerminate && !is_switching) ||
        (simulink_wo_integrator_P.HILInitialize_AOExit && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOVoltages =
          &simulink_wo_integrator_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = simulink_wo_integrator_P.HILInitialize_AOFinal;
        }
      }

      num_final_analog_outputs = 8U;
    } else {
      num_final_analog_outputs = 0;
    }

    if ((simulink_wo_integrator_P.HILInitialize_POTerminate && !is_switching) ||
        (simulink_wo_integrator_P.HILInitialize_POExit && is_switching)) {
      {
        int_T i1;
        real_T *dw_POValues = &simulink_wo_integrator_DW.HILInitialize_POValues
          [0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_wo_integrator_P.HILInitialize_POFinal;
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
      result = hil_write(simulink_wo_integrator_DW.HILInitialize_Card
                         , simulink_wo_integrator_P.HILInitialize_AOChannels,
                         num_final_analog_outputs
                         , simulink_wo_integrator_P.HILInitialize_POChannels,
                         num_final_pwm_outputs
                         , NULL, 0
                         , NULL, 0
                         , &simulink_wo_integrator_DW.HILInitialize_AOVoltages[0]
                         , &simulink_wo_integrator_DW.HILInitialize_POValues[0]
                         , (t_boolean *) NULL
                         , NULL
                         );
      if (result == -QERR_HIL_WRITE_NOT_SUPPORTED) {
        t_error local_result;
        result = 0;

        /* The hil_write operation is not supported by this card. Write final outputs for each channel type */
        if (num_final_analog_outputs > 0) {
          local_result = hil_write_analog
            (simulink_wo_integrator_DW.HILInitialize_Card,
             simulink_wo_integrator_P.HILInitialize_AOChannels,
             num_final_analog_outputs,
             &simulink_wo_integrator_DW.HILInitialize_AOVoltages[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_pwm_outputs > 0) {
          local_result = hil_write_pwm
            (simulink_wo_integrator_DW.HILInitialize_Card,
             simulink_wo_integrator_P.HILInitialize_POChannels,
             num_final_pwm_outputs,
             &simulink_wo_integrator_DW.HILInitialize_POValues[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(simulink_wo_integrator_M, _rt_error_message);
        }
      }
    }

    hil_task_delete_all(simulink_wo_integrator_DW.HILInitialize_Card);
    hil_monitor_delete_all(simulink_wo_integrator_DW.HILInitialize_Card);
    hil_close(simulink_wo_integrator_DW.HILInitialize_Card);
    simulink_wo_integrator_DW.HILInitialize_Card = NULL;
  }

  /* Terminate for S-Function (stream_call_block): '<S11>/Stream Call1' incorporates:
   *  Constant: '<S11>/Constant'
   *  S-Function (string_constant_block): '<S11>/String Constant'
   */

  /* S-Function Block: simulink_wo_integrator/IMU system/IMU/Stream Call1 (stream_call_block) */
  {
    if (simulink_wo_integrator_DW.StreamCall1_Stream != NULL) {
      stream_close(simulink_wo_integrator_DW.StreamCall1_Stream);
      simulink_wo_integrator_DW.StreamCall1_Stream = NULL;
    }
  }

  /* Terminate for ToFile: '<Root>/To File' */
  {
    FILE *fp = (FILE *) simulink_wo_integrator_DW.ToFile_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] =
        "enc_rates_vs_imu_rates_func_block_w_offset_added.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(simulink_wo_integrator_M,
                          "Error closing MAT-file enc_rates_vs_imu_rates_func_block_w_offset_added.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(simulink_wo_integrator_M,
                          "Error reopening MAT-file enc_rates_vs_imu_rates_func_block_w_offset_added.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 6 + 1,
           simulink_wo_integrator_DW.ToFile_IWORK.Count, "ans")) {
        rtmSetErrorStatus(simulink_wo_integrator_M,
                          "Error writing header for ans to MAT-file enc_rates_vs_imu_rates_func_block_w_offset_added.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(simulink_wo_integrator_M,
                          "Error closing MAT-file enc_rates_vs_imu_rates_func_block_w_offset_added.mat");
        return;
      }

      simulink_wo_integrator_DW.ToFile_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File1' */
  {
    FILE *fp = (FILE *) simulink_wo_integrator_DW.ToFile1_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "lab2_Q_1_1_20_R_1_0.5.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(simulink_wo_integrator_M,
                          "Error closing MAT-file lab2_Q_1_1_20_R_1_0.5.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(simulink_wo_integrator_M,
                          "Error reopening MAT-file lab2_Q_1_1_20_R_1_0.5.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 1 + 1,
           simulink_wo_integrator_DW.ToFile1_IWORK.Count, "ans")) {
        rtmSetErrorStatus(simulink_wo_integrator_M,
                          "Error writing header for ans to MAT-file lab2_Q_1_1_20_R_1_0.5.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(simulink_wo_integrator_M,
                          "Error closing MAT-file lab2_Q_1_1_20_R_1_0.5.mat");
        return;
      }

      simulink_wo_integrator_DW.ToFile1_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for S-Function (game_controller_block): '<S6>/Game Controller' */

  /* S-Function Block: simulink_wo_integrator/Joystick/Game Controller (game_controller_block) */
  {
    if (simulink_wo_integrator_P.GameController_Enabled) {
      game_controller_close(simulink_wo_integrator_DW.GameController_Controller);
      simulink_wo_integrator_DW.GameController_Controller = NULL;
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
  simulink_wo_integrator_output(tid);
}

void MdlUpdate(int_T tid)
{
  if (tid == 1)
    tid = 0;
  simulink_wo_integrator_update(tid);
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
  simulink_wo_integrator_initialize();
}

void MdlTerminate(void)
{
  simulink_wo_integrator_terminate();
}

/* Registration function */
RT_MODEL_simulink_wo_integrat_T *simulink_wo_integrator(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  simulink_wo_integrator_P.Integrator_UpperSat = rtInf;
  simulink_wo_integrator_P.Integrator_LowerSat = rtMinusInf;

  /* initialize real-time model */
  (void) memset((void *)simulink_wo_integrator_M, 0,
                sizeof(RT_MODEL_simulink_wo_integrat_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&simulink_wo_integrator_M->solverInfo,
                          &simulink_wo_integrator_M->Timing.simTimeStep);
    rtsiSetTPtr(&simulink_wo_integrator_M->solverInfo, &rtmGetTPtr
                (simulink_wo_integrator_M));
    rtsiSetStepSizePtr(&simulink_wo_integrator_M->solverInfo,
                       &simulink_wo_integrator_M->Timing.stepSize0);
    rtsiSetdXPtr(&simulink_wo_integrator_M->solverInfo,
                 &simulink_wo_integrator_M->derivs);
    rtsiSetContStatesPtr(&simulink_wo_integrator_M->solverInfo, (real_T **)
                         &simulink_wo_integrator_M->contStates);
    rtsiSetNumContStatesPtr(&simulink_wo_integrator_M->solverInfo,
      &simulink_wo_integrator_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&simulink_wo_integrator_M->solverInfo,
      &simulink_wo_integrator_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&simulink_wo_integrator_M->solverInfo,
      &simulink_wo_integrator_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&simulink_wo_integrator_M->solverInfo,
      &simulink_wo_integrator_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&simulink_wo_integrator_M->solverInfo,
                          (&rtmGetErrorStatus(simulink_wo_integrator_M)));
    rtsiSetRTModelPtr(&simulink_wo_integrator_M->solverInfo,
                      simulink_wo_integrator_M);
  }

  rtsiSetSimTimeStep(&simulink_wo_integrator_M->solverInfo, MAJOR_TIME_STEP);
  simulink_wo_integrator_M->intgData.f[0] = simulink_wo_integrator_M->odeF[0];
  simulink_wo_integrator_M->contStates = ((real_T *) &simulink_wo_integrator_X);
  rtsiSetSolverData(&simulink_wo_integrator_M->solverInfo, (void *)
                    &simulink_wo_integrator_M->intgData);
  rtsiSetSolverName(&simulink_wo_integrator_M->solverInfo,"ode1");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = simulink_wo_integrator_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    mdlTsMap[2] = 2;
    simulink_wo_integrator_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    simulink_wo_integrator_M->Timing.sampleTimes =
      (&simulink_wo_integrator_M->Timing.sampleTimesArray[0]);
    simulink_wo_integrator_M->Timing.offsetTimes =
      (&simulink_wo_integrator_M->Timing.offsetTimesArray[0]);

    /* task periods */
    simulink_wo_integrator_M->Timing.sampleTimes[0] = (0.0);
    simulink_wo_integrator_M->Timing.sampleTimes[1] = (0.002);
    simulink_wo_integrator_M->Timing.sampleTimes[2] = (0.01);

    /* task offsets */
    simulink_wo_integrator_M->Timing.offsetTimes[0] = (0.0);
    simulink_wo_integrator_M->Timing.offsetTimes[1] = (0.0);
    simulink_wo_integrator_M->Timing.offsetTimes[2] = (0.0);
  }

  rtmSetTPtr(simulink_wo_integrator_M, &simulink_wo_integrator_M->Timing.tArray
             [0]);

  {
    int_T *mdlSampleHits = simulink_wo_integrator_M->Timing.sampleHitArray;
    int_T *mdlPerTaskSampleHits =
      simulink_wo_integrator_M->Timing.perTaskSampleHitsArray;
    simulink_wo_integrator_M->Timing.perTaskSampleHits = (&mdlPerTaskSampleHits
      [0]);
    mdlSampleHits[0] = 1;
    simulink_wo_integrator_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(simulink_wo_integrator_M, -1);
  simulink_wo_integrator_M->Timing.stepSize0 = 0.002;
  simulink_wo_integrator_M->Timing.stepSize1 = 0.002;
  simulink_wo_integrator_M->Timing.stepSize2 = 0.01;

  /* External mode info */
  simulink_wo_integrator_M->Sizes.checksums[0] = (1440101632U);
  simulink_wo_integrator_M->Sizes.checksums[1] = (471666128U);
  simulink_wo_integrator_M->Sizes.checksums[2] = (2378170468U);
  simulink_wo_integrator_M->Sizes.checksums[3] = (3398417479U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[5];
    simulink_wo_integrator_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(simulink_wo_integrator_M->extModeInfo,
      &simulink_wo_integrator_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(simulink_wo_integrator_M->extModeInfo,
                        simulink_wo_integrator_M->Sizes.checksums);
    rteiSetTPtr(simulink_wo_integrator_M->extModeInfo, rtmGetTPtr
                (simulink_wo_integrator_M));
  }

  simulink_wo_integrator_M->solverInfoPtr =
    (&simulink_wo_integrator_M->solverInfo);
  simulink_wo_integrator_M->Timing.stepSize = (0.002);
  rtsiSetFixedStepSize(&simulink_wo_integrator_M->solverInfo, 0.002);
  rtsiSetSolverMode(&simulink_wo_integrator_M->solverInfo,
                    SOLVER_MODE_MULTITASKING);

  /* block I/O */
  simulink_wo_integrator_M->blockIO = ((void *) &simulink_wo_integrator_B);
  (void) memset(((void *) &simulink_wo_integrator_B), 0,
                sizeof(B_simulink_wo_integrator_T));

  {
    int32_T i;
    for (i = 0; i < 10; i++) {
      simulink_wo_integrator_B.Switch[i] = 0.0;
    }

    simulink_wo_integrator_B.Gain2[0] = 0.0;
    simulink_wo_integrator_B.Gain2[1] = 0.0;
    simulink_wo_integrator_B.Gain2[2] = 0.0;
    simulink_wo_integrator_B.RateTransitionx = 0.0;
    simulink_wo_integrator_B.Joystick_gain_x = 0.0;
    simulink_wo_integrator_B.RateTransitiony = 0.0;
    simulink_wo_integrator_B.Joystick_gain_y = 0.0;
    simulink_wo_integrator_B.Gain[0] = 0.0;
    simulink_wo_integrator_B.Gain[1] = 0.0;
    simulink_wo_integrator_B.PitchCounttorad = 0.0;
    simulink_wo_integrator_B.PitchTransferFcn = 0.0;
    simulink_wo_integrator_B.ElevationCounttorad = 0.0;
    simulink_wo_integrator_B.ElevationTransferFcn = 0.0;
    simulink_wo_integrator_B.Sum[0] = 0.0;
    simulink_wo_integrator_B.Sum[1] = 0.0;
    simulink_wo_integrator_B.TravelCounttorad = 0.0;
    simulink_wo_integrator_B.TravelTransferFcn = 0.0;
    simulink_wo_integrator_B.Sum_m = 0.0;
    simulink_wo_integrator_B.Sum_o = 0.0;
    simulink_wo_integrator_B.Gain1[0] = 0.0;
    simulink_wo_integrator_B.Gain1[1] = 0.0;
    simulink_wo_integrator_B.Gain1[2] = 0.0;
    simulink_wo_integrator_B.Sum_c = 0.0;
    simulink_wo_integrator_B.Sum1 = 0.0;
    simulink_wo_integrator_B.K_ei = 0.0;
    simulink_wo_integrator_B.sf_Gyrovectortopitchrateeleva_h.euler_rates[0] =
      0.0;
    simulink_wo_integrator_B.sf_Gyrovectortopitchrateeleva_h.euler_rates[1] =
      0.0;
    simulink_wo_integrator_B.sf_Gyrovectortopitchrateeleva_h.euler_rates[2] =
      0.0;
    simulink_wo_integrator_B.sf_Gyrovectortopitchrateelevati.euler_rates[0] =
      0.0;
    simulink_wo_integrator_B.sf_Gyrovectortopitchrateelevati.euler_rates[1] =
      0.0;
    simulink_wo_integrator_B.sf_Gyrovectortopitchrateelevati.euler_rates[2] =
      0.0;
  }

  /* parameters */
  simulink_wo_integrator_M->defaultParam = ((real_T *)&simulink_wo_integrator_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &simulink_wo_integrator_X;
    simulink_wo_integrator_M->contStates = (x);
    (void) memset((void *)&simulink_wo_integrator_X, 0,
                  sizeof(X_simulink_wo_integrator_T));
  }

  /* states (dwork) */
  simulink_wo_integrator_M->dwork = ((void *) &simulink_wo_integrator_DW);
  (void) memset((void *)&simulink_wo_integrator_DW, 0,
                sizeof(DW_simulink_wo_integrator_T));

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_wo_integrator_DW.HILInitialize_AIMinimums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_wo_integrator_DW.HILInitialize_AIMaximums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_wo_integrator_DW.HILInitialize_AOMinimums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_wo_integrator_DW.HILInitialize_AOMaximums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_wo_integrator_DW.HILInitialize_AOVoltages[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_wo_integrator_DW.HILInitialize_FilterFrequency[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_wo_integrator_DW.HILInitialize_POSortedFreqs[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_wo_integrator_DW.HILInitialize_POValues[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 10; i++) {
      simulink_wo_integrator_DW.Memory_PreviousInput[i] = 0.0;
    }
  }

  simulink_wo_integrator_DW.RateTransitionx_Buffer0 = 0.0;
  simulink_wo_integrator_DW.RateTransitiony_Buffer0 = 0.0;
  simulink_wo_integrator_DW.HILWriteAnalog_Buffer[0] = 0.0;
  simulink_wo_integrator_DW.HILWriteAnalog_Buffer[1] = 0.0;

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    simulink_wo_integrator_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 25;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.PTransTable = &rtPTransTable;
  }

  /* Initialize Sizes */
  simulink_wo_integrator_M->Sizes.numContStates = (4);/* Number of continuous states */
  simulink_wo_integrator_M->Sizes.numPeriodicContStates = (0);
                                      /* Number of periodic continuous states */
  simulink_wo_integrator_M->Sizes.numY = (0);/* Number of model outputs */
  simulink_wo_integrator_M->Sizes.numU = (0);/* Number of model inputs */
  simulink_wo_integrator_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  simulink_wo_integrator_M->Sizes.numSampTimes = (3);/* Number of sample times */
  simulink_wo_integrator_M->Sizes.numBlocks = (74);/* Number of blocks */
  simulink_wo_integrator_M->Sizes.numBlockIO = (23);/* Number of block outputs */
  simulink_wo_integrator_M->Sizes.numBlockPrms = (444);/* Sum of parameter "widths" */
  return simulink_wo_integrator_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
