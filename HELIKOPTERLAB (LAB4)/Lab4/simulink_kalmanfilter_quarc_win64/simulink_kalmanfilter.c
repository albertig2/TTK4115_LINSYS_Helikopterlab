/*
 * simulink_kalmanfilter.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "simulink_kalmanfilter".
 *
 * Model version              : 11.45
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C source code generated on : Mon Oct 27 18:39:43 2025
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "simulink_kalmanfilter.h"
#include "simulink_kalmanfilter_private.h"
#include "simulink_kalmanfilter_dt.h"

t_stream simulink_kalmanfilter_rtZt_stream = NULL;

/* Block signals (default storage) */
B_simulink_kalmanfilter_T simulink_kalmanfilter_B;

/* Continuous states */
X_simulink_kalmanfilter_T simulink_kalmanfilter_X;

/* Block states (default storage) */
DW_simulink_kalmanfilter_T simulink_kalmanfilter_DW;

/* Real-time model */
static RT_MODEL_simulink_kalmanfilte_T simulink_kalmanfilter_M_;
RT_MODEL_simulink_kalmanfilte_T *const simulink_kalmanfilter_M =
  &simulink_kalmanfilter_M_;
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
  rtmSampleHitPtr[1] = rtmStepTask(simulink_kalmanfilter_M, 1);
  rtmSampleHitPtr[2] = rtmStepTask(simulink_kalmanfilter_M, 2);
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
  if (simulink_kalmanfilter_M->Timing.TaskCounters.TID[1] == 0) {
    simulink_kalmanfilter_M->Timing.RateInteraction.TID1_2 =
      (simulink_kalmanfilter_M->Timing.TaskCounters.TID[2] == 0);

    /* update PerTaskSampleHits matrix for non-inline sfcn */
    simulink_kalmanfilter_M->Timing.perTaskSampleHits[5] =
      simulink_kalmanfilter_M->Timing.RateInteraction.TID1_2;
  }

  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (simulink_kalmanfilter_M->Timing.TaskCounters.TID[2])++;
  if ((simulink_kalmanfilter_M->Timing.TaskCounters.TID[2]) > 4) {/* Sample time: [0.01s, 0.0s] */
    simulink_kalmanfilter_M->Timing.TaskCounters.TID[2] = 0;
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
  int_T nXc = 6;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  simulink_kalmanfilter_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model output function for TID0 */
void simulink_kalmanfilter_output0(void) /* Sample time: [0.0s, 0.0s] */
{
  /* local block i/o variables */
  t_stream_ptr rtb_StreamCall1_o1;
  real_T rtb_HILReadEncoderTimebase_o1;
  real_T rtb_HILReadEncoderTimebase_o2;
  real_T rtb_HILReadEncoderTimebase_o3;
  real_T rtb_TmpSignalConversionAtToFile[19];
  real32_T rtb_StreamRead1_o2[10];
  int32_T rtb_StreamFormattedWrite_o2;
  int32_T rtb_StreamCall1_o3;
  boolean_T rtb_StreamRead1_o3;
  real_T P_hat_tmp[36];
  real_T b_I_0[36];
  real_T c_I_0[36];
  real_T c_I_1[36];
  real_T A_tmp[30];
  real_T L[30];
  real_T tmp_1[30];
  real_T A[25];
  real_T b[25];
  real_T tmp[9];
  real_T tmp_0[6];
  real_T tmp_2[5];
  real_T tmp_3[5];
  real_T tmp_4[5];
  real_T rtb_Gain2[3];
  real_T az;
  real_T euler_rates_tmp;
  real_T euler_rates_tmp_0;
  real_T smax;
  int32_T A_tmp_0;
  int32_T c_ix;
  int32_T d_k;
  int32_T e;
  int32_T i;
  int32_T ijA;
  int32_T ix;
  int32_T iy;
  int8_T b_I[36];
  int8_T c_I[36];
  int8_T ipiv[5];
  int8_T p[5];
  if (rtmIsMajorTimeStep(simulink_kalmanfilter_M)) {
    /* set solver stop time */
    if (!(simulink_kalmanfilter_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&simulink_kalmanfilter_M->solverInfo,
                            ((simulink_kalmanfilter_M->Timing.clockTickH0 + 1) *
        simulink_kalmanfilter_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&simulink_kalmanfilter_M->solverInfo,
                            ((simulink_kalmanfilter_M->Timing.clockTick0 + 1) *
        simulink_kalmanfilter_M->Timing.stepSize0 +
        simulink_kalmanfilter_M->Timing.clockTickH0 *
        simulink_kalmanfilter_M->Timing.stepSize0 * 4294967296.0));
    }

    {                                  /* Sample time: [0.0s, 0.0s] */
      rate_monotonic_scheduler();
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(simulink_kalmanfilter_M)) {
    simulink_kalmanfilter_M->Timing.t[0] = rtsiGetT
      (&simulink_kalmanfilter_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(simulink_kalmanfilter_M)) {
    /* S-Function (hil_read_encoder_timebase_block): '<S3>/HIL Read Encoder Timebase' */

    /* S-Function Block: simulink_kalmanfilter/Heli 3D/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
    {
      t_error result;
      result = hil_task_read_encoder
        (simulink_kalmanfilter_DW.HILReadEncoderTimebase_Task, 1,
         &simulink_kalmanfilter_DW.HILReadEncoderTimebase_Buffer[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
      } else {
        rtb_HILReadEncoderTimebase_o1 =
          simulink_kalmanfilter_DW.HILReadEncoderTimebase_Buffer[0];
        rtb_HILReadEncoderTimebase_o2 =
          simulink_kalmanfilter_DW.HILReadEncoderTimebase_Buffer[1];
        rtb_HILReadEncoderTimebase_o3 =
          simulink_kalmanfilter_DW.HILReadEncoderTimebase_Buffer[2];
      }
    }

    /* RateTransition: '<S5>/Rate Transition: x' */
    if (simulink_kalmanfilter_M->Timing.RateInteraction.TID1_2) {
      /* RateTransition: '<S5>/Rate Transition: x' */
      simulink_kalmanfilter_B.RateTransitionx =
        simulink_kalmanfilter_DW.RateTransitionx_Buffer0;
    }

    /* End of RateTransition: '<S5>/Rate Transition: x' */

    /* DeadZone: '<S5>/Dead Zone: x' */
    if (simulink_kalmanfilter_B.RateTransitionx >
        simulink_kalmanfilter_P.DeadZonex_End) {
      smax = simulink_kalmanfilter_B.RateTransitionx -
        simulink_kalmanfilter_P.DeadZonex_End;
    } else if (simulink_kalmanfilter_B.RateTransitionx >=
               simulink_kalmanfilter_P.DeadZonex_Start) {
      smax = 0.0;
    } else {
      smax = simulink_kalmanfilter_B.RateTransitionx -
        simulink_kalmanfilter_P.DeadZonex_Start;
    }

    /* End of DeadZone: '<S5>/Dead Zone: x' */

    /* Gain: '<S5>/Joystick_gain_x' incorporates:
     *  Gain: '<S5>/Gain: x'
     */
    simulink_kalmanfilter_B.Joystick_gain_x = simulink_kalmanfilter_P.Gainx_Gain
      * smax * simulink_kalmanfilter_P.Joystick_gain_x;
    for (i = 0; i < 6; i++) {
      /* UnitDelay: '<Root>/Unit Delay' */
      simulink_kalmanfilter_B.x_bar[i] =
        simulink_kalmanfilter_DW.UnitDelay_DSTATE[i];
    }

    /* S-Function (stream_call_block): '<S12>/Stream Call1' incorporates:
     *  Constant: '<S12>/Constant'
     *  S-Function (string_constant_block): '<S12>/String Constant'
     */

    /* S-Function Block: simulink_kalmanfilter/IMU system/IMU/Stream Call1 (stream_call_block) */
    {
      t_error result = 0;
      t_boolean close_flag = (simulink_kalmanfilter_P.Constant_Value_m != 0);
      rtb_StreamCall1_o1 = NULL;
      switch (simulink_kalmanfilter_DW.StreamCall1_State) {
       case STREAM_CALL_STATE_NOT_CONNECTED:
        {
          if (!close_flag) {
            /* Make sure URI is null-terminated */
            if (string_length((char *)
                              simulink_kalmanfilter_P.StringConstant_Value, 255)
                == 255) {
              rtmSetErrorStatus(simulink_kalmanfilter_M,
                                "URI passed to Stream Call block is not null-terminated!");
              result = -QERR_STRING_NOT_TERMINATED;
            } else {
              result = stream_connect((char *)
                simulink_kalmanfilter_P.StringConstant_Value,
                simulink_kalmanfilter_P.StreamCall1_NonBlocking != 0,
                simulink_kalmanfilter_P.StreamCall1_SendBufferSize,
                simulink_kalmanfilter_P.StreamCall1_ReceiveBufferSize,
                &simulink_kalmanfilter_DW.StreamCall1_Stream);
              if (result == 0) {
                simulink_kalmanfilter_DW.StreamCall1_State =
                  STREAM_CALL_STATE_CONNECTED;
                stream_set_byte_order
                  (simulink_kalmanfilter_DW.StreamCall1_Stream,
                   (t_stream_byte_order)
                   simulink_kalmanfilter_P.StreamCall1_Endian);
                rtb_StreamCall1_o1 =
                  &simulink_kalmanfilter_DW.StreamCall1_Stream;
              } else if (result == -QERR_WOULD_BLOCK) {
                simulink_kalmanfilter_DW.StreamCall1_State =
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

            result = stream_poll(simulink_kalmanfilter_DW.StreamCall1_Stream,
                                 &timeout, STREAM_POLL_CONNECT);
            if (result > 0) {
              simulink_kalmanfilter_DW.StreamCall1_State =
                STREAM_CALL_STATE_CONNECTED;
              stream_set_byte_order(simulink_kalmanfilter_DW.StreamCall1_Stream,
                                    (t_stream_byte_order)
                                    simulink_kalmanfilter_P.StreamCall1_Endian);
              rtb_StreamCall1_o1 = &simulink_kalmanfilter_DW.StreamCall1_Stream;
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
          rtb_StreamCall1_o1 = &simulink_kalmanfilter_DW.StreamCall1_Stream;
          if (!close_flag) {
            break;
          }

          /* Fall through deliberately */
        }

       default:
        {
          t_error close_result = stream_close
            (simulink_kalmanfilter_DW.StreamCall1_Stream);
          if (close_result == 0) {
            simulink_kalmanfilter_DW.StreamCall1_State =
              STREAM_CALL_STATE_NOT_CONNECTED;
            simulink_kalmanfilter_DW.StreamCall1_Stream = NULL;
            rtb_StreamCall1_o1 = NULL;
          } else if (result == 0) {
            result = close_result;
          }
          break;
        }
      }

      simulink_kalmanfilter_B.StreamCall1_o2 =
        simulink_kalmanfilter_DW.StreamCall1_State;
      rtb_StreamCall1_o3 = (int32_T) result;
    }

    /* S-Function (stream_formatted_write_block): '<S12>/Stream Formatted Write' incorporates:
     *  Constant: '<S12>/Constant1'
     */
    {
      t_error result;
      if (rtb_StreamCall1_o1 != NULL) {
        result = stream_print_utf8_char_array(*rtb_StreamCall1_o1,
          simulink_kalmanfilter_P.StreamFormattedWrite_MaxUnits,
          &rtb_StreamFormattedWrite_o2, "%c\n"
          , (char) simulink_kalmanfilter_P.Constant1_Value
          );
        if (result > 0) {
          result = stream_flush(*rtb_StreamCall1_o1);
        }

        if (result == -QERR_WOULD_BLOCK) {
          result = 0;
        }
      }
    }

    /* S-Function (stream_read_block): '<S12>/Stream Read1' incorporates:
     *  Constant: '<S12>/Constant1'
     *  S-Function (stream_call_block): '<S12>/Stream Call1'
     *  S-Function (stream_formatted_write_block): '<S12>/Stream Formatted Write'
     */

    /* S-Function Block: simulink_kalmanfilter/IMU system/IMU/Stream Read1 (stream_read_block) */
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
      /* Switch: '<S12>/Switch' */
      if (rtb_StreamRead1_o3) {
        /* Switch: '<S12>/Switch' incorporates:
         *  DataTypeConversion: '<S12>/Data Type Conversion'
         */
        simulink_kalmanfilter_B.Switch[i] = rtb_StreamRead1_o2[i];
      } else {
        /* Switch: '<S12>/Switch' incorporates:
         *  Memory: '<S12>/Memory'
         */
        simulink_kalmanfilter_B.Switch[i] =
          simulink_kalmanfilter_DW.Memory_PreviousInput[i];
      }

      /* End of Switch: '<S12>/Switch' */
    }

    /* Gain: '<S12>/Gain2' */
    for (i = 0; i < 3; i++) {
      rtb_Gain2[i] = simulink_kalmanfilter_P.Gain2_Gain[i + 6] *
        simulink_kalmanfilter_B.Switch[2] +
        (simulink_kalmanfilter_P.Gain2_Gain[i + 3] *
         simulink_kalmanfilter_B.Switch[1] +
         simulink_kalmanfilter_P.Gain2_Gain[i] * simulink_kalmanfilter_B.Switch
         [0]);
    }

    /* End of Gain: '<S12>/Gain2' */

    /* MATLAB Function: '<S4>/MATLAB Function' */
    smax = rtb_Gain2[1];
    az = rtb_Gain2[2];

    /* MATLAB Function 'IMU system/MATLAB Function': '<S13>:1' */
    if (rtb_Gain2[2] == 0.0) {
      /* '<S13>:1:2' */
      /* '<S13>:1:3' */
      az = 0.01;
      if (rtb_Gain2[1] == 0.0) {
        /* '<S13>:1:4' */
        /* '<S13>:1:5' */
        smax = 0.01;
      }
    }

    /* Sum: '<S4>/Sum' incorporates:
     *  Constant: '<S4>/Constant'
     *  MATLAB Function: '<S4>/MATLAB Function'
     */
    /* '<S13>:1:8' */
    /* '<S13>:1:9' */
    simulink_kalmanfilter_B.p = atan(smax / az) +
      simulink_kalmanfilter_P.Constant_Value;
    for (i = 0; i < 3; i++) {
      /* Gain: '<S12>/Gain1' */
      simulink_kalmanfilter_B.Gain1[i] = 0.0;
      simulink_kalmanfilter_B.Gain1[i] += simulink_kalmanfilter_P.Gain1_Gain[i] *
        simulink_kalmanfilter_B.Switch[3];
      simulink_kalmanfilter_B.Gain1[i] += simulink_kalmanfilter_P.Gain1_Gain[i +
        3] * simulink_kalmanfilter_B.Switch[4];
      simulink_kalmanfilter_B.Gain1[i] += simulink_kalmanfilter_P.Gain1_Gain[i +
        6] * simulink_kalmanfilter_B.Switch[5];
    }

    /* Sum: '<S4>/Sum1' incorporates:
     *  Constant: '<S4>/Constant1'
     *  MATLAB Function: '<S4>/MATLAB Function'
     */
    simulink_kalmanfilter_B.e = atan(rtb_Gain2[0] / sqrt(smax * smax + az * az))
      + simulink_kalmanfilter_P.Constant1_Value_n;

    /* MATLAB Function: '<S4>/Gyro vector to [pitch rate, elevation rate, travle rate]' incorporates:
     *  SignalConversion generated from: '<S11>/ SFunction '
     */
    /* MATLAB Function 'IMU system/Gyro vector to [pitch rate, elevation rate, travle rate]': '<S11>:1' */
    /* '<S11>:1:3' */
    /* '<S11>:1:4' */
    /* '<S11>:1:8' */
    /* '<S11>:1:11' */
    smax = tan(simulink_kalmanfilter_B.e);
    az = sin(simulink_kalmanfilter_B.p);
    euler_rates_tmp = cos(simulink_kalmanfilter_B.p);
    euler_rates_tmp_0 = cos(simulink_kalmanfilter_B.e);
    tmp[0] = 1.0;
    tmp[3] = az * smax;
    tmp[6] = euler_rates_tmp * smax;
    tmp[1] = 0.0;
    tmp[4] = euler_rates_tmp;
    tmp[7] = -az;
    tmp[2] = 0.0;
    tmp[5] = az / euler_rates_tmp_0;
    tmp[8] = euler_rates_tmp / euler_rates_tmp_0;
    for (i = 0; i < 3; i++) {
      simulink_kalmanfilter_B.euler_rates[i] = 0.0;
      simulink_kalmanfilter_B.euler_rates[i] += tmp[i] *
        simulink_kalmanfilter_B.Gain1[0];
      simulink_kalmanfilter_B.euler_rates[i] += tmp[i + 3] *
        simulink_kalmanfilter_B.Gain1[1];
      simulink_kalmanfilter_B.euler_rates[i] += tmp[i + 6] *
        simulink_kalmanfilter_B.Gain1[2];
    }

    /* End of MATLAB Function: '<S4>/Gyro vector to [pitch rate, elevation rate, travle rate]' */

    /* MATLAB Function: '<Root>/correction' incorporates:
     *  Constant: '<Root>/Constant1'
     *  Constant: '<Root>/Constant2'
     *  UnitDelay: '<Root>/Unit Delay'
     *  UnitDelay: '<Root>/Unit Delay1'
     */
    /* MATLAB Function 'correction': '<S8>:1' */
    if (rtb_StreamRead1_o3) {
      /* '<S8>:1:2' */
      /* '<S8>:1:3' */
      for (i = 0; i < 25; i++) {
        b[i] = 0.0;
      }

      for (i = 0; i < 5; i++) {
        for (iy = 0; iy < 6; iy++) {
          ix = 5 * iy + i;
          A_tmp[iy + 6 * i] = simulink_kalmanfilter_P.Cd[ix];
          tmp_1[ix] = 0.0;
          for (A_tmp_0 = 0; A_tmp_0 < 6; A_tmp_0++) {
            tmp_1[ix] += simulink_kalmanfilter_P.Cd[5 * A_tmp_0 + i] *
              simulink_kalmanfilter_DW.UnitDelay1_DSTATE[6 * iy + A_tmp_0];
          }
        }
      }

      for (i = 0; i < 5; i++) {
        for (iy = 0; iy < 5; iy++) {
          smax = 0.0;
          for (A_tmp_0 = 0; A_tmp_0 < 6; A_tmp_0++) {
            smax += tmp_1[5 * A_tmp_0 + i] * A_tmp[6 * iy + A_tmp_0];
          }

          A_tmp_0 = 5 * iy + i;
          A[A_tmp_0] = simulink_kalmanfilter_P.Rd[A_tmp_0] + smax;
        }

        ipiv[i] = (int8_T)(i + 1);
      }

      for (A_tmp_0 = 0; A_tmp_0 < 4; A_tmp_0++) {
        i = A_tmp_0 * 6;
        iy = 0;
        ix = i;
        smax = fabs(A[i]);
        for (d_k = 2; d_k <= 5 - A_tmp_0; d_k++) {
          ix++;
          az = fabs(A[ix]);
          if (az > smax) {
            iy = d_k - 1;
            smax = az;
          }
        }

        if (A[i + iy] != 0.0) {
          if (iy != 0) {
            iy += A_tmp_0;
            ipiv[A_tmp_0] = (int8_T)(iy + 1);
            ix = A_tmp_0;
            for (d_k = 0; d_k < 5; d_k++) {
              smax = A[ix];
              A[ix] = A[iy];
              A[iy] = smax;
              ix += 5;
              iy += 5;
            }
          }

          iy = (i - A_tmp_0) + 5;
          for (ix = i + 1; ix < iy; ix++) {
            A[ix] /= A[i];
          }
        }

        iy = i;
        ix = i + 5;
        for (d_k = 0; d_k <= 3 - A_tmp_0; d_k++) {
          if (A[ix] != 0.0) {
            smax = -A[ix];
            c_ix = i + 1;
            e = (iy - A_tmp_0) + 10;
            for (ijA = iy + 6; ijA < e; ijA++) {
              A[ijA] += A[c_ix] * smax;
              c_ix++;
            }
          }

          ix += 5;
          iy += 5;
        }
      }

      for (i = 0; i < 5; i++) {
        p[i] = (int8_T)(i + 1);
      }

      if (ipiv[0] > 1) {
        i = p[ipiv[0] - 1];
        p[ipiv[0] - 1] = p[0];
        p[0] = (int8_T)i;
      }

      if (ipiv[1] > 2) {
        i = p[ipiv[1] - 1];
        p[ipiv[1] - 1] = p[1];
        p[1] = (int8_T)i;
      }

      if (ipiv[2] > 3) {
        i = p[ipiv[2] - 1];
        p[ipiv[2] - 1] = p[2];
        p[2] = (int8_T)i;
      }

      if (ipiv[3] > 4) {
        i = p[ipiv[3] - 1];
        p[ipiv[3] - 1] = p[3];
        p[3] = (int8_T)i;
      }

      for (A_tmp_0 = 0; A_tmp_0 < 5; A_tmp_0++) {
        d_k = p[A_tmp_0] - 1;
        b[A_tmp_0 + 5 * d_k] = 1.0;
        for (iy = A_tmp_0; iy + 1 < 6; iy++) {
          i = 5 * d_k + iy;
          if (b[i] != 0.0) {
            for (ix = iy + 1; ix + 1 < 6; ix++) {
              c_ix = 5 * d_k + ix;
              b[c_ix] -= b[i] * A[5 * iy + ix];
            }
          }
        }
      }

      for (A_tmp_0 = 0; A_tmp_0 < 5; A_tmp_0++) {
        i = 5 * A_tmp_0;
        for (iy = 4; iy >= 0; iy--) {
          ix = 5 * iy;
          if (b[iy + i] != 0.0) {
            c_ix = iy + i;
            b[c_ix] /= A[iy + ix];
            for (d_k = 0; d_k < iy; d_k++) {
              e = d_k + i;
              b[e] -= b[c_ix] * A[d_k + ix];
            }
          }
        }
      }

      for (i = 0; i < 6; i++) {
        for (iy = 0; iy < 5; iy++) {
          A_tmp_0 = i + 6 * iy;
          tmp_1[A_tmp_0] = 0.0;
          for (ix = 0; ix < 6; ix++) {
            tmp_1[A_tmp_0] += simulink_kalmanfilter_DW.UnitDelay1_DSTATE[6 * ix
              + i] * A_tmp[6 * iy + ix];
          }
        }

        for (iy = 0; iy < 5; iy++) {
          ix = i + 6 * iy;
          L[ix] = 0.0;
          for (A_tmp_0 = 0; A_tmp_0 < 5; A_tmp_0++) {
            L[ix] += tmp_1[6 * A_tmp_0 + i] * b[5 * iy + A_tmp_0];
          }
        }
      }

      /* SignalConversion generated from: '<S8>/ SFunction ' incorporates:
       *  Constant: '<Root>/Constant1'
       *  Constant: '<Root>/Constant2'
       *  UnitDelay: '<Root>/Unit Delay1'
       */
      /* '<S8>:1:4' */
      tmp_2[0] = simulink_kalmanfilter_B.p;
      tmp_2[1] = simulink_kalmanfilter_B.euler_rates[0];
      tmp_2[2] = simulink_kalmanfilter_B.e;
      tmp_2[3] = simulink_kalmanfilter_B.euler_rates[1];
      tmp_2[4] = simulink_kalmanfilter_B.euler_rates[2];
      for (i = 0; i < 5; i++) {
        tmp_3[i] = 0.0;
        for (iy = 0; iy < 6; iy++) {
          tmp_3[i] += simulink_kalmanfilter_P.Cd[5 * iy + i] *
            simulink_kalmanfilter_B.x_bar[iy];
        }

        tmp_4[i] = tmp_2[i] - tmp_3[i];
      }

      for (i = 0; i < 6; i++) {
        smax = 0.0;
        for (iy = 0; iy < 5; iy++) {
          smax += L[6 * iy + i] * tmp_4[iy];
        }

        simulink_kalmanfilter_B.x_hat[i] = simulink_kalmanfilter_B.x_bar[i] +
          smax;
      }

      for (i = 0; i < 36; i++) {
        b_I[i] = 0;
      }

      for (A_tmp_0 = 0; A_tmp_0 < 6; A_tmp_0++) {
        b_I[A_tmp_0 + 6 * A_tmp_0] = 1;
      }

      /* '<S8>:1:5' */
      for (i = 0; i < 36; i++) {
        c_I[i] = 0;
      }

      for (A_tmp_0 = 0; A_tmp_0 < 6; A_tmp_0++) {
        c_I[A_tmp_0 + 6 * A_tmp_0] = 1;
        for (i = 0; i < 6; i++) {
          ix = A_tmp_0 + 6 * i;
          P_hat_tmp[ix] = 0.0;
          for (iy = 0; iy < 5; iy++) {
            P_hat_tmp[ix] += L[6 * iy + A_tmp_0] * simulink_kalmanfilter_P.Cd[5 *
              i + iy];
          }
        }
      }

      for (i = 0; i < 36; i++) {
        c_I_0[i] = (real_T)c_I[i] - P_hat_tmp[i];
      }

      for (i = 0; i < 6; i++) {
        for (iy = 0; iy < 6; iy++) {
          ix = i + 6 * iy;
          c_I_1[ix] = 0.0;
          for (A_tmp_0 = 0; A_tmp_0 < 6; A_tmp_0++) {
            c_I_1[ix] += c_I_0[6 * A_tmp_0 + i] *
              simulink_kalmanfilter_DW.UnitDelay1_DSTATE[6 * iy + A_tmp_0];
          }

          b_I_0[iy + 6 * i] = (real_T)b_I[ix] - P_hat_tmp[ix];
        }

        for (iy = 0; iy < 5; iy++) {
          ix = i + 6 * iy;
          A_tmp[ix] = 0.0;
          for (A_tmp_0 = 0; A_tmp_0 < 5; A_tmp_0++) {
            A_tmp[ix] += L[6 * A_tmp_0 + i] * simulink_kalmanfilter_P.Rd[5 * iy
              + A_tmp_0];
          }
        }
      }

      for (i = 0; i < 6; i++) {
        for (iy = 0; iy < 6; iy++) {
          ix = i + 6 * iy;
          c_I_0[ix] = 0.0;
          for (A_tmp_0 = 0; A_tmp_0 < 6; A_tmp_0++) {
            c_I_0[ix] += c_I_1[6 * A_tmp_0 + i] * b_I_0[6 * iy + A_tmp_0];
          }

          P_hat_tmp[ix] = 0.0;
          for (A_tmp_0 = 0; A_tmp_0 < 5; A_tmp_0++) {
            P_hat_tmp[ix] += A_tmp[6 * A_tmp_0 + i] * L[6 * A_tmp_0 + iy];
          }
        }
      }

      for (i = 0; i < 36; i++) {
        simulink_kalmanfilter_B.P_hat[i] = c_I_0[i] + P_hat_tmp[i];
      }
    } else {
      /* '<S8>:1:7' */
      for (i = 0; i < 6; i++) {
        simulink_kalmanfilter_B.x_hat[i] = simulink_kalmanfilter_B.x_bar[i];
      }

      /* '<S8>:1:8' */
      memcpy(&simulink_kalmanfilter_B.P_hat[0],
             &simulink_kalmanfilter_DW.UnitDelay1_DSTATE[0], 36U * sizeof(real_T));
    }

    /* End of MATLAB Function: '<Root>/correction' */

    /* RateTransition: '<S5>/Rate Transition: y' */
    if (simulink_kalmanfilter_M->Timing.RateInteraction.TID1_2) {
      /* RateTransition: '<S5>/Rate Transition: y' */
      simulink_kalmanfilter_B.RateTransitiony =
        simulink_kalmanfilter_DW.RateTransitiony_Buffer0;
    }

    /* End of RateTransition: '<S5>/Rate Transition: y' */

    /* DeadZone: '<S5>/Dead Zone: y' */
    if (simulink_kalmanfilter_B.RateTransitiony >
        simulink_kalmanfilter_P.DeadZoney_End) {
      smax = simulink_kalmanfilter_B.RateTransitiony -
        simulink_kalmanfilter_P.DeadZoney_End;
    } else if (simulink_kalmanfilter_B.RateTransitiony >=
               simulink_kalmanfilter_P.DeadZoney_Start) {
      smax = 0.0;
    } else {
      smax = simulink_kalmanfilter_B.RateTransitiony -
        simulink_kalmanfilter_P.DeadZoney_Start;
    }

    /* End of DeadZone: '<S5>/Dead Zone: y' */

    /* Gain: '<S5>/Joystick_gain_y' incorporates:
     *  Gain: '<S5>/Gain: y'
     */
    simulink_kalmanfilter_B.Joystick_gain_y = simulink_kalmanfilter_P.Gainy_Gain
      * smax * simulink_kalmanfilter_P.Joystick_gain_y;

    /* SignalConversion generated from: '<S6>/Gain' incorporates:
     *  Sum: '<S6>/Sum3'
     *  Sum: '<S6>/Sum4'
     */
    smax = simulink_kalmanfilter_B.Joystick_gain_x -
      simulink_kalmanfilter_B.x_hat[0];
    az = simulink_kalmanfilter_B.Joystick_gain_y -
      simulink_kalmanfilter_B.x_hat[3];

    /* Gain: '<S6>/Gain' */
    simulink_kalmanfilter_B.Gain[0] = 0.0;
    simulink_kalmanfilter_B.Gain[0] += simulink_kalmanfilter_P.F[0] * smax;
    simulink_kalmanfilter_B.Gain[0] += simulink_kalmanfilter_P.F[2] * az;
    simulink_kalmanfilter_B.Gain[1] = 0.0;
    simulink_kalmanfilter_B.Gain[1] += simulink_kalmanfilter_P.F[1] * smax;
    simulink_kalmanfilter_B.Gain[1] += simulink_kalmanfilter_P.F[3] * az;
  }

  /* SignalConversion generated from: '<S6>/Gain3' incorporates:
   *  Integrator: '<S6>/Integrator'
   *  Integrator: '<S6>/Integrator1'
   */
  tmp_2[0] = simulink_kalmanfilter_B.x_hat[0];
  tmp_2[1] = simulink_kalmanfilter_B.x_hat[1];
  tmp_2[2] = simulink_kalmanfilter_B.x_hat[3];
  tmp_2[3] = simulink_kalmanfilter_X.Integrator_CSTATE;
  tmp_2[4] = simulink_kalmanfilter_X.Integrator1_CSTATE;
  for (i = 0; i < 2; i++) {
    /* Sum: '<S6>/Sum' incorporates:
     *  Gain: '<S6>/Gain3'
     */
    smax = 0.0;
    for (iy = 0; iy < 5; iy++) {
      smax += simulink_kalmanfilter_P.K_LQR[(iy << 1) + i] * tmp_2[iy];
    }

    simulink_kalmanfilter_B.Sum[i] = simulink_kalmanfilter_B.Gain[i] - smax;

    /* End of Sum: '<S6>/Sum' */
  }

  if (rtmIsMajorTimeStep(simulink_kalmanfilter_M)) {
    /* Gain: '<S3>/Travel: Count to rad' */
    simulink_kalmanfilter_B.TravelCounttorad =
      simulink_kalmanfilter_P.TravelCounttorad_Gain *
      rtb_HILReadEncoderTimebase_o1;

    /* Gain: '<S3>/Pitch: Count to rad' */
    simulink_kalmanfilter_B.PitchCounttorad =
      simulink_kalmanfilter_P.PitchCounttorad_Gain *
      rtb_HILReadEncoderTimebase_o2;
  }

  /* TransferFcn: '<S3>/Travel: Transfer Fcn' */
  simulink_kalmanfilter_B.TravelTransferFcn = 0.0;
  simulink_kalmanfilter_B.TravelTransferFcn +=
    simulink_kalmanfilter_P.TravelTransferFcn_C *
    simulink_kalmanfilter_X.TravelTransferFcn_CSTATE;
  simulink_kalmanfilter_B.TravelTransferFcn +=
    simulink_kalmanfilter_P.TravelTransferFcn_D *
    simulink_kalmanfilter_B.TravelCounttorad;

  /* TransferFcn: '<S3>/Pitch: Transfer Fcn' */
  simulink_kalmanfilter_B.PitchTransferFcn = 0.0;
  simulink_kalmanfilter_B.PitchTransferFcn +=
    simulink_kalmanfilter_P.PitchTransferFcn_C *
    simulink_kalmanfilter_X.PitchTransferFcn_CSTATE;
  simulink_kalmanfilter_B.PitchTransferFcn +=
    simulink_kalmanfilter_P.PitchTransferFcn_D *
    simulink_kalmanfilter_B.PitchCounttorad;
  if (rtmIsMajorTimeStep(simulink_kalmanfilter_M)) {
    /* Gain: '<S3>/Elevation: Count to rad' */
    simulink_kalmanfilter_B.ElevationCounttorad =
      simulink_kalmanfilter_P.ElevationCounttorad_Gain *
      rtb_HILReadEncoderTimebase_o3;

    /* Sum: '<S3>/Sum' incorporates:
     *  Constant: '<S3>/Constant'
     */
    simulink_kalmanfilter_B.Sum_m = simulink_kalmanfilter_B.ElevationCounttorad
      - simulink_kalmanfilter_P.Constant_Value_p;
  }

  /* TransferFcn: '<S3>/Elevation: Transfer Fcn' */
  simulink_kalmanfilter_B.ElevationTransferFcn = 0.0;
  simulink_kalmanfilter_B.ElevationTransferFcn +=
    simulink_kalmanfilter_P.ElevationTransferFcn_C *
    simulink_kalmanfilter_X.ElevationTransferFcn_CSTATE;
  simulink_kalmanfilter_B.ElevationTransferFcn +=
    simulink_kalmanfilter_P.ElevationTransferFcn_D *
    simulink_kalmanfilter_B.ElevationCounttorad;

  /* Sum: '<Root>/Sum' incorporates:
   *  Constant: '<Root>/Constant'
   */
  simulink_kalmanfilter_B.Sum_o = simulink_kalmanfilter_B.Sum[0] +
    simulink_kalmanfilter_P.Vs_0;
  if (rtmIsMajorTimeStep(simulink_kalmanfilter_M)) {
    /* SignalConversion generated from: '<Root>/To File3' */
    simulink_kalmanfilter_B.y_vector[0] = simulink_kalmanfilter_B.p;
    simulink_kalmanfilter_B.y_vector[1] = simulink_kalmanfilter_B.euler_rates[0];
    simulink_kalmanfilter_B.y_vector[2] = simulink_kalmanfilter_B.e;
    simulink_kalmanfilter_B.y_vector[3] = simulink_kalmanfilter_B.euler_rates[1];
    simulink_kalmanfilter_B.y_vector[4] = simulink_kalmanfilter_B.euler_rates[2];

    /* SignalConversion generated from: '<Root>/To File2' */
    rtb_TmpSignalConversionAtToFile[0] = simulink_kalmanfilter_B.p;
    rtb_TmpSignalConversionAtToFile[1] = simulink_kalmanfilter_B.euler_rates[0];
    rtb_TmpSignalConversionAtToFile[2] = simulink_kalmanfilter_B.e;
    rtb_TmpSignalConversionAtToFile[3] = simulink_kalmanfilter_B.euler_rates[1];
    rtb_TmpSignalConversionAtToFile[4] = simulink_kalmanfilter_B.euler_rates[2];
    for (i = 0; i < 6; i++) {
      rtb_TmpSignalConversionAtToFile[i + 5] = simulink_kalmanfilter_B.x_bar[i];
      rtb_TmpSignalConversionAtToFile[i + 11] = simulink_kalmanfilter_B.x_hat[i];
    }

    rtb_TmpSignalConversionAtToFile[17] =
      simulink_kalmanfilter_B.Joystick_gain_x;
    rtb_TmpSignalConversionAtToFile[18] =
      simulink_kalmanfilter_B.Joystick_gain_y;

    /* End of SignalConversion generated from: '<Root>/To File2' */

    /* ToFile: '<Root>/To File2' */
    if (rtmIsMajorTimeStep(simulink_kalmanfilter_M)) {
      if (rtmIsMajorTimeStep(simulink_kalmanfilter_M) ) {
        {
          if (!(++simulink_kalmanfilter_DW.ToFile2_IWORK.Decimation % 1) &&
              (simulink_kalmanfilter_DW.ToFile2_IWORK.Count * (19 + 1)) + 1 <
              100000000 ) {
            FILE *fp = (FILE *) simulink_kalmanfilter_DW.ToFile2_PWORK.FilePtr;
            if (fp != (NULL)) {
              real_T u[19 + 1];
              simulink_kalmanfilter_DW.ToFile2_IWORK.Decimation = 0;
              u[0] = simulink_kalmanfilter_M->Timing.t[1];
              u[1] = rtb_TmpSignalConversionAtToFile[0];
              u[2] = rtb_TmpSignalConversionAtToFile[1];
              u[3] = rtb_TmpSignalConversionAtToFile[2];
              u[4] = rtb_TmpSignalConversionAtToFile[3];
              u[5] = rtb_TmpSignalConversionAtToFile[4];
              u[6] = rtb_TmpSignalConversionAtToFile[5];
              u[7] = rtb_TmpSignalConversionAtToFile[6];
              u[8] = rtb_TmpSignalConversionAtToFile[7];
              u[9] = rtb_TmpSignalConversionAtToFile[8];
              u[10] = rtb_TmpSignalConversionAtToFile[9];
              u[11] = rtb_TmpSignalConversionAtToFile[10];
              u[12] = rtb_TmpSignalConversionAtToFile[11];
              u[13] = rtb_TmpSignalConversionAtToFile[12];
              u[14] = rtb_TmpSignalConversionAtToFile[13];
              u[15] = rtb_TmpSignalConversionAtToFile[14];
              u[16] = rtb_TmpSignalConversionAtToFile[15];
              u[17] = rtb_TmpSignalConversionAtToFile[16];
              u[18] = rtb_TmpSignalConversionAtToFile[17];
              u[19] = rtb_TmpSignalConversionAtToFile[18];
              if (fwrite(u, sizeof(real_T), 19 + 1, fp) != 19 + 1) {
                rtmSetErrorStatus(simulink_kalmanfilter_M,
                                  "Error writing to MAT-file lab4_Q_Rd_lower_lamda.mat");
                return;
              }

              if (((++simulink_kalmanfilter_DW.ToFile2_IWORK.Count) * (19 + 1))+
                  1 >= 100000000) {
                (void)fprintf(stdout,
                              "*** The ToFile block will stop logging data before\n"
                              "    the simulation has ended, because it has reached\n"
                              "    the maximum number of elements (100000000)\n"
                              "    allowed in MAT-file lab4_Q_Rd_lower_lamda.mat.\n");
              }
            }
          }
        }
      }
    }

    /* ToFile: '<Root>/To File3' */
    if (rtmIsMajorTimeStep(simulink_kalmanfilter_M)) {
      if (rtmIsMajorTimeStep(simulink_kalmanfilter_M) ) {
        {
          if (!(++simulink_kalmanfilter_DW.ToFile3_IWORK.Decimation % 1) &&
              (simulink_kalmanfilter_DW.ToFile3_IWORK.Count * (5 + 1)) + 1 <
              100000000 ) {
            FILE *fp = (FILE *) simulink_kalmanfilter_DW.ToFile3_PWORK.FilePtr;
            if (fp != (NULL)) {
              real_T u[5 + 1];
              simulink_kalmanfilter_DW.ToFile3_IWORK.Decimation = 0;
              u[0] = simulink_kalmanfilter_M->Timing.t[1];
              u[1] = simulink_kalmanfilter_B.y_vector[0];
              u[2] = simulink_kalmanfilter_B.y_vector[1];
              u[3] = simulink_kalmanfilter_B.y_vector[2];
              u[4] = simulink_kalmanfilter_B.y_vector[3];
              u[5] = simulink_kalmanfilter_B.y_vector[4];
              if (fwrite(u, sizeof(real_T), 5 + 1, fp) != 5 + 1) {
                rtmSetErrorStatus(simulink_kalmanfilter_M,
                                  "Error writing to MAT-file lab4_opg1_IMU_STATES_flying_50_sek.mat");
                return;
              }

              if (((++simulink_kalmanfilter_DW.ToFile3_IWORK.Count) * (5 + 1))+1
                  >= 100000000) {
                (void)fprintf(stdout,
                              "*** The ToFile block will stop logging data before\n"
                              "    the simulation has ended, because it has reached\n"
                              "    the maximum number of elements (100000000)\n"
                              "    allowed in MAT-file lab4_opg1_IMU_STATES_flying_50_sek.mat.\n");
              }
            }
          }
        }
      }
    }
  }

  /* Gain: '<S1>/Front gain' incorporates:
   *  Sum: '<S1>/Add'
   */
  smax = (simulink_kalmanfilter_B.Sum_o - simulink_kalmanfilter_B.Sum[1]) *
    simulink_kalmanfilter_P.Frontgain_Gain;

  /* Saturate: '<S3>/Front motor: Saturation' */
  if (smax > simulink_kalmanfilter_P.FrontmotorSaturation_UpperSat) {
    /* Saturate: '<S3>/Front motor: Saturation' */
    simulink_kalmanfilter_B.FrontmotorSaturation =
      simulink_kalmanfilter_P.FrontmotorSaturation_UpperSat;
  } else if (smax < simulink_kalmanfilter_P.FrontmotorSaturation_LowerSat) {
    /* Saturate: '<S3>/Front motor: Saturation' */
    simulink_kalmanfilter_B.FrontmotorSaturation =
      simulink_kalmanfilter_P.FrontmotorSaturation_LowerSat;
  } else {
    /* Saturate: '<S3>/Front motor: Saturation' */
    simulink_kalmanfilter_B.FrontmotorSaturation = smax;
  }

  /* End of Saturate: '<S3>/Front motor: Saturation' */

  /* Gain: '<S1>/Back gain' incorporates:
   *  Sum: '<S1>/Subtract'
   */
  smax = (simulink_kalmanfilter_B.Sum_o + simulink_kalmanfilter_B.Sum[1]) *
    simulink_kalmanfilter_P.Backgain_Gain;

  /* Saturate: '<S3>/Back motor: Saturation' */
  if (smax > simulink_kalmanfilter_P.BackmotorSaturation_UpperSat) {
    /* Saturate: '<S3>/Back motor: Saturation' */
    simulink_kalmanfilter_B.BackmotorSaturation =
      simulink_kalmanfilter_P.BackmotorSaturation_UpperSat;
  } else if (smax < simulink_kalmanfilter_P.BackmotorSaturation_LowerSat) {
    /* Saturate: '<S3>/Back motor: Saturation' */
    simulink_kalmanfilter_B.BackmotorSaturation =
      simulink_kalmanfilter_P.BackmotorSaturation_LowerSat;
  } else {
    /* Saturate: '<S3>/Back motor: Saturation' */
    simulink_kalmanfilter_B.BackmotorSaturation = smax;
  }

  /* End of Saturate: '<S3>/Back motor: Saturation' */
  if (rtmIsMajorTimeStep(simulink_kalmanfilter_M)) {
    /* S-Function (hil_write_analog_block): '<S3>/HIL Write Analog' */

    /* S-Function Block: simulink_kalmanfilter/Heli 3D/HIL Write Analog (hil_write_analog_block) */
    {
      t_error result;
      simulink_kalmanfilter_DW.HILWriteAnalog_Buffer[0] =
        simulink_kalmanfilter_B.FrontmotorSaturation;
      simulink_kalmanfilter_DW.HILWriteAnalog_Buffer[1] =
        simulink_kalmanfilter_B.BackmotorSaturation;
      result = hil_write_analog(simulink_kalmanfilter_DW.HILInitialize_Card,
        simulink_kalmanfilter_P.HILWriteAnalog_channels, 2,
        &simulink_kalmanfilter_DW.HILWriteAnalog_Buffer[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
      }
    }

    /* S-Function (stop_with_error_block): '<S12>/Stop with Call Error' */

    /* S-Function Block: simulink_kalmanfilter/IMU system/IMU/Stop with Call Error (stop_with_error_block) */
    {
      if (rtb_StreamCall1_o3 < 0) {
        msg_get_error_messageA(NULL, rtb_StreamCall1_o3, _rt_error_message,
          sizeof(_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }
    }

    /* Sum: '<S6>/Sum1' */
    simulink_kalmanfilter_B.Sum1 = simulink_kalmanfilter_B.Joystick_gain_x -
      simulink_kalmanfilter_B.x_hat[0];

    /* Sum: '<S6>/Sum2' */
    simulink_kalmanfilter_B.Sum2 = simulink_kalmanfilter_B.Joystick_gain_y -
      simulink_kalmanfilter_B.x_hat[3];
  }

  /* MATLAB Function: '<Root>/prediction' incorporates:
   *  Constant: '<Root>/Constant4'
   *  Constant: '<Root>/Constant5'
   *  Constant: '<Root>/Constant6'
   */
  /* MATLAB Function 'prediction': '<S9>:1' */
  /* '<S9>:1:2' */
  /* '<S9>:1:3' */
  for (i = 0; i < 6; i++) {
    tmp_0[i] = 0.0;
    for (iy = 0; iy < 6; iy++) {
      A_tmp_0 = 6 * iy + i;
      c_I_0[A_tmp_0] = 0.0;
      for (ix = 0; ix < 6; ix++) {
        c_I_0[A_tmp_0] += simulink_kalmanfilter_P.Ad[6 * ix + i] *
          simulink_kalmanfilter_B.P_hat[6 * iy + ix];
      }

      tmp_0[i] += simulink_kalmanfilter_P.Ad[A_tmp_0] *
        simulink_kalmanfilter_B.x_hat[iy];
    }

    simulink_kalmanfilter_B.x_bar_k[i] = tmp_0[i] +
      (simulink_kalmanfilter_P.Bd[i + 6] * simulink_kalmanfilter_B.Sum[1] +
       simulink_kalmanfilter_P.Bd[i] * simulink_kalmanfilter_B.Sum[0]);
    for (iy = 0; iy < 6; iy++) {
      smax = 0.0;
      for (A_tmp_0 = 0; A_tmp_0 < 6; A_tmp_0++) {
        smax += c_I_0[6 * A_tmp_0 + i] * simulink_kalmanfilter_P.Ad[6 * A_tmp_0
          + iy];
      }

      A_tmp_0 = 6 * iy + i;
      simulink_kalmanfilter_B.P_bar[A_tmp_0] =
        simulink_kalmanfilter_P.Qd[A_tmp_0] + smax;
    }
  }

  /* End of MATLAB Function: '<Root>/prediction' */

  /* Integrator: '<S10>/Integrator' */
  /* Limited  Integrator  */
  if (simulink_kalmanfilter_X.Integrator_CSTATE_n >=
      simulink_kalmanfilter_P.Integrator_UpperSat) {
    simulink_kalmanfilter_X.Integrator_CSTATE_n =
      simulink_kalmanfilter_P.Integrator_UpperSat;
  } else {
    if (simulink_kalmanfilter_X.Integrator_CSTATE_n <=
        simulink_kalmanfilter_P.Integrator_LowerSat) {
      simulink_kalmanfilter_X.Integrator_CSTATE_n =
        simulink_kalmanfilter_P.Integrator_LowerSat;
    }
  }

  /* End of Integrator: '<S10>/Integrator' */
  if (rtmIsMajorTimeStep(simulink_kalmanfilter_M)) {
    /* Gain: '<S10>/K_ei' incorporates:
     *  Sum: '<S2>/Sum'
     */
    simulink_kalmanfilter_B.K_ei = simulink_kalmanfilter_P.K_ei_Gain * 0.0;
  }
}

/* Model update function for TID0 */
void simulink_kalmanfilter_update0(void) /* Sample time: [0.0s, 0.0s] */
{
  int32_T i;
  if (rtmIsMajorTimeStep(simulink_kalmanfilter_M)) {
    /* Update for UnitDelay: '<Root>/Unit Delay' */
    for (i = 0; i < 6; i++) {
      simulink_kalmanfilter_DW.UnitDelay_DSTATE[i] =
        simulink_kalmanfilter_B.x_bar_k[i];
    }

    /* End of Update for UnitDelay: '<Root>/Unit Delay' */

    /* Update for UnitDelay: '<Root>/Unit Delay1' */
    memcpy(&simulink_kalmanfilter_DW.UnitDelay1_DSTATE[0],
           &simulink_kalmanfilter_B.P_bar[0], 36U * sizeof(real_T));

    /* Update for Memory: '<S12>/Memory' */
    memcpy(&simulink_kalmanfilter_DW.Memory_PreviousInput[0],
           &simulink_kalmanfilter_B.Switch[0], 10U * sizeof(real_T));
  }

  if (rtmIsMajorTimeStep(simulink_kalmanfilter_M)) {
    rt_ertODEUpdateContinuousStates(&simulink_kalmanfilter_M->solverInfo);
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
  if (!(++simulink_kalmanfilter_M->Timing.clockTick0)) {
    ++simulink_kalmanfilter_M->Timing.clockTickH0;
  }

  simulink_kalmanfilter_M->Timing.t[0] = rtsiGetSolverStopTime
    (&simulink_kalmanfilter_M->solverInfo);

  /* Update absolute time */
  /* The "clockTick1" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick1"
   * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick1 and the high bits
   * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++simulink_kalmanfilter_M->Timing.clockTick1)) {
    ++simulink_kalmanfilter_M->Timing.clockTickH1;
  }

  simulink_kalmanfilter_M->Timing.t[1] =
    simulink_kalmanfilter_M->Timing.clockTick1 *
    simulink_kalmanfilter_M->Timing.stepSize1 +
    simulink_kalmanfilter_M->Timing.clockTickH1 *
    simulink_kalmanfilter_M->Timing.stepSize1 * 4294967296.0;
}

/* Derivatives for root system: '<Root>' */
void simulink_kalmanfilter_derivatives(void)
{
  XDot_simulink_kalmanfilter_T *_rtXdot;
  boolean_T lsat;
  boolean_T usat;
  _rtXdot = ((XDot_simulink_kalmanfilter_T *) simulink_kalmanfilter_M->derivs);

  /* Derivatives for Integrator: '<S6>/Integrator' */
  _rtXdot->Integrator_CSTATE = simulink_kalmanfilter_B.Sum1;

  /* Derivatives for Integrator: '<S6>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = simulink_kalmanfilter_B.Sum2;

  /* Derivatives for TransferFcn: '<S3>/Travel: Transfer Fcn' */
  _rtXdot->TravelTransferFcn_CSTATE = 0.0;
  _rtXdot->TravelTransferFcn_CSTATE +=
    simulink_kalmanfilter_P.TravelTransferFcn_A *
    simulink_kalmanfilter_X.TravelTransferFcn_CSTATE;
  _rtXdot->TravelTransferFcn_CSTATE += simulink_kalmanfilter_B.TravelCounttorad;

  /* Derivatives for TransferFcn: '<S3>/Pitch: Transfer Fcn' */
  _rtXdot->PitchTransferFcn_CSTATE = 0.0;
  _rtXdot->PitchTransferFcn_CSTATE += simulink_kalmanfilter_P.PitchTransferFcn_A
    * simulink_kalmanfilter_X.PitchTransferFcn_CSTATE;
  _rtXdot->PitchTransferFcn_CSTATE += simulink_kalmanfilter_B.PitchCounttorad;

  /* Derivatives for TransferFcn: '<S3>/Elevation: Transfer Fcn' */
  _rtXdot->ElevationTransferFcn_CSTATE = 0.0;
  _rtXdot->ElevationTransferFcn_CSTATE +=
    simulink_kalmanfilter_P.ElevationTransferFcn_A *
    simulink_kalmanfilter_X.ElevationTransferFcn_CSTATE;
  _rtXdot->ElevationTransferFcn_CSTATE +=
    simulink_kalmanfilter_B.ElevationCounttorad;

  /* Derivatives for Integrator: '<S10>/Integrator' */
  lsat = (simulink_kalmanfilter_X.Integrator_CSTATE_n <=
          simulink_kalmanfilter_P.Integrator_LowerSat);
  usat = (simulink_kalmanfilter_X.Integrator_CSTATE_n >=
          simulink_kalmanfilter_P.Integrator_UpperSat);
  if (((!lsat) && (!usat)) || (lsat && (simulink_kalmanfilter_B.K_ei > 0.0)) ||
      (usat && (simulink_kalmanfilter_B.K_ei < 0.0))) {
    _rtXdot->Integrator_CSTATE_n = simulink_kalmanfilter_B.K_ei;
  } else {
    /* in saturation */
    _rtXdot->Integrator_CSTATE_n = 0.0;
  }

  /* End of Derivatives for Integrator: '<S10>/Integrator' */
}

/* Model output function for TID2 */
void simulink_kalmanfilter_output2(void) /* Sample time: [0.01s, 0.0s] */
{
  /* local block i/o variables */
  real_T rtb_GameController_o4;
  real_T rtb_GameController_o5;

  /* S-Function (game_controller_block): '<S5>/Game Controller' */

  /* S-Function Block: simulink_kalmanfilter/Joystick/Game Controller (game_controller_block) */
  {
    if (simulink_kalmanfilter_P.GameController_Enabled) {
      t_game_controller_states state;
      t_boolean new_data;
      t_error result;
      result = game_controller_poll
        (simulink_kalmanfilter_DW.GameController_Controller, &state, &new_data);
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

  /* RateTransition: '<S5>/Rate Transition: x' */
  simulink_kalmanfilter_DW.RateTransitionx_Buffer0 = rtb_GameController_o4;

  /* RateTransition: '<S5>/Rate Transition: y' */
  simulink_kalmanfilter_DW.RateTransitiony_Buffer0 = rtb_GameController_o5;
}

/* Model update function for TID2 */
void simulink_kalmanfilter_update2(void) /* Sample time: [0.01s, 0.0s] */
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
  if (!(++simulink_kalmanfilter_M->Timing.clockTick2)) {
    ++simulink_kalmanfilter_M->Timing.clockTickH2;
  }

  simulink_kalmanfilter_M->Timing.t[2] =
    simulink_kalmanfilter_M->Timing.clockTick2 *
    simulink_kalmanfilter_M->Timing.stepSize2 +
    simulink_kalmanfilter_M->Timing.clockTickH2 *
    simulink_kalmanfilter_M->Timing.stepSize2 * 4294967296.0;
}

/* Model output wrapper function for compatibility with a static main program */
void simulink_kalmanfilter_output(int_T tid)
{
  switch (tid) {
   case 0 :
    simulink_kalmanfilter_output0();
    break;

   case 2 :
    simulink_kalmanfilter_output2();
    break;

   default :
    break;
  }
}

/* Model update wrapper function for compatibility with a static main program */
void simulink_kalmanfilter_update(int_T tid)
{
  switch (tid) {
   case 0 :
    simulink_kalmanfilter_update0();
    break;

   case 2 :
    simulink_kalmanfilter_update2();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void simulink_kalmanfilter_initialize(void)
{
  /* Start for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: simulink_kalmanfilter/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("q8_usb", "0",
                      &simulink_kalmanfilter_DW.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
      return;
    }

    is_switching = false;
    result = hil_set_card_specific_options
      (simulink_kalmanfilter_DW.HILInitialize_Card,
       "update_rate=normal;decimation=1", 32);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
      return;
    }

    result = hil_watchdog_clear(simulink_kalmanfilter_DW.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
      return;
    }

    if ((simulink_kalmanfilter_P.HILInitialize_AIPStart && !is_switching) ||
        (simulink_kalmanfilter_P.HILInitialize_AIPEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AIMinimums =
          &simulink_kalmanfilter_DW.HILInitialize_AIMinimums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AIMinimums[i1] = (simulink_kalmanfilter_P.HILInitialize_AILow);
        }
      }

      {
        int_T i1;
        real_T *dw_AIMaximums =
          &simulink_kalmanfilter_DW.HILInitialize_AIMaximums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AIMaximums[i1] = simulink_kalmanfilter_P.HILInitialize_AIHigh;
        }
      }

      result = hil_set_analog_input_ranges
        (simulink_kalmanfilter_DW.HILInitialize_Card,
         simulink_kalmanfilter_P.HILInitialize_AIChannels, 8U,
         &simulink_kalmanfilter_DW.HILInitialize_AIMinimums[0],
         &simulink_kalmanfilter_DW.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_kalmanfilter_P.HILInitialize_AOPStart && !is_switching) ||
        (simulink_kalmanfilter_P.HILInitialize_AOPEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOMinimums =
          &simulink_kalmanfilter_DW.HILInitialize_AOMinimums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOMinimums[i1] = (simulink_kalmanfilter_P.HILInitialize_AOLow);
        }
      }

      {
        int_T i1;
        real_T *dw_AOMaximums =
          &simulink_kalmanfilter_DW.HILInitialize_AOMaximums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOMaximums[i1] = simulink_kalmanfilter_P.HILInitialize_AOHigh;
        }
      }

      result = hil_set_analog_output_ranges
        (simulink_kalmanfilter_DW.HILInitialize_Card,
         simulink_kalmanfilter_P.HILInitialize_AOChannels, 8U,
         &simulink_kalmanfilter_DW.HILInitialize_AOMinimums[0],
         &simulink_kalmanfilter_DW.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_kalmanfilter_P.HILInitialize_AOStart && !is_switching) ||
        (simulink_kalmanfilter_P.HILInitialize_AOEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOVoltages =
          &simulink_kalmanfilter_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = simulink_kalmanfilter_P.HILInitialize_AOInitial;
        }
      }

      result = hil_write_analog(simulink_kalmanfilter_DW.HILInitialize_Card,
        simulink_kalmanfilter_P.HILInitialize_AOChannels, 8U,
        &simulink_kalmanfilter_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }
    }

    if (simulink_kalmanfilter_P.HILInitialize_AOReset) {
      {
        int_T i1;
        real_T *dw_AOVoltages =
          &simulink_kalmanfilter_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = simulink_kalmanfilter_P.HILInitialize_AOWatchdog;
        }
      }

      result = hil_watchdog_set_analog_expiration_state
        (simulink_kalmanfilter_DW.HILInitialize_Card,
         simulink_kalmanfilter_P.HILInitialize_AOChannels, 8U,
         &simulink_kalmanfilter_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_kalmanfilter_P.HILInitialize_EIPStart && !is_switching) ||
        (simulink_kalmanfilter_P.HILInitialize_EIPEnter && is_switching)) {
      {
        int_T i1;
        int32_T *dw_QuadratureModes =
          &simulink_kalmanfilter_DW.HILInitialize_QuadratureModes[0];
        for (i1=0; i1 < 8; i1++) {
          dw_QuadratureModes[i1] =
            simulink_kalmanfilter_P.HILInitialize_EIQuadrature;
        }
      }

      result = hil_set_encoder_quadrature_mode
        (simulink_kalmanfilter_DW.HILInitialize_Card,
         simulink_kalmanfilter_P.HILInitialize_EIChannels, 8U,
         (t_encoder_quadrature_mode *)
         &simulink_kalmanfilter_DW.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_kalmanfilter_P.HILInitialize_EIStart && !is_switching) ||
        (simulink_kalmanfilter_P.HILInitialize_EIEnter && is_switching)) {
      {
        int_T i1;
        int32_T *dw_InitialEICounts =
          &simulink_kalmanfilter_DW.HILInitialize_InitialEICounts[0];
        for (i1=0; i1 < 8; i1++) {
          dw_InitialEICounts[i1] =
            simulink_kalmanfilter_P.HILInitialize_EIInitial;
        }
      }

      result = hil_set_encoder_counts
        (simulink_kalmanfilter_DW.HILInitialize_Card,
         simulink_kalmanfilter_P.HILInitialize_EIChannels, 8U,
         &simulink_kalmanfilter_DW.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_kalmanfilter_P.HILInitialize_POPStart && !is_switching) ||
        (simulink_kalmanfilter_P.HILInitialize_POPEnter && is_switching)) {
      uint32_T num_duty_cycle_modes = 0;
      uint32_T num_frequency_modes = 0;

      {
        int_T i1;
        int32_T *dw_POModeValues =
          &simulink_kalmanfilter_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POModeValues[i1] = simulink_kalmanfilter_P.HILInitialize_POModes;
        }
      }

      result = hil_set_pwm_mode(simulink_kalmanfilter_DW.HILInitialize_Card,
        simulink_kalmanfilter_P.HILInitialize_POChannels, 8U, (t_pwm_mode *)
        &simulink_kalmanfilter_DW.HILInitialize_POModeValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }

      {
        int_T i1;
        const uint32_T *p_HILInitialize_POChannels =
          simulink_kalmanfilter_P.HILInitialize_POChannels;
        int32_T *dw_POModeValues =
          &simulink_kalmanfilter_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          if (dw_POModeValues[i1] == PWM_DUTY_CYCLE_MODE || dw_POModeValues[i1] ==
              PWM_ONE_SHOT_MODE || dw_POModeValues[i1] == PWM_TIME_MODE ||
              dw_POModeValues[i1] == PWM_RAW_MODE) {
            simulink_kalmanfilter_DW.HILInitialize_POSortedChans[num_duty_cycle_modes]
              = (p_HILInitialize_POChannels[i1]);
            simulink_kalmanfilter_DW.HILInitialize_POSortedFreqs[num_duty_cycle_modes]
              = simulink_kalmanfilter_P.HILInitialize_POFrequency;
            num_duty_cycle_modes++;
          } else {
            simulink_kalmanfilter_DW.HILInitialize_POSortedChans[7U -
              num_frequency_modes] = (p_HILInitialize_POChannels[i1]);
            simulink_kalmanfilter_DW.HILInitialize_POSortedFreqs[7U -
              num_frequency_modes] =
              simulink_kalmanfilter_P.HILInitialize_POFrequency;
            num_frequency_modes++;
          }
        }
      }

      if (num_duty_cycle_modes > 0) {
        result = hil_set_pwm_frequency
          (simulink_kalmanfilter_DW.HILInitialize_Card,
           &simulink_kalmanfilter_DW.HILInitialize_POSortedChans[0],
           num_duty_cycle_modes,
           &simulink_kalmanfilter_DW.HILInitialize_POSortedFreqs[0]);
        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
          return;
        }
      }

      if (num_frequency_modes > 0) {
        result = hil_set_pwm_duty_cycle
          (simulink_kalmanfilter_DW.HILInitialize_Card,
           &simulink_kalmanfilter_DW.HILInitialize_POSortedChans[num_duty_cycle_modes],
           num_frequency_modes,
           &simulink_kalmanfilter_DW.HILInitialize_POSortedFreqs[num_duty_cycle_modes]);
        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
          return;
        }
      }

      {
        int_T i1;
        int32_T *dw_POModeValues =
          &simulink_kalmanfilter_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POModeValues[i1] =
            simulink_kalmanfilter_P.HILInitialize_POConfiguration;
        }
      }

      {
        int_T i1;
        int32_T *dw_POAlignValues =
          &simulink_kalmanfilter_DW.HILInitialize_POAlignValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POAlignValues[i1] =
            simulink_kalmanfilter_P.HILInitialize_POAlignment;
        }
      }

      {
        int_T i1;
        int32_T *dw_POPolarityVals =
          &simulink_kalmanfilter_DW.HILInitialize_POPolarityVals[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POPolarityVals[i1] =
            simulink_kalmanfilter_P.HILInitialize_POPolarity;
        }
      }

      result = hil_set_pwm_configuration
        (simulink_kalmanfilter_DW.HILInitialize_Card,
         simulink_kalmanfilter_P.HILInitialize_POChannels, 8U,
         (t_pwm_configuration *)
         &simulink_kalmanfilter_DW.HILInitialize_POModeValues[0],
         (t_pwm_alignment *)
         &simulink_kalmanfilter_DW.HILInitialize_POAlignValues[0],
         (t_pwm_polarity *)
         &simulink_kalmanfilter_DW.HILInitialize_POPolarityVals[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }

      {
        int_T i1;
        real_T *dw_POSortedFreqs =
          &simulink_kalmanfilter_DW.HILInitialize_POSortedFreqs[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POSortedFreqs[i1] = simulink_kalmanfilter_P.HILInitialize_POLeading;
        }
      }

      {
        int_T i1;
        real_T *dw_POValues = &simulink_kalmanfilter_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_kalmanfilter_P.HILInitialize_POTrailing;
        }
      }

      result = hil_set_pwm_deadband(simulink_kalmanfilter_DW.HILInitialize_Card,
        simulink_kalmanfilter_P.HILInitialize_POChannels, 8U,
        &simulink_kalmanfilter_DW.HILInitialize_POSortedFreqs[0],
        &simulink_kalmanfilter_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }
    }

    if ((simulink_kalmanfilter_P.HILInitialize_POStart && !is_switching) ||
        (simulink_kalmanfilter_P.HILInitialize_POEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_POValues = &simulink_kalmanfilter_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_kalmanfilter_P.HILInitialize_POInitial;
        }
      }

      result = hil_write_pwm(simulink_kalmanfilter_DW.HILInitialize_Card,
        simulink_kalmanfilter_P.HILInitialize_POChannels, 8U,
        &simulink_kalmanfilter_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }
    }

    if (simulink_kalmanfilter_P.HILInitialize_POReset) {
      {
        int_T i1;
        real_T *dw_POValues = &simulink_kalmanfilter_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_kalmanfilter_P.HILInitialize_POWatchdog;
        }
      }

      result = hil_watchdog_set_pwm_expiration_state
        (simulink_kalmanfilter_DW.HILInitialize_Card,
         simulink_kalmanfilter_P.HILInitialize_POChannels, 8U,
         &simulink_kalmanfilter_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for S-Function (hil_read_encoder_timebase_block): '<S3>/HIL Read Encoder Timebase' */

  /* S-Function Block: simulink_kalmanfilter/Heli 3D/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
  {
    t_error result;
    result = hil_task_create_encoder_reader
      (simulink_kalmanfilter_DW.HILInitialize_Card,
       simulink_kalmanfilter_P.HILReadEncoderTimebase_SamplesI,
       simulink_kalmanfilter_P.HILReadEncoderTimebase_Channels, 3,
       &simulink_kalmanfilter_DW.HILReadEncoderTimebase_Task);
    if (result >= 0) {
      result = hil_task_set_buffer_overflow_mode
        (simulink_kalmanfilter_DW.HILReadEncoderTimebase_Task,
         (t_buffer_overflow_mode)
         (simulink_kalmanfilter_P.HILReadEncoderTimebase_Overflow - 1));
    }

    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
    }
  }

  /* Start for RateTransition: '<S5>/Rate Transition: x' */
  simulink_kalmanfilter_B.RateTransitionx =
    simulink_kalmanfilter_P.RateTransitionx_InitialConditio;

  /* Start for S-Function (stream_call_block): '<S12>/Stream Call1' incorporates:
   *  Constant: '<S12>/Constant'
   *  S-Function (string_constant_block): '<S12>/String Constant'
   */

  /* S-Function Block: simulink_kalmanfilter/IMU system/IMU/Stream Call1 (stream_call_block) */
  {
    simulink_kalmanfilter_DW.StreamCall1_State = STREAM_CALL_STATE_NOT_CONNECTED;
    simulink_kalmanfilter_DW.StreamCall1_Stream = NULL;
  }

  /* Start for RateTransition: '<S5>/Rate Transition: y' */
  simulink_kalmanfilter_B.RateTransitiony =
    simulink_kalmanfilter_P.RateTransitiony_InitialConditio;

  /* Start for ToFile: '<Root>/To File2' */
  {
    FILE *fp = (NULL);
    char fileName[509] = "lab4_Q_Rd_lower_lamda.mat";
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(simulink_kalmanfilter_M,
                        "Error creating .mat file lab4_Q_Rd_lower_lamda.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp, 19 + 1, 0, "ans")) {
      rtmSetErrorStatus(simulink_kalmanfilter_M,
                        "Error writing mat file header to file lab4_Q_Rd_lower_lamda.mat");
      return;
    }

    simulink_kalmanfilter_DW.ToFile2_IWORK.Count = 0;
    simulink_kalmanfilter_DW.ToFile2_IWORK.Decimation = -1;
    simulink_kalmanfilter_DW.ToFile2_PWORK.FilePtr = fp;
  }

  /* Start for ToFile: '<Root>/To File3' */
  {
    FILE *fp = (NULL);
    char fileName[509] = "lab4_opg1_IMU_STATES_flying_50_sek.mat";
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(simulink_kalmanfilter_M,
                        "Error creating .mat file lab4_opg1_IMU_STATES_flying_50_sek.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp, 5 + 1, 0, "ans")) {
      rtmSetErrorStatus(simulink_kalmanfilter_M,
                        "Error writing mat file header to file lab4_opg1_IMU_STATES_flying_50_sek.mat");
      return;
    }

    simulink_kalmanfilter_DW.ToFile3_IWORK.Count = 0;
    simulink_kalmanfilter_DW.ToFile3_IWORK.Decimation = -1;
    simulink_kalmanfilter_DW.ToFile3_PWORK.FilePtr = fp;
  }

  /* Start for S-Function (game_controller_block): '<S5>/Game Controller' */

  /* S-Function Block: simulink_kalmanfilter/Joystick/Game Controller (game_controller_block) */
  {
    if (simulink_kalmanfilter_P.GameController_Enabled) {
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
        (simulink_kalmanfilter_P.GameController_ControllerNumber,
         simulink_kalmanfilter_P.GameController_BufferSize, deadzone, saturation,
         simulink_kalmanfilter_P.GameController_AutoCenter, 0, 1.0,
         &simulink_kalmanfilter_DW.GameController_Controller);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
      }
    }
  }

  {
    int32_T i;

    /* InitializeConditions for RateTransition: '<S5>/Rate Transition: x' */
    simulink_kalmanfilter_DW.RateTransitionx_Buffer0 =
      simulink_kalmanfilter_P.RateTransitionx_InitialConditio;

    /* InitializeConditions for UnitDelay: '<Root>/Unit Delay' */
    for (i = 0; i < 6; i++) {
      simulink_kalmanfilter_DW.UnitDelay_DSTATE[i] =
        simulink_kalmanfilter_P.UnitDelay_InitialCondition;
    }

    /* End of InitializeConditions for UnitDelay: '<Root>/Unit Delay' */

    /* InitializeConditions for UnitDelay: '<Root>/Unit Delay1' */
    for (i = 0; i < 36; i++) {
      simulink_kalmanfilter_DW.UnitDelay1_DSTATE[i] =
        simulink_kalmanfilter_P.UnitDelay1_InitialCondition;
    }

    /* End of InitializeConditions for UnitDelay: '<Root>/Unit Delay1' */

    /* InitializeConditions for Memory: '<S12>/Memory' */
    memcpy(&simulink_kalmanfilter_DW.Memory_PreviousInput[0],
           &simulink_kalmanfilter_P.Memory_InitialCondition[0], 10U * sizeof
           (real_T));

    /* InitializeConditions for RateTransition: '<S5>/Rate Transition: y' */
    simulink_kalmanfilter_DW.RateTransitiony_Buffer0 =
      simulink_kalmanfilter_P.RateTransitiony_InitialConditio;

    /* InitializeConditions for Integrator: '<S6>/Integrator' */
    simulink_kalmanfilter_X.Integrator_CSTATE =
      simulink_kalmanfilter_P.Integrator_IC;

    /* InitializeConditions for Integrator: '<S6>/Integrator1' */
    simulink_kalmanfilter_X.Integrator1_CSTATE =
      simulink_kalmanfilter_P.Integrator1_IC;

    /* InitializeConditions for TransferFcn: '<S3>/Travel: Transfer Fcn' */
    simulink_kalmanfilter_X.TravelTransferFcn_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S3>/Pitch: Transfer Fcn' */
    simulink_kalmanfilter_X.PitchTransferFcn_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S3>/Elevation: Transfer Fcn' */
    simulink_kalmanfilter_X.ElevationTransferFcn_CSTATE = 0.0;

    /* InitializeConditions for Integrator: '<S10>/Integrator' */
    simulink_kalmanfilter_X.Integrator_CSTATE_n =
      simulink_kalmanfilter_P.Integrator_IC_k;
  }
}

/* Model terminate function */
void simulink_kalmanfilter_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: simulink_kalmanfilter/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    t_uint32 num_final_analog_outputs = 0;
    t_uint32 num_final_pwm_outputs = 0;
    hil_task_stop_all(simulink_kalmanfilter_DW.HILInitialize_Card);
    hil_monitor_stop_all(simulink_kalmanfilter_DW.HILInitialize_Card);
    is_switching = false;
    if ((simulink_kalmanfilter_P.HILInitialize_AOTerminate && !is_switching) ||
        (simulink_kalmanfilter_P.HILInitialize_AOExit && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOVoltages =
          &simulink_kalmanfilter_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = simulink_kalmanfilter_P.HILInitialize_AOFinal;
        }
      }

      num_final_analog_outputs = 8U;
    } else {
      num_final_analog_outputs = 0;
    }

    if ((simulink_kalmanfilter_P.HILInitialize_POTerminate && !is_switching) ||
        (simulink_kalmanfilter_P.HILInitialize_POExit && is_switching)) {
      {
        int_T i1;
        real_T *dw_POValues = &simulink_kalmanfilter_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = simulink_kalmanfilter_P.HILInitialize_POFinal;
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
      result = hil_write(simulink_kalmanfilter_DW.HILInitialize_Card
                         , simulink_kalmanfilter_P.HILInitialize_AOChannels,
                         num_final_analog_outputs
                         , simulink_kalmanfilter_P.HILInitialize_POChannels,
                         num_final_pwm_outputs
                         , NULL, 0
                         , NULL, 0
                         , &simulink_kalmanfilter_DW.HILInitialize_AOVoltages[0]
                         , &simulink_kalmanfilter_DW.HILInitialize_POValues[0]
                         , (t_boolean *) NULL
                         , NULL
                         );
      if (result == -QERR_HIL_WRITE_NOT_SUPPORTED) {
        t_error local_result;
        result = 0;

        /* The hil_write operation is not supported by this card. Write final outputs for each channel type */
        if (num_final_analog_outputs > 0) {
          local_result = hil_write_analog
            (simulink_kalmanfilter_DW.HILInitialize_Card,
             simulink_kalmanfilter_P.HILInitialize_AOChannels,
             num_final_analog_outputs,
             &simulink_kalmanfilter_DW.HILInitialize_AOVoltages[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_pwm_outputs > 0) {
          local_result = hil_write_pwm
            (simulink_kalmanfilter_DW.HILInitialize_Card,
             simulink_kalmanfilter_P.HILInitialize_POChannels,
             num_final_pwm_outputs,
             &simulink_kalmanfilter_DW.HILInitialize_POValues[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(simulink_kalmanfilter_M, _rt_error_message);
        }
      }
    }

    hil_task_delete_all(simulink_kalmanfilter_DW.HILInitialize_Card);
    hil_monitor_delete_all(simulink_kalmanfilter_DW.HILInitialize_Card);
    hil_close(simulink_kalmanfilter_DW.HILInitialize_Card);
    simulink_kalmanfilter_DW.HILInitialize_Card = NULL;
  }

  /* Terminate for S-Function (stream_call_block): '<S12>/Stream Call1' incorporates:
   *  Constant: '<S12>/Constant'
   *  S-Function (string_constant_block): '<S12>/String Constant'
   */

  /* S-Function Block: simulink_kalmanfilter/IMU system/IMU/Stream Call1 (stream_call_block) */
  {
    if (simulink_kalmanfilter_DW.StreamCall1_Stream != NULL) {
      stream_close(simulink_kalmanfilter_DW.StreamCall1_Stream);
      simulink_kalmanfilter_DW.StreamCall1_Stream = NULL;
    }
  }

  /* Terminate for ToFile: '<Root>/To File2' */
  {
    FILE *fp = (FILE *) simulink_kalmanfilter_DW.ToFile2_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "lab4_Q_Rd_lower_lamda.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(simulink_kalmanfilter_M,
                          "Error closing MAT-file lab4_Q_Rd_lower_lamda.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(simulink_kalmanfilter_M,
                          "Error reopening MAT-file lab4_Q_Rd_lower_lamda.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 19 + 1,
           simulink_kalmanfilter_DW.ToFile2_IWORK.Count, "ans")) {
        rtmSetErrorStatus(simulink_kalmanfilter_M,
                          "Error writing header for ans to MAT-file lab4_Q_Rd_lower_lamda.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(simulink_kalmanfilter_M,
                          "Error closing MAT-file lab4_Q_Rd_lower_lamda.mat");
        return;
      }

      simulink_kalmanfilter_DW.ToFile2_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File3' */
  {
    FILE *fp = (FILE *) simulink_kalmanfilter_DW.ToFile3_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "lab4_opg1_IMU_STATES_flying_50_sek.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(simulink_kalmanfilter_M,
                          "Error closing MAT-file lab4_opg1_IMU_STATES_flying_50_sek.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(simulink_kalmanfilter_M,
                          "Error reopening MAT-file lab4_opg1_IMU_STATES_flying_50_sek.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 5 + 1,
           simulink_kalmanfilter_DW.ToFile3_IWORK.Count, "ans")) {
        rtmSetErrorStatus(simulink_kalmanfilter_M,
                          "Error writing header for ans to MAT-file lab4_opg1_IMU_STATES_flying_50_sek.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(simulink_kalmanfilter_M,
                          "Error closing MAT-file lab4_opg1_IMU_STATES_flying_50_sek.mat");
        return;
      }

      simulink_kalmanfilter_DW.ToFile3_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for S-Function (game_controller_block): '<S5>/Game Controller' */

  /* S-Function Block: simulink_kalmanfilter/Joystick/Game Controller (game_controller_block) */
  {
    if (simulink_kalmanfilter_P.GameController_Enabled) {
      game_controller_close(simulink_kalmanfilter_DW.GameController_Controller);
      simulink_kalmanfilter_DW.GameController_Controller = NULL;
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
  simulink_kalmanfilter_output(tid);
}

void MdlUpdate(int_T tid)
{
  if (tid == 1)
    tid = 0;
  simulink_kalmanfilter_update(tid);
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
  simulink_kalmanfilter_initialize();
}

void MdlTerminate(void)
{
  simulink_kalmanfilter_terminate();
}

/* Registration function */
RT_MODEL_simulink_kalmanfilte_T *simulink_kalmanfilter(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  simulink_kalmanfilter_P.Integrator_UpperSat = rtInf;
  simulink_kalmanfilter_P.Integrator_LowerSat = rtMinusInf;

  /* initialize real-time model */
  (void) memset((void *)simulink_kalmanfilter_M, 0,
                sizeof(RT_MODEL_simulink_kalmanfilte_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&simulink_kalmanfilter_M->solverInfo,
                          &simulink_kalmanfilter_M->Timing.simTimeStep);
    rtsiSetTPtr(&simulink_kalmanfilter_M->solverInfo, &rtmGetTPtr
                (simulink_kalmanfilter_M));
    rtsiSetStepSizePtr(&simulink_kalmanfilter_M->solverInfo,
                       &simulink_kalmanfilter_M->Timing.stepSize0);
    rtsiSetdXPtr(&simulink_kalmanfilter_M->solverInfo,
                 &simulink_kalmanfilter_M->derivs);
    rtsiSetContStatesPtr(&simulink_kalmanfilter_M->solverInfo, (real_T **)
                         &simulink_kalmanfilter_M->contStates);
    rtsiSetNumContStatesPtr(&simulink_kalmanfilter_M->solverInfo,
      &simulink_kalmanfilter_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&simulink_kalmanfilter_M->solverInfo,
      &simulink_kalmanfilter_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&simulink_kalmanfilter_M->solverInfo,
      &simulink_kalmanfilter_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&simulink_kalmanfilter_M->solverInfo,
      &simulink_kalmanfilter_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&simulink_kalmanfilter_M->solverInfo,
                          (&rtmGetErrorStatus(simulink_kalmanfilter_M)));
    rtsiSetRTModelPtr(&simulink_kalmanfilter_M->solverInfo,
                      simulink_kalmanfilter_M);
  }

  rtsiSetSimTimeStep(&simulink_kalmanfilter_M->solverInfo, MAJOR_TIME_STEP);
  simulink_kalmanfilter_M->intgData.f[0] = simulink_kalmanfilter_M->odeF[0];
  simulink_kalmanfilter_M->contStates = ((real_T *) &simulink_kalmanfilter_X);
  rtsiSetSolverData(&simulink_kalmanfilter_M->solverInfo, (void *)
                    &simulink_kalmanfilter_M->intgData);
  rtsiSetSolverName(&simulink_kalmanfilter_M->solverInfo,"ode1");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = simulink_kalmanfilter_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    mdlTsMap[2] = 2;
    simulink_kalmanfilter_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    simulink_kalmanfilter_M->Timing.sampleTimes =
      (&simulink_kalmanfilter_M->Timing.sampleTimesArray[0]);
    simulink_kalmanfilter_M->Timing.offsetTimes =
      (&simulink_kalmanfilter_M->Timing.offsetTimesArray[0]);

    /* task periods */
    simulink_kalmanfilter_M->Timing.sampleTimes[0] = (0.0);
    simulink_kalmanfilter_M->Timing.sampleTimes[1] = (0.002);
    simulink_kalmanfilter_M->Timing.sampleTimes[2] = (0.01);

    /* task offsets */
    simulink_kalmanfilter_M->Timing.offsetTimes[0] = (0.0);
    simulink_kalmanfilter_M->Timing.offsetTimes[1] = (0.0);
    simulink_kalmanfilter_M->Timing.offsetTimes[2] = (0.0);
  }

  rtmSetTPtr(simulink_kalmanfilter_M, &simulink_kalmanfilter_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = simulink_kalmanfilter_M->Timing.sampleHitArray;
    int_T *mdlPerTaskSampleHits =
      simulink_kalmanfilter_M->Timing.perTaskSampleHitsArray;
    simulink_kalmanfilter_M->Timing.perTaskSampleHits = (&mdlPerTaskSampleHits[0]);
    mdlSampleHits[0] = 1;
    simulink_kalmanfilter_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(simulink_kalmanfilter_M, -1);
  simulink_kalmanfilter_M->Timing.stepSize0 = 0.002;
  simulink_kalmanfilter_M->Timing.stepSize1 = 0.002;
  simulink_kalmanfilter_M->Timing.stepSize2 = 0.01;

  /* External mode info */
  simulink_kalmanfilter_M->Sizes.checksums[0] = (3855355860U);
  simulink_kalmanfilter_M->Sizes.checksums[1] = (3663055974U);
  simulink_kalmanfilter_M->Sizes.checksums[2] = (3582113415U);
  simulink_kalmanfilter_M->Sizes.checksums[3] = (1460462062U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[6];
    simulink_kalmanfilter_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(simulink_kalmanfilter_M->extModeInfo,
      &simulink_kalmanfilter_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(simulink_kalmanfilter_M->extModeInfo,
                        simulink_kalmanfilter_M->Sizes.checksums);
    rteiSetTPtr(simulink_kalmanfilter_M->extModeInfo, rtmGetTPtr
                (simulink_kalmanfilter_M));
  }

  simulink_kalmanfilter_M->solverInfoPtr = (&simulink_kalmanfilter_M->solverInfo);
  simulink_kalmanfilter_M->Timing.stepSize = (0.002);
  rtsiSetFixedStepSize(&simulink_kalmanfilter_M->solverInfo, 0.002);
  rtsiSetSolverMode(&simulink_kalmanfilter_M->solverInfo,
                    SOLVER_MODE_MULTITASKING);

  /* block I/O */
  simulink_kalmanfilter_M->blockIO = ((void *) &simulink_kalmanfilter_B);
  (void) memset(((void *) &simulink_kalmanfilter_B), 0,
                sizeof(B_simulink_kalmanfilter_T));

  {
    int32_T i;
    for (i = 0; i < 6; i++) {
      simulink_kalmanfilter_B.x_bar[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
      simulink_kalmanfilter_B.Switch[i] = 0.0;
    }

    for (i = 0; i < 5; i++) {
      simulink_kalmanfilter_B.y_vector[i] = 0.0;
    }

    for (i = 0; i < 6; i++) {
      simulink_kalmanfilter_B.x_bar_k[i] = 0.0;
    }

    for (i = 0; i < 36; i++) {
      simulink_kalmanfilter_B.P_bar[i] = 0.0;
    }

    for (i = 0; i < 6; i++) {
      simulink_kalmanfilter_B.x_hat[i] = 0.0;
    }

    for (i = 0; i < 36; i++) {
      simulink_kalmanfilter_B.P_hat[i] = 0.0;
    }

    simulink_kalmanfilter_B.RateTransitionx = 0.0;
    simulink_kalmanfilter_B.Joystick_gain_x = 0.0;
    simulink_kalmanfilter_B.p = 0.0;
    simulink_kalmanfilter_B.Gain1[0] = 0.0;
    simulink_kalmanfilter_B.Gain1[1] = 0.0;
    simulink_kalmanfilter_B.Gain1[2] = 0.0;
    simulink_kalmanfilter_B.e = 0.0;
    simulink_kalmanfilter_B.RateTransitiony = 0.0;
    simulink_kalmanfilter_B.Joystick_gain_y = 0.0;
    simulink_kalmanfilter_B.Gain[0] = 0.0;
    simulink_kalmanfilter_B.Gain[1] = 0.0;
    simulink_kalmanfilter_B.Sum[0] = 0.0;
    simulink_kalmanfilter_B.Sum[1] = 0.0;
    simulink_kalmanfilter_B.TravelCounttorad = 0.0;
    simulink_kalmanfilter_B.TravelTransferFcn = 0.0;
    simulink_kalmanfilter_B.PitchCounttorad = 0.0;
    simulink_kalmanfilter_B.PitchTransferFcn = 0.0;
    simulink_kalmanfilter_B.ElevationCounttorad = 0.0;
    simulink_kalmanfilter_B.Sum_m = 0.0;
    simulink_kalmanfilter_B.ElevationTransferFcn = 0.0;
    simulink_kalmanfilter_B.Sum_o = 0.0;
    simulink_kalmanfilter_B.FrontmotorSaturation = 0.0;
    simulink_kalmanfilter_B.BackmotorSaturation = 0.0;
    simulink_kalmanfilter_B.Sum1 = 0.0;
    simulink_kalmanfilter_B.Sum2 = 0.0;
    simulink_kalmanfilter_B.K_ei = 0.0;
    simulink_kalmanfilter_B.euler_rates[0] = 0.0;
    simulink_kalmanfilter_B.euler_rates[1] = 0.0;
    simulink_kalmanfilter_B.euler_rates[2] = 0.0;
  }

  /* parameters */
  simulink_kalmanfilter_M->defaultParam = ((real_T *)&simulink_kalmanfilter_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &simulink_kalmanfilter_X;
    simulink_kalmanfilter_M->contStates = (x);
    (void) memset((void *)&simulink_kalmanfilter_X, 0,
                  sizeof(X_simulink_kalmanfilter_T));
  }

  /* states (dwork) */
  simulink_kalmanfilter_M->dwork = ((void *) &simulink_kalmanfilter_DW);
  (void) memset((void *)&simulink_kalmanfilter_DW, 0,
                sizeof(DW_simulink_kalmanfilter_T));

  {
    int32_T i;
    for (i = 0; i < 6; i++) {
      simulink_kalmanfilter_DW.UnitDelay_DSTATE[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 36; i++) {
      simulink_kalmanfilter_DW.UnitDelay1_DSTATE[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_kalmanfilter_DW.HILInitialize_AIMinimums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_kalmanfilter_DW.HILInitialize_AIMaximums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_kalmanfilter_DW.HILInitialize_AOMinimums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_kalmanfilter_DW.HILInitialize_AOMaximums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_kalmanfilter_DW.HILInitialize_AOVoltages[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_kalmanfilter_DW.HILInitialize_FilterFrequency[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_kalmanfilter_DW.HILInitialize_POSortedFreqs[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      simulink_kalmanfilter_DW.HILInitialize_POValues[i] = 0.0;
    }
  }

  simulink_kalmanfilter_DW.RateTransitionx_Buffer0 = 0.0;

  {
    int32_T i;
    for (i = 0; i < 10; i++) {
      simulink_kalmanfilter_DW.Memory_PreviousInput[i] = 0.0;
    }
  }

  simulink_kalmanfilter_DW.RateTransitiony_Buffer0 = 0.0;
  simulink_kalmanfilter_DW.HILWriteAnalog_Buffer[0] = 0.0;
  simulink_kalmanfilter_DW.HILWriteAnalog_Buffer[1] = 0.0;

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    simulink_kalmanfilter_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 25;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.PTransTable = &rtPTransTable;
  }

  /* Initialize Sizes */
  simulink_kalmanfilter_M->Sizes.numContStates = (6);/* Number of continuous states */
  simulink_kalmanfilter_M->Sizes.numPeriodicContStates = (0);
                                      /* Number of periodic continuous states */
  simulink_kalmanfilter_M->Sizes.numY = (0);/* Number of model outputs */
  simulink_kalmanfilter_M->Sizes.numU = (0);/* Number of model inputs */
  simulink_kalmanfilter_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  simulink_kalmanfilter_M->Sizes.numSampTimes = (3);/* Number of sample times */
  simulink_kalmanfilter_M->Sizes.numBlocks = (96);/* Number of blocks */
  simulink_kalmanfilter_M->Sizes.numBlockIO = (31);/* Number of block outputs */
  simulink_kalmanfilter_M->Sizes.numBlockPrms = (593);/* Sum of parameter "widths" */
  return simulink_kalmanfilter_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
