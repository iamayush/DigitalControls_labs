/*
 * simulink_plotAndGains.c
 *
 * Classroom License -- for classroom instructional use only.  Not for
 * government, commercial, academic research, or other organizational use.
 *
 * Code generation for model "simulink_plotAndGains".
 *
 * Model version              : 1.111
 * Simulink Coder version : 8.14 (R2018a) 06-Feb-2018
 * C source code generated on : Mon Oct 15 17:29:36 2018
 *
 * Target selection: sldrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "simulink_plotAndGains.h"
#include "simulink_plotAndGains_private.h"
#include "simulink_plotAndGains_dt.h"

/* options for Simulink Desktop Real-Time board 0 */
static double SLDRTBoardOptions0[] = {
  115200.0,
  8.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  1.0,
};

/* list of Simulink Desktop Real-Time timers */
const int SLDRTTimerCount = 2;
const double SLDRTTimers[4] = {
  0.001, 0.0,
  0.005, 0.0,
};

/* list of Simulink Desktop Real-Time boards */
const int SLDRTBoardCount = 1;
SLDRTBOARD SLDRTBoards[1] = {
  { "Standard_Devices/Serial_Port", 1U, 8, SLDRTBoardOptions0 },
};

/* Block signals (default storage) */
B_simulink_plotAndGains_T simulink_plotAndGains_B;

/* Block states (default storage) */
DW_simulink_plotAndGains_T simulink_plotAndGains_DW;

/* Real-time model */
RT_MODEL_simulink_plotAndGains_T simulink_plotAndGains_M_;
RT_MODEL_simulink_plotAndGains_T *const simulink_plotAndGains_M =
  &simulink_plotAndGains_M_;
static void rate_monotonic_scheduler(void);
time_T rt_SimUpdateDiscreteEvents(
  int_T rtmNumSampTimes, void *rtmTimingData, int_T *rtmSampleHitPtr, int_T
  *rtmPerTaskSampleHits )
{
  rtmSampleHitPtr[1] = rtmStepTask(simulink_plotAndGains_M, 1);
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

  /* tid 0 shares data with slower tid rate: 1 */
  simulink_plotAndGains_M->Timing.RateInteraction.TID0_1 =
    (simulink_plotAndGains_M->Timing.TaskCounters.TID[1] == 0);

  /* update PerTaskSampleHits matrix for non-inline sfcn */
  simulink_plotAndGains_M->Timing.perTaskSampleHits[1] =
    simulink_plotAndGains_M->Timing.RateInteraction.TID0_1;

  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (simulink_plotAndGains_M->Timing.TaskCounters.TID[1])++;
  if ((simulink_plotAndGains_M->Timing.TaskCounters.TID[1]) > 4) {/* Sample time: [0.005s, 0.0s] */
    simulink_plotAndGains_M->Timing.TaskCounters.TID[1] = 0;
  }
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Model output function for TID0 */
void simulink_plotAndGains_output0(void) /* Sample time: [0.001s, 0.0s] */
{
  real_T Ki2;

  {                                    /* Sample time: [0.001s, 0.0s] */
    rate_monotonic_scheduler();
  }

  /* DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn3' incorporates:
   *  DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn2'
   *  Gain: '<Root>/Ki1'
   *  Gain: '<Root>/Ki2'
   *  Rounding: '<Root>/Round'
   */
  simulink_plotAndGains_DW.DiscreteTransferFcn3_tmp = (rt_roundd_snf
    (simulink_plotAndGains_P.DiscreteTransferFcn2_NumCoef *
     simulink_plotAndGains_DW.DiscreteTransferFcn2_states *
     simulink_plotAndGains_P.Ki1_Gain) * simulink_plotAndGains_P.Ki2_Gain -
    simulink_plotAndGains_P.DiscreteTransferFcn3_DenCoef[1] *
    simulink_plotAndGains_DW.DiscreteTransferFcn3_states) /
    simulink_plotAndGains_P.DiscreteTransferFcn3_DenCoef[0];
  simulink_plotAndGains_B.DiscreteTransferFcn3 =
    simulink_plotAndGains_P.DiscreteTransferFcn3_NumCoef[0] *
    simulink_plotAndGains_DW.DiscreteTransferFcn3_tmp +
    simulink_plotAndGains_P.DiscreteTransferFcn3_NumCoef[1] *
    simulink_plotAndGains_DW.DiscreteTransferFcn3_states;

  /* RateTransition: '<Root>/Rate Transition1' */
  if (simulink_plotAndGains_M->Timing.RateInteraction.TID0_1) {
    simulink_plotAndGains_B.RateTransition1 =
      simulink_plotAndGains_B.DiscreteTransferFcn3;

    /* RateTransition: '<Root>/Rate Transition' */
    simulink_plotAndGains_B.RateTransition =
      simulink_plotAndGains_DW.RateTransition_Buffer0;
  }

  /* End of RateTransition: '<Root>/Rate Transition1' */

  /* Sum: '<Root>/Minus' */
  Ki2 = simulink_plotAndGains_B.RateTransition -
    simulink_plotAndGains_B.DiscreteTransferFcn3;

  /* DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn' */
  simulink_plotAndGains_DW.DiscreteTransferFcn_tmp = (Ki2 -
    simulink_plotAndGains_P.DiscreteTransferFcn_DenCoef[1] *
    simulink_plotAndGains_DW.DiscreteTransferFcn_states) /
    simulink_plotAndGains_P.DiscreteTransferFcn_DenCoef[0];

  /* Sum: '<Root>/Minus1' incorporates:
   *  DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn'
   *  Gain: '<Root>/Ki'
   *  Gain: '<Root>/Kp'
   */
  Ki2 = (simulink_plotAndGains_P.DiscreteTransferFcn_NumCoef[0] *
         simulink_plotAndGains_DW.DiscreteTransferFcn_tmp +
         simulink_plotAndGains_P.DiscreteTransferFcn_NumCoef[1] *
         simulink_plotAndGains_DW.DiscreteTransferFcn_states) *
    simulink_plotAndGains_P.Ki_Gain + simulink_plotAndGains_P.Kp_Gain * Ki2;

  /* Saturate: '<Root>/Saturation' */
  if (Ki2 > simulink_plotAndGains_P.Saturation_UpperSat) {
    simulink_plotAndGains_B.Saturation =
      simulink_plotAndGains_P.Saturation_UpperSat;
  } else if (Ki2 < simulink_plotAndGains_P.Saturation_LowerSat) {
    simulink_plotAndGains_B.Saturation =
      simulink_plotAndGains_P.Saturation_LowerSat;
  } else {
    simulink_plotAndGains_B.Saturation = Ki2;
  }

  /* End of Saturate: '<Root>/Saturation' */
  /* DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn1' */
  simulink_plotAndGains_B.DiscreteTransferFcn1 =
    simulink_plotAndGains_P.DiscreteTransferFcn1_NumCoef *
    simulink_plotAndGains_DW.DiscreteTransferFcn1_states;
}

/* Model update function for TID0 */
void simulink_plotAndGains_update0(void) /* Sample time: [0.001s, 0.0s] */
{
  /* Update for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn2' */
  simulink_plotAndGains_DW.DiscreteTransferFcn2_states =
    (simulink_plotAndGains_B.DiscreteTransferFcn1 -
     simulink_plotAndGains_P.DiscreteTransferFcn2_DenCoef[1] *
     simulink_plotAndGains_DW.DiscreteTransferFcn2_states) /
    simulink_plotAndGains_P.DiscreteTransferFcn2_DenCoef[0];

  /* Update for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn3' */
  simulink_plotAndGains_DW.DiscreteTransferFcn3_states =
    simulink_plotAndGains_DW.DiscreteTransferFcn3_tmp;

  /* Update for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn' */
  simulink_plotAndGains_DW.DiscreteTransferFcn_states =
    simulink_plotAndGains_DW.DiscreteTransferFcn_tmp;

  /* Update for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn1' */
  simulink_plotAndGains_DW.DiscreteTransferFcn1_states =
    (simulink_plotAndGains_B.Saturation -
     simulink_plotAndGains_P.DiscreteTransferFcn1_DenCoef[1] *
     simulink_plotAndGains_DW.DiscreteTransferFcn1_states) /
    simulink_plotAndGains_P.DiscreteTransferFcn1_DenCoef[0];

  /* Update absolute time */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++simulink_plotAndGains_M->Timing.clockTick0)) {
    ++simulink_plotAndGains_M->Timing.clockTickH0;
  }

  simulink_plotAndGains_M->Timing.t[0] =
    simulink_plotAndGains_M->Timing.clockTick0 *
    simulink_plotAndGains_M->Timing.stepSize0 +
    simulink_plotAndGains_M->Timing.clockTickH0 *
    simulink_plotAndGains_M->Timing.stepSize0 * 4294967296.0;
}

/* Model output function for TID1 */
void simulink_plotAndGains_output1(void) /* Sample time: [0.005s, 0.0s] */
{
  real_T tmp;

  /* S-Function (sldrtpi): '<S1>/Packet Input1' */
  /* S-Function Block: <S1>/Packet Input1 */
  {
    uint8_T indata[12U];
    int status = RTBIO_DriverIO(0, STREAMINPUT, IOREAD, 12U,
      &simulink_plotAndGains_P.PacketInput1_PacketID, (double*) indata, NULL);
    simulink_plotAndGains_B.PacketInput1_o3 = 0;/* Missed Ticks value is always zero */
    if (status & 0x1) {
      RTWin_ANYTYPEPTR indp;
      indp.p_uint8_T = indata;
      simulink_plotAndGains_B.PacketInput1_o1[0] = *indp.p_int32_T++;
      simulink_plotAndGains_B.PacketInput1_o1[1] = *indp.p_int32_T++;
      simulink_plotAndGains_B.PacketInput1_o2[0] = *indp.p_int16_T++;
      simulink_plotAndGains_B.PacketInput1_o2[1] = *indp.p_int16_T++;
    }
  }

  /* Gain: '<Root>/Gain3' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion6'
   */
  simulink_plotAndGains_B.Gain3 = simulink_plotAndGains_P.Gain3_Gain * (real_T)
    simulink_plotAndGains_B.PacketInput1_o2[0];

  /* Gain: '<Root>/Gain4' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion5'
   */
  simulink_plotAndGains_B.Gain4 = simulink_plotAndGains_P.Gain4_Gain * (real_T)
    simulink_plotAndGains_B.PacketInput1_o2[1];

  /* Gain: '<Root>/Gain1' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion'
   */
  simulink_plotAndGains_B.Gain1 = simulink_plotAndGains_P.Gain1_Gain * (real_T)
    simulink_plotAndGains_B.PacketInput1_o1[0];

  /* Gain: '<Root>/Gain2' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion2'
   */
  simulink_plotAndGains_B.Gain2 = simulink_plotAndGains_P.Gain2_Gain * (real_T)
    simulink_plotAndGains_B.PacketInput1_o1[1];

  /* S-Function (sldrtpo): '<S2>/Packet Output' incorporates:
   *  Constant: '<S2>/Constant Must be This Value 0x7fff'
   */
  /* S-Function Block: <S2>/Packet Output */

  /* no code required */

  /* Gain: '<S2>/Gain1' incorporates:
   *  Constant: '<Root>/Value_16bit2'
   *  Gain: '<Root>/Gain6'
   */
  tmp = floor(simulink_plotAndGains_P.Gain6_Gain *
              simulink_plotAndGains_P.Value_16bit2_Value *
              simulink_plotAndGains_P.Gain1_Gain_g);
  if (rtIsNaN(tmp) || rtIsInf(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 65536.0);
  }

  simulink_plotAndGains_B.Gain1_i = (int16_T)(tmp < 0.0 ? (int32_T)(int16_T)
    -(int16_T)(uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);

  /* End of Gain: '<S2>/Gain1' */

  /* Gain: '<S2>/Gain2' incorporates:
   *  Constant: '<Root>/Value_16bit3'
   *  Gain: '<Root>/Gain7'
   */
  tmp = floor(simulink_plotAndGains_P.Gain7_Gain *
              simulink_plotAndGains_P.Value_16bit3_Value *
              simulink_plotAndGains_P.Gain2_Gain_h);
  if (rtIsNaN(tmp) || rtIsInf(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 65536.0);
  }

  simulink_plotAndGains_B.Gain2_d = (int16_T)(tmp < 0.0 ? (int32_T)(int16_T)
    -(int16_T)(uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);

  /* End of Gain: '<S2>/Gain2' */

  /* Gain: '<S2>/Gain3' incorporates:
   *  Constant: '<Root>/Value_16bit1'
   *  Gain: '<Root>/Gain5'
   */
  tmp = floor(simulink_plotAndGains_P.Gain5_Gain *
              simulink_plotAndGains_P.Value_16bit1_Value *
              simulink_plotAndGains_P.Gain3_Gain_d);
  if (rtIsNaN(tmp) || rtIsInf(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 65536.0);
  }

  simulink_plotAndGains_B.Gain3_j = (int16_T)(tmp < 0.0 ? (int32_T)(int16_T)
    -(int16_T)(uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);

  /* End of Gain: '<S2>/Gain3' */

  /* Gain: '<S2>/Gain4' incorporates:
   *  Constant: '<Root>/Value_16bit4'
   *  Gain: '<Root>/Gain8'
   */
  tmp = floor(simulink_plotAndGains_P.Gain8_Gain *
              simulink_plotAndGains_P.Value_16bit4_Value *
              simulink_plotAndGains_P.Gain4_Gain_h);
  if (rtIsNaN(tmp) || rtIsInf(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 65536.0);
  }

  simulink_plotAndGains_B.Gain4_h = (int16_T)(tmp < 0.0 ? (int32_T)(int16_T)
    -(int16_T)(uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);

  /* End of Gain: '<S2>/Gain4' */

  /* Gain: '<S2>/Gain5' incorporates:
   *  Constant: '<Root>/Value_16bit5 '
   *  Gain: '<Root>/Gain9'
   */
  tmp = floor(simulink_plotAndGains_P.Gain9_Gain *
              simulink_plotAndGains_P.Value_16bit5_Value *
              simulink_plotAndGains_P.Gain5_Gain_d);
  if (rtIsNaN(tmp) || rtIsInf(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 65536.0);
  }

  simulink_plotAndGains_B.Gain5 = (int16_T)(tmp < 0.0 ? (int32_T)(int16_T)
    -(int16_T)(uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);

  /* End of Gain: '<S2>/Gain5' */

  /* Gain: '<S2>/Gain6' incorporates:
   *  Constant: '<Root>/Value_16bit6'
   *  Gain: '<Root>/Gain10'
   */
  tmp = floor(simulink_plotAndGains_P.Gain10_Gain *
              simulink_plotAndGains_P.Value_16bit6_Value *
              simulink_plotAndGains_P.Gain6_Gain_j);
  if (rtIsNaN(tmp) || rtIsInf(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 65536.0);
  }

  simulink_plotAndGains_B.Gain6 = (int16_T)(tmp < 0.0 ? (int32_T)(int16_T)
    -(int16_T)(uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);

  /* End of Gain: '<S2>/Gain6' */

  /* Gain: '<S2>/Gain7' incorporates:
   *  Constant: '<Root>/Value_16bit7'
   *  Gain: '<Root>/Gain11'
   */
  tmp = floor(simulink_plotAndGains_P.Gain11_Gain *
              simulink_plotAndGains_P.Value_16bit7_Value *
              simulink_plotAndGains_P.Gain7_Gain_e);
  if (rtIsNaN(tmp) || rtIsInf(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 65536.0);
  }

  simulink_plotAndGains_B.Gain7 = (int16_T)(tmp < 0.0 ? (int32_T)(int16_T)
    -(int16_T)(uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);

  /* End of Gain: '<S2>/Gain7' */
}

/* Model update function for TID1 */
void simulink_plotAndGains_update1(void) /* Sample time: [0.005s, 0.0s] */
{
  /* Update for RateTransition: '<Root>/Rate Transition' */
  simulink_plotAndGains_DW.RateTransition_Buffer0 =
    simulink_plotAndGains_B.Gain3;

  /* Update for S-Function (sldrtpo): '<S2>/Packet Output' incorporates:
   *  Constant: '<S2>/Constant Must be This Value 0x7fff'
   */

  /* S-Function Block: <S2>/Packet Output */
  {
    uint8_T outdata[16U];
    RTWin_ANYTYPEPTR outdp;
    outdp.p_uint8_T = outdata;

    {
      int16_T pktout =
        simulink_plotAndGains_P.ConstantMustbeThisValue0x7fff_Value;
      *outdp.p_int16_T++ = pktout;
    }

    {
      int16_T pktout = simulink_plotAndGains_B.Gain3_j;
      *outdp.p_int16_T++ = pktout;
    }

    {
      int16_T pktout = simulink_plotAndGains_B.Gain1_i;
      *outdp.p_int16_T++ = pktout;
    }

    {
      int16_T pktout = simulink_plotAndGains_B.Gain2_d;
      *outdp.p_int16_T++ = pktout;
    }

    {
      int16_T pktout = simulink_plotAndGains_B.Gain4_h;
      *outdp.p_int16_T++ = pktout;
    }

    {
      int16_T pktout = simulink_plotAndGains_B.Gain5;
      *outdp.p_int16_T++ = pktout;
    }

    {
      int16_T pktout = simulink_plotAndGains_B.Gain6;
      *outdp.p_int16_T++ = pktout;
    }

    {
      int16_T pktout = simulink_plotAndGains_B.Gain7;
      *outdp.p_int16_T++ = pktout;
    }

    RTBIO_DriverIO(0, STREAMOUTPUT, IOWRITE, 16U,
                   &simulink_plotAndGains_P.PacketOutput_PacketID, (double*)
                   outdata, NULL);
  }

  /* Update absolute time */
  /* The "clockTick1" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick1"
   * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick1 and the high bits
   * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++simulink_plotAndGains_M->Timing.clockTick1)) {
    ++simulink_plotAndGains_M->Timing.clockTickH1;
  }

  simulink_plotAndGains_M->Timing.t[1] =
    simulink_plotAndGains_M->Timing.clockTick1 *
    simulink_plotAndGains_M->Timing.stepSize1 +
    simulink_plotAndGains_M->Timing.clockTickH1 *
    simulink_plotAndGains_M->Timing.stepSize1 * 4294967296.0;
}

/* Model output wrapper function for compatibility with a static main program */
void simulink_plotAndGains_output(int_T tid)
{
  switch (tid) {
   case 0 :
    simulink_plotAndGains_output0();
    break;

   case 1 :
    simulink_plotAndGains_output1();
    break;

   default :
    break;
  }
}

/* Model update wrapper function for compatibility with a static main program */
void simulink_plotAndGains_update(int_T tid)
{
  switch (tid) {
   case 0 :
    simulink_plotAndGains_update0();
    break;

   case 1 :
    simulink_plotAndGains_update1();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void simulink_plotAndGains_initialize(void)
{
  /* Start for RateTransition: '<Root>/Rate Transition' */
  simulink_plotAndGains_B.RateTransition =
    simulink_plotAndGains_P.RateTransition_InitialCondition;

  /* Start for S-Function (sldrtpo): '<S2>/Packet Output' incorporates:
   *  Constant: '<S2>/Constant Must be This Value 0x7fff'
   */

  /* S-Function Block: <S2>/Packet Output */
  /* no initial value should be set */

  /* InitializeConditions for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn2' */
  simulink_plotAndGains_DW.DiscreteTransferFcn2_states =
    simulink_plotAndGains_P.DiscreteTransferFcn2_InitialStates;

  /* InitializeConditions for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn3' */
  simulink_plotAndGains_DW.DiscreteTransferFcn3_states =
    simulink_plotAndGains_P.DiscreteTransferFcn3_InitialStates;

  /* InitializeConditions for RateTransition: '<Root>/Rate Transition' */
  simulink_plotAndGains_DW.RateTransition_Buffer0 =
    simulink_plotAndGains_P.RateTransition_InitialCondition;

  /* InitializeConditions for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn' */
  simulink_plotAndGains_DW.DiscreteTransferFcn_states =
    simulink_plotAndGains_P.DiscreteTransferFcn_InitialStates;

  /* InitializeConditions for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn1' */
  simulink_plotAndGains_DW.DiscreteTransferFcn1_states =
    simulink_plotAndGains_P.DiscreteTransferFcn1_InitialStates;
}

/* Model terminate function */
void simulink_plotAndGains_terminate(void)
{
  /* Terminate for S-Function (sldrtpo): '<S2>/Packet Output' incorporates:
   *  Constant: '<S2>/Constant Must be This Value 0x7fff'
   */

  /* S-Function Block: <S2>/Packet Output */
  /* no initial value should be set */
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  simulink_plotAndGains_output(tid);
}

void MdlUpdate(int_T tid)
{
  simulink_plotAndGains_update(tid);
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
  simulink_plotAndGains_initialize();
}

void MdlTerminate(void)
{
  simulink_plotAndGains_terminate();
}

/* Registration function */
RT_MODEL_simulink_plotAndGains_T *simulink_plotAndGains(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)simulink_plotAndGains_M, 0,
                sizeof(RT_MODEL_simulink_plotAndGains_T));

  /* Initialize timing info */
  {
    int_T *mdlTsMap = simulink_plotAndGains_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    simulink_plotAndGains_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    simulink_plotAndGains_M->Timing.sampleTimes =
      (&simulink_plotAndGains_M->Timing.sampleTimesArray[0]);
    simulink_plotAndGains_M->Timing.offsetTimes =
      (&simulink_plotAndGains_M->Timing.offsetTimesArray[0]);

    /* task periods */
    simulink_plotAndGains_M->Timing.sampleTimes[0] = (0.001);
    simulink_plotAndGains_M->Timing.sampleTimes[1] = (0.005);

    /* task offsets */
    simulink_plotAndGains_M->Timing.offsetTimes[0] = (0.0);
    simulink_plotAndGains_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(simulink_plotAndGains_M, &simulink_plotAndGains_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = simulink_plotAndGains_M->Timing.sampleHitArray;
    int_T *mdlPerTaskSampleHits =
      simulink_plotAndGains_M->Timing.perTaskSampleHitsArray;
    simulink_plotAndGains_M->Timing.perTaskSampleHits = (&mdlPerTaskSampleHits[0]);
    mdlSampleHits[0] = 1;
    simulink_plotAndGains_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(simulink_plotAndGains_M, 1000.0);
  simulink_plotAndGains_M->Timing.stepSize0 = 0.001;
  simulink_plotAndGains_M->Timing.stepSize1 = 0.005;

  /* External mode info */
  simulink_plotAndGains_M->Sizes.checksums[0] = (2453460796U);
  simulink_plotAndGains_M->Sizes.checksums[1] = (2152053025U);
  simulink_plotAndGains_M->Sizes.checksums[2] = (979795426U);
  simulink_plotAndGains_M->Sizes.checksums[3] = (2560813684U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    simulink_plotAndGains_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(simulink_plotAndGains_M->extModeInfo,
      &simulink_plotAndGains_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(simulink_plotAndGains_M->extModeInfo,
                        simulink_plotAndGains_M->Sizes.checksums);
    rteiSetTPtr(simulink_plotAndGains_M->extModeInfo, rtmGetTPtr
                (simulink_plotAndGains_M));
  }

  simulink_plotAndGains_M->solverInfoPtr = (&simulink_plotAndGains_M->solverInfo);
  simulink_plotAndGains_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&simulink_plotAndGains_M->solverInfo, 0.001);
  rtsiSetSolverMode(&simulink_plotAndGains_M->solverInfo,
                    SOLVER_MODE_MULTITASKING);

  /* block I/O */
  simulink_plotAndGains_M->blockIO = ((void *) &simulink_plotAndGains_B);
  (void) memset(((void *) &simulink_plotAndGains_B), 0,
                sizeof(B_simulink_plotAndGains_T));

  /* parameters */
  simulink_plotAndGains_M->defaultParam = ((real_T *)&simulink_plotAndGains_P);

  /* states (dwork) */
  simulink_plotAndGains_M->dwork = ((void *) &simulink_plotAndGains_DW);
  (void) memset((void *)&simulink_plotAndGains_DW, 0,
                sizeof(DW_simulink_plotAndGains_T));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    simulink_plotAndGains_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 14;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.PTransTable = &rtPTransTable;
  }

  /* Initialize Sizes */
  simulink_plotAndGains_M->Sizes.numContStates = (0);/* Number of continuous states */
  simulink_plotAndGains_M->Sizes.numY = (0);/* Number of model outputs */
  simulink_plotAndGains_M->Sizes.numU = (0);/* Number of model inputs */
  simulink_plotAndGains_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  simulink_plotAndGains_M->Sizes.numSampTimes = (2);/* Number of sample times */
  simulink_plotAndGains_M->Sizes.numBlocks = (53);/* Number of blocks */
  simulink_plotAndGains_M->Sizes.numBlockIO = (21);/* Number of block outputs */
  simulink_plotAndGains_M->Sizes.numBlockPrms = (57);/* Sum of parameter "widths" */
  return simulink_plotAndGains_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
