/*
 * simulink_plotAndGains_data.c
 *
 * Classroom License -- for classroom instructional use only.  Not for
 * government, commercial, academic research, or other organizational use.
 *
 * Code generation for model "simulink_plotAndGains".
 *
 * Model version              : 1.111
 * Simulink Coder version : 8.14 (R2018a) 06-Feb-2018
 * C source code generated on : Fri Nov 16 16:41:48 2018
 *
 * Target selection: sldrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "simulink_plotAndGains.h"
#include "simulink_plotAndGains_private.h"

/* Block parameters (default storage) */
P_simulink_plotAndGains_T simulink_plotAndGains_P = {
  /* Mask Parameter: PacketInput1_MaxMissedTicks
   * Referenced by: '<S4>/Packet Input1'
   */
  50.0,

  /* Mask Parameter: PacketOutput_MaxMissedTicks
   * Referenced by: '<S5>/Packet Output'
   */
  10.0,

  /* Mask Parameter: PacketInput1_YieldWhenWaiting
   * Referenced by: '<S4>/Packet Input1'
   */
  0.0,

  /* Mask Parameter: PacketOutput_YieldWhenWaiting
   * Referenced by: '<S5>/Packet Output'
   */
  0.0,

  /* Mask Parameter: PacketInput1_PacketID
   * Referenced by: '<S4>/Packet Input1'
   */
  1,

  /* Mask Parameter: PacketOutput_PacketID
   * Referenced by: '<S5>/Packet Output'
   */
  1,

  /* Expression: 1/10
   * Referenced by: '<Root>/Gain3'
   */
  0.1,

  /* Expression: 1
   * Referenced by: '<Root>/Gain4'
   */
  1.0,

  /* Expression: 1/10000
   * Referenced by: '<Root>/Gain1'
   */
  0.0001,

  /* Expression: 1/10000
   * Referenced by: '<Root>/Gain2'
   */
  0.0001,

  /* Computed Parameter: Internal_A
   * Referenced by: '<S1>/Internal'
   */
  { 1.9959280490039844, -0.99987434418920917, 1.0 },

  /* Computed Parameter: Internal_B
   * Referenced by: '<S1>/Internal'
   */
  0.03125,

  /* Computed Parameter: Internal_C
   * Referenced by: '<S1>/Internal'
   */
  { 0.020521164805002566, 0.020520305121335795 },

  /* Expression: 0
   * Referenced by: '<S1>/Internal'
   */
  0.0,

  /* Computed Parameter: Internal_A_h
   * Referenced by: '<S2>/Internal'
   */
  0.6,

  /* Computed Parameter: Internal_B_k
   * Referenced by: '<S2>/Internal'
   */
  16.0,

  /* Computed Parameter: Internal_C_p
   * Referenced by: '<S2>/Internal'
   */
  -10.0,

  /* Computed Parameter: Internal_D_i
   * Referenced by: '<S2>/Internal'
   */
  400.0,

  /* Expression: 0
   * Referenced by: '<S2>/Internal'
   */
  0.0,

  /* Expression: -0.1572
   * Referenced by: '<Root>/Gain'
   */
  -0.1572,

  /* Expression: 0
   * Referenced by: '<Root>/Rate Transition'
   */
  0.0,

  /* Computed Parameter: Internal_A_o
   * Referenced by: '<S3>/Internal'
   */
  1.0,

  /* Computed Parameter: Internal_B_d
   * Referenced by: '<S3>/Internal'
   */
  0.03125,

  /* Computed Parameter: Internal_C_e
   * Referenced by: '<S3>/Internal'
   */
  0.032,

  /* Computed Parameter: Internal_D_p
   * Referenced by: '<S3>/Internal'
   */
  0.0005,

  /* Expression: 0
   * Referenced by: '<S3>/Internal'
   */
  0.0,

  /* Expression: 93.9734
   * Referenced by: '<Root>/Ki'
   */
  93.9734,

  /* Expression: 5.2924
   * Referenced by: '<Root>/Kp'
   */
  5.2924,

  /* Expression: 10
   * Referenced by: '<Root>/Saturation'
   */
  10.0,

  /* Expression: -10
   * Referenced by: '<Root>/Saturation'
   */
  -10.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit2'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain6'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S5>/Gain1'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit3'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain7'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S5>/Gain2'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit1'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain5'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S5>/Gain3'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit4'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain8'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S5>/Gain4'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit5 '
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain9'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S5>/Gain5'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit6'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain10'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S5>/Gain6'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit7'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain11'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S5>/Gain7'
   */
  1.0,

  /* Computed Parameter: ConstantMustbeThisValue0x7fff_Value
   * Referenced by: '<S5>/Constant Must be This Value 0x7fff'
   */
  32767
};
