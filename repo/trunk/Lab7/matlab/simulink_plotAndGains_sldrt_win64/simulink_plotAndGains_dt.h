/*
 * simulink_plotAndGains_dt.h
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

#include "ext_types.h"

/* data type size table */
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T)
};

/* data type name table */
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T"
};

/* data type transitions for block I/O structure */
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&simulink_plotAndGains_B.Gain3), 0, 0, 9 },

  { (char_T *)(&simulink_plotAndGains_B.PacketInput1_o1[0]), 6, 0, 3 },

  { (char_T *)(&simulink_plotAndGains_B.PacketInput1_o2[0]), 4, 0, 9 }
  ,

  { (char_T *)(&simulink_plotAndGains_DW.DiscreteTransferFcn2_states), 0, 0, 7 },

  { (char_T *)(&simulink_plotAndGains_DW.PacketInput1_PWORK), 11, 0, 12 }
};

/* data type transition table for block I/O structure */
static DataTypeTransitionTable rtBTransTable = {
  5U,
  rtBTransitions
};

/* data type transitions for Parameters structure */
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&simulink_plotAndGains_P.PacketInput1_MaxMissedTicks), 0, 0, 4 },

  { (char_T *)(&simulink_plotAndGains_P.PacketInput1_PacketID), 6, 0, 2 },

  { (char_T *)(&simulink_plotAndGains_P.Gain3_Gain), 0, 0, 50 },

  { (char_T *)(&simulink_plotAndGains_P.ConstantMustbeThisValue0x7fff_Value), 4,
    0, 1 }
};

/* data type transition table for Parameters structure */
static DataTypeTransitionTable rtPTransTable = {
  4U,
  rtPTransitions
};

/* [EOF] simulink_plotAndGains_dt.h */
