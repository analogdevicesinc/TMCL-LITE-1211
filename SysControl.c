/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  SysControl.c
           Motor monitoring (automatic current switching etc.)

   Copyright (C) 2016 TRINAMIC Motion Control GmbH & Co KG
                      Waterloohain 5
                      D - 22769 Hamburg, Germany
                      http://www.trinamic.com/

   This program is free software; you can redistribute it and/or modify it
   freely.

   This program is distributed "as is" in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/

/**
  \file SysControl.c
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Motor monitoring

  This file contains the SystemControl function which does all necessary motor
  monitoring tasks.
*/

#include <limits.h>
#include <stdlib.h>
#if defined(MK20DX128)
  #include "derivative.h"
#elif defined(GD32F425)
  #include "gd32f4xx.h"
#endif
#include "bits.h"
#include "stealthRocker.h"
#include "Globals.h"
#include "IO.h"
#include "SysTick.h"
#include "TMC5160.h"

static UCHAR ActualAxis;                      //!< monitored axis
static int TOld[N_O_MOTORS];                  //!< last time (for speed measuring)
static int XOld[N_O_MOTORS];                  //!< last position (for speed measuring)
static int MeasuredSpeed[N_O_MOTORS];         //!< measured speed
static UCHAR StopOnStallState[N_O_MOTORS];    //!< state of stop on stall function


/***************************************************************//**
   \fn GetMeasuredSpeed(UCHAR Axis)
   \brief Read measured speed
   \param Axis  Axis number (with stepRocker always 0)
   \return Measured speed

   This function returns the speed measured by the TMC5160.
********************************************************************/
int GetMeasuredSpeed(UCHAR Axis)
{
  return MeasuredSpeed[Axis];
}

/***************************************************************//**
   \fn SystemControl(void)
   \brief Motor monitoring

   This function must be called periodically from the main loop and
   does some monitoring tasks, e.g. lowering the current after the
   motor has not been moving for some time.
********************************************************************/
void SystemControl(void)
{
  int XActual;
  int t2;
  UINT RampStat;

  //speed measuring (mainly for dcStep)
  if(abs(GetSysTimer()-TOld[ActualAxis])>10)
  {
    t2=GetSysTimer();
    XActual=ReadTMC5160Int(WHICH_5160(ActualAxis), TMC5160_XACTUAL);

    MeasuredSpeed[ActualAxis]=((float) (XActual-XOld[ActualAxis])/ (float) (t2-TOld[ActualAxis]))*1048.576;
    TOld[ActualAxis]=t2;
    XOld[ActualAxis]=XActual;
  }

  //status of the ramp generator
  RampStat=ReadTMC5160Int(WHICH_5160(ActualAxis), TMC5160_RAMPSTAT);

  //stop on stall
  if(RampStat & TMC5160_RS_EV_STOP_SG)
  {
    HardStop(ActualAxis);
    StallFlag[ActualAxis]=TRUE;
  }

  //switch stop on stall off or on depending on the speed
  if(MotorConfig[ActualAxis].StallVMin>0 && abs(ReadTMC5160Int(WHICH_5160(ActualAxis), TMC5160_VACTUAL))>MotorConfig[ActualAxis].StallVMin)
  {
    if(!StopOnStallState[ActualAxis])
    {
      WriteTMC5160Int(WHICH_5160(ActualAxis), TMC5160_SWMODE, ReadTMC5160Int(WHICH_5160(ActualAxis), TMC5160_SWMODE) | TMC5160_SW_SG_STOP);
      StopOnStallState[ActualAxis]=TRUE;
    }
  }
  else
  {
    if(StopOnStallState[ActualAxis])
    {
      WriteTMC5160Int(WHICH_5160(ActualAxis), TMC5160_SWMODE, ReadTMC5160Int(WHICH_5160(ActualAxis), TMC5160_SWMODE) & ~TMC5160_SW_SG_STOP);
      StopOnStallState[ActualAxis]=FALSE;
    }
  }

  //Reset flags
  WriteTMC5160Int(WHICH_5160(ActualAxis), TMC5160_RAMPSTAT, TMC5160_RS_EV_POSREACHED|TMC5160_RS_EV_STOP_SG);

  //Stop on deviation
  if(MotorConfig[ActualAxis].MaxDeviation>0 && !(RampStat & TMC5160_RS_VZERO))
  {
    if(abs(ReadTMC5160Int(WHICH_5160(ActualAxis), TMC5160_XENC)-ReadTMC5160Int(WHICH_5160(ActualAxis), TMC5160_XACTUAL)) > MotorConfig[ActualAxis].MaxDeviation)
    {
      HardStop(ActualAxis);
      DeviationFlag[ActualAxis]=TRUE;
    }
  }

  //next motor (stepRocker only has one)
  ActualAxis++;
  if(ActualAxis>=N_O_MOTORS) ActualAxis=0;
}
