/*******************************************************************************
  Project: stealthRocker Mini-TMCL (for stealthRocker)

  Module:  Globals.c
           Global variables and data structures

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
  \file Globals.c
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Global variables

  This file contains all globally used variables.
*/

#include "stealthRocker.h"


//! Global parameters (here only used for the RS232 interface)
TModuleConfig ModuleConfig=
{
  0,      //!< RS485 bitrate (0=9600)
  1,      //!< RS485 address
  2,      //!< RS485 reply address
  1,      //!< CAN receive ID
  2,      //!< CAN send ID
  8,      //!< CAN bit rate (8=1000kBit/s)
};

//! Motor configuration data
TMotorConfig MotorConfig[N_O_MOTORS]=
{{
  128,   //!< IRun
  32,    //!< IStandby
  8,     //!< MicrostepResolution
  0,     //!< SwitchMode
  0,     //!< StallVMin
  1,     //!< UnitMode
  200,   //!< MotorFullStepResolution
  4096,  //!< EncoderResolution
  0,     //!< MaxDeviation
 }
};


UCHAR StallFlag[N_O_MOTORS];        //!< actual stallGuard flag states
UCHAR VMaxModified[N_O_MOTORS];     //!< Maximum positioning speed has been modified
int AMax[N_O_MOTORS];               //!< Maximum acceleration (axis parameter #5)
UCHAR AMaxModified[N_O_MOTORS];     //!< Maximum acceleration has been modified
UCHAR DeviationFlag[N_O_MOTORS];    //!< Deviation hae been detected

UCHAR ExitTMCLFlag;   //!< This will be set to TRUE for exiting TMCL and branching to the boot loader
