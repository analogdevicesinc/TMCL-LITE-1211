/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
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
