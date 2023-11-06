/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  \file Globals.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Global variables

  This file contains the definitions for importing the variables
  defined in Globals.c.
*/

extern TModuleConfig ModuleConfig;
extern TMotorConfig MotorConfig[N_O_MOTORS];

extern UCHAR StallFlag[N_O_MOTORS];
extern UCHAR VMaxModified[N_O_MOTORS];
extern int AMax[N_O_MOTORS];
extern UCHAR AMaxModified[N_O_MOTORS];
extern UCHAR DeviationFlag[N_O_MOTORS];

extern UCHAR ExitTMCLFlag;
