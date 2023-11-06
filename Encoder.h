/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains functions for using the encoder interface of the TMC5160.
*/

void InitEncoder(void);
void CalculateEncoderParameters(UCHAR Axis);
int GetEncoderPosition(UCHAR Axis);
void SetEncoderPosition(UCHAR Axis, int Value);

