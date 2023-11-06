/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file all functions necessary to implement a small TMCL interpreter.
*/

#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#if defined(MK20DX128)
  #include "derivative.h"
#elif defined(GD32F425)
  #include "gd32f4xx.h"
#endif
#include "bits.h"
#include "stealthRocker.h"
#include "Commands.h"
#include "Globals.h"
#include "RS485.h"
#include "SysTick.h"
#include "SysControl.h"
#include "TMC5160.h"
#include "Eeprom.h"
#include "IO.h"
#include "Can.h"
#include "USB.h"
#include "Encoder.h"


#ifdef BOOTLOADER
extern UINT BLMagic;                        //!< Magic code for bootloader
#endif

#define MAX_PPS_ACC   4194303               //!< Maximum possible acceleration/deceleration in PPS/S

//Variables
static UCHAR UARTCmd[9];                    //!< RS485 command buffer
static UCHAR UARTCount;                     //!< RS485 commnd byte counter
static UCHAR TMCLCommandState;              //!< State of the interpreter
static TTMCLCommand ActualCommand;          //!< TMCL command to be executed (with all parameters)
static TTMCLReply ActualReply;              //!< Reply of last executed TMCL command
static UCHAR TMCLReplyFormat;               //!< format of next reply (RF_NORMAL or RF_SPECIAL)
static UCHAR SpecialReply[9];               //!< buffer for special replies
static UCHAR ResetRequested;                //!< TRUE after executing the software reset command
static UCHAR ExtendedCANFrame;              //!< TRUE when extended CAN frame used

static UCHAR RelativePositioningOptionCode[N_O_MOTORS];  //!< Option code for MVP REL command
static int VMax[N_O_MOTORS];                             //!< Maximum positioning speed

typedef int (*PConvertFunction)(int);                    //!< Function pointer type definition for velocity and acceleration conversion functions
static PConvertFunction VelocityToInternal[N_O_MOTORS];      //!< Pointer to velocity conversion function from PPS to TMC4361
static PConvertFunction VelocityToUser[N_O_MOTORS];          //!< Pointer to velocity conversion function from TMC4361 to PPS
static PConvertFunction AccelerationToInternal[N_O_MOTORS];  //!< Pointer to acceleration conversion function from PPS to TMC4361
static PConvertFunction AccelerationToUser[N_O_MOTORS];      //!< Pointer to acceleration conversion function from TMC4361 to PPS

//Prototypes
static void RotateRight(void);
static void RotateLeft(void);
static void MotorStop(void);
static void MoveToPosition(void);
static void SetAxisParameter(void);
static void GetAxisParameter(void);
static void GetVersion(void);
static void Boot(void);
static void SoftwareReset(void);


//Imported variables
extern char VersionString[];   //!< Imported version string


//Functions

/***************************************************************//**
   \fn ExecuteActualCommand()
   \brief Execute actual TMCL command

   Execute the TMCL command that must have been written
   to the global variable "ActualCommand" before.
********************************************************************/
static void ExecuteActualCommand(void)
{
  //Prepare answer
  ActualReply.Opcode=ActualCommand.Opcode;
  ActualReply.Status=REPLY_OK;
  ActualReply.Value.Int32=ActualCommand.Value.Int32;

  //Execute command
  switch(ActualCommand.Opcode)
  {
    case TMCL_ROR:
      RotateRight();
      break;

    case TMCL_ROL:
      RotateLeft();
      break;

    case TMCL_MST:
      MotorStop();
      break;

    case TMCL_MVP:
      MoveToPosition();
      break;

    case TMCL_SAP:
      SetAxisParameter();
      break;

    case TMCL_GAP:
      GetAxisParameter();
      break;

    case TMCL_GetVersion:
      GetVersion();
      break;

    case TMCL_Boot:
      Boot();
      break;

    case TMCL_SoftwareReset:
      SoftwareReset();
      break;

    default:
      ActualReply.Status=REPLY_INVALID_CMD;
      break;
  }
}


/***************************************************************//**
   \fn InitTMCL(void)
   \brief Initialize TMCL interpreter

   Intialise the TMCL interpreter. Must be called once at startup.
********************************************************************/
void InitTMCL(void)
{
  UINT i;

  TMCLCommandState=TCS_IDLE;
  for(i=0; i<N_O_MOTORS; i++)
  {
    if(MotorConfig[i].UnitMode==UNIT_MODE_INTERNAL)
    {
      VelocityToInternal[i]=ConvertInternalToInternal;
      VelocityToUser[i]=ConvertInternalToInternal;
      AccelerationToInternal[i]=ConvertInternalToInternal;
      AccelerationToUser[i]=ConvertInternalToInternal;
    }
    else
    {
      VelocityToInternal[i]=ConvertVelocityUserToInternal;
      VelocityToUser[i]=ConvertVelocityInternalToUser;
      AccelerationToInternal[i]=ConvertAccelerationUserToInternal;
      AccelerationToUser[i]=ConvertAccelerationInternalToUser;
    }

    VMax[i]=ReadTMC5160Int(WHICH_5160(i), TMC5160_VMAX);
    VMaxModified[i]=FALSE;
    AMax[i]=ReadTMC5160Int(WHICH_5160(i), TMC5160_AMAX);
    AMaxModified[i]=FALSE;
    CalculateEncoderParameters(i);
  }
}


/***************************************************************//**
   \fn ProcessCommand(void)
   \brief Fetch and execute TMCL commands

   This is the main function for fetching and executing TMCL commands
   and has to be called periodically from the main loop.
********************************************************************/
void ProcessCommand(void)
{
  UCHAR Byte;
  UCHAR Checksum;
  UCHAR i;
  TCanFrame CanFrame;
  UCHAR USBCmd[9];
  UCHAR USBReply[9];

  //**Send answer for the last command**
  if(TMCLCommandState==TCS_CAN7 || TMCLCommandState==TCS_CAN8)  //via CAN
  {
    CanFrame.Id=ModuleConfig.CANSendID;
    CanFrame.Dlc=(TMCLCommandState==TCS_CAN7 ? 7:8);
    CanFrame.Ext=ExtendedCANFrame;
    CanFrame.Rtr=FALSE;

    if(TMCLReplyFormat==RF_STANDARD)
    {
      CanFrame.Data[0]=ModuleConfig.CANReceiveID & 0xff;
      CanFrame.Data[1]=ActualReply.Status;
      CanFrame.Data[2]=ActualReply.Opcode;
      CanFrame.Data[3]=ActualReply.Value.Byte[3];
      CanFrame.Data[4]=ActualReply.Value.Byte[2];
      CanFrame.Data[5]=ActualReply.Value.Byte[1];
      CanFrame.Data[6]=ActualReply.Value.Byte[0];
      CanFrame.Data[7]=0;
    }
    else if(TMCLReplyFormat==RF_SPECIAL)
    {
      for(i=0; i<8; i++)
        CanFrame.Data[i]=SpecialReply[i+1];
    }

    //Antwort senden
    if(!CanSendMessage(&CanFrame)) return;
  }
  else if(TMCLCommandState==TCS_UART)  //via UART
  {
    if(TMCLReplyFormat==RF_STANDARD)
    {
      Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
               ActualReply.Status+ActualReply.Opcode+
               ActualReply.Value.Byte[3]+
               ActualReply.Value.Byte[2]+
               ActualReply.Value.Byte[1]+
               ActualReply.Value.Byte[0];

      WriteRS485(ModuleConfig.SerialHostAddress);
      WriteRS485(ModuleConfig.SerialModuleAddress);
      WriteRS485(ActualReply.Status);
      WriteRS485(ActualReply.Opcode);
      WriteRS485(ActualReply.Value.Byte[3]);
      WriteRS485(ActualReply.Value.Byte[2]);
      WriteRS485(ActualReply.Value.Byte[1]);
      WriteRS485(ActualReply.Value.Byte[0]);
      WriteRS485(Checksum);
    }
    else if(TMCLReplyFormat==RF_SPECIAL)
    {
      for(i=0; i<9; i++)
      {
        WriteRS485(SpecialReply[i]);
      }
    }
  }
  else if(TMCLCommandState==TCS_UART_ERROR)  //check sum of the last command has been wrong
  {
    ActualReply.Opcode=0;
    ActualReply.Status=REPLY_CHKERR;
    ActualReply.Value.Int32=0;

    Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
             ActualReply.Status+ActualReply.Opcode+
             ActualReply.Value.Byte[3]+
             ActualReply.Value.Byte[2]+
             ActualReply.Value.Byte[1]+
             ActualReply.Value.Byte[0];

    WriteRS485(ModuleConfig.SerialHostAddress);
    WriteRS485(ModuleConfig.SerialModuleAddress);
    WriteRS485(ActualReply.Status);
    WriteRS485(ActualReply.Opcode);
    WriteRS485(ActualReply.Value.Byte[3]);
    WriteRS485(ActualReply.Value.Byte[2]);
    WriteRS485(ActualReply.Value.Byte[1]);
    WriteRS485(ActualReply.Value.Byte[0]);
    WriteRS485(Checksum);
  }
  else if(TMCLCommandState==TCS_USB)  //via USB
  {
    if(TMCLReplyFormat==RF_STANDARD)
    {
      Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
               ActualReply.Status+ActualReply.Opcode+
               ActualReply.Value.Byte[3]+
               ActualReply.Value.Byte[2]+
               ActualReply.Value.Byte[1]+
               ActualReply.Value.Byte[0];

      USBReply[0]=ModuleConfig.SerialHostAddress;
      USBReply[1]=ModuleConfig.SerialModuleAddress;
      USBReply[2]=ActualReply.Status;
      USBReply[3]=ActualReply.Opcode;
      USBReply[4]=ActualReply.Value.Byte[3];
      USBReply[5]=ActualReply.Value.Byte[2];
      USBReply[6]=ActualReply.Value.Byte[1];
      USBReply[7]=ActualReply.Value.Byte[0];
      USBReply[8]=Checksum;
    }
    else if(TMCLReplyFormat==RF_SPECIAL)
    {
      for(i=0; i<9; i++)
      {
        USBReply[i]=SpecialReply[i];
      }
    }

    SendUSBReply(USBReply);
  }
  else if(TMCLCommandState==TCS_USB_ERROR)  //Check sum of last USB command was wrong
  {
    ActualReply.Opcode=0;
    ActualReply.Status=REPLY_CHKERR;
    ActualReply.Value.Int32=0;

    Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
             ActualReply.Status+ActualReply.Opcode+
             ActualReply.Value.Byte[3]+
             ActualReply.Value.Byte[2]+
             ActualReply.Value.Byte[1]+
             ActualReply.Value.Byte[0];

    USBReply[0]=ModuleConfig.SerialHostAddress;
    USBReply[1]=ModuleConfig.SerialModuleAddress;
    USBReply[2]=ActualReply.Status;
    USBReply[3]=ActualReply.Opcode;
    USBReply[4]=ActualReply.Value.Byte[3];
    USBReply[5]=ActualReply.Value.Byte[2];
    USBReply[6]=ActualReply.Value.Byte[1];
    USBReply[7]=ActualReply.Value.Byte[0];
    USBReply[8]=Checksum;

    //Antwort senden
    SendUSBReply(USBReply);
  }


  //Reset state (answer has been sent now)
  TMCLCommandState=TCS_IDLE;
  TMCLReplyFormat=RF_STANDARD;

  //Generate a system reset if requested by the host
  #if defined(MK20DX128)
  if(ResetRequested) ResetCPU(TRUE);
  #elif defined(GD32F425)
  if(ResetRequested) NVIC_SystemReset();
  #endif

  //**Try to get a new command**
  if(CanGetMessage(&CanFrame))  //From CAN?
  {
    ActualCommand.Opcode=CanFrame.Data[0];
    ActualCommand.Type=CanFrame.Data[1];
    ActualCommand.Motor=CanFrame.Data[2];
    ActualCommand.Value.Byte[3]=CanFrame.Data[3];
    ActualCommand.Value.Byte[2]=CanFrame.Data[4];
    ActualCommand.Value.Byte[1]=CanFrame.Data[5];
    ActualCommand.Value.Byte[0]=CanFrame.Data[6];
    ExtendedCANFrame=CanFrame.Ext;

    if(CanFrame.Dlc==7)
      TMCLCommandState=TCS_CAN7;
    else
      TMCLCommandState=TCS_CAN8;
  }
  else if(ReadRS485(&Byte))  //Get data from UART
  {
    if(CheckUARTTimeout()) UARTCount=0;  //discard everything when there has been a command timeout
    UARTCmd[UARTCount++]=Byte;

    if(UARTCount==9)  //Nine bytes have been received without timeout
    {
      UARTCount=0;

      if(UARTCmd[0]==ModuleConfig.SerialModuleAddress)  //Is this our addresss?
      {
        Checksum=0;
        for(i=0; i<8; i++) Checksum+=UARTCmd[i];

        if(Checksum==UARTCmd[8])  //Is the checksum correct?
        {
          ActualCommand.Opcode=UARTCmd[1];
          ActualCommand.Type=UARTCmd[2];
          ActualCommand.Motor=UARTCmd[3];
          ActualCommand.Value.Byte[3]=UARTCmd[4];
          ActualCommand.Value.Byte[2]=UARTCmd[5];
          ActualCommand.Value.Byte[1]=UARTCmd[6];
          ActualCommand.Value.Byte[0]=UARTCmd[7];

          TMCLCommandState=TCS_UART;

          UARTCount=0;
        }
        else TMCLCommandState=TCS_UART_ERROR;  //Checksum wrong
      }
    }
  }
  else if(GetUSBCmd(USBCmd))
  {
    //Ignore address byte
    Checksum=0;
    for(i=0; i<8; i++) Checksum+=USBCmd[i];

    if(Checksum==USBCmd[8])  //Checksum correct?
    {
      ActualCommand.Opcode=USBCmd[1];
      ActualCommand.Type=USBCmd[2];
      ActualCommand.Motor=USBCmd[3];
      ActualCommand.Value.Byte[3]=USBCmd[4];
      ActualCommand.Value.Byte[2]=USBCmd[5];
      ActualCommand.Value.Byte[1]=USBCmd[6];
      ActualCommand.Value.Byte[0]=USBCmd[7];

      TMCLCommandState=TCS_USB;

    } else TMCLCommandState=TCS_USB_ERROR;  //Checksum wrong
  }

  //**Execute the command**
  //Check if a command could be fetched and execute it.
  if(TMCLCommandState!=TCS_IDLE && TMCLCommandState!=TCS_UART_ERROR) ExecuteActualCommand();
}


//** TMCL Commands **

/***************************************************************//**
  \fn RotateRight(void)
  \brief Command ROR (see TMCL manual)

  ROR (ROtate Right) command (see TMCL manual).
********************************************************************/
static void RotateRight(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    DeviationFlag[ActualCommand.Motor]=FALSE;
    StallFlag[ActualCommand.Motor]=FALSE;

    if(AMaxModified[ActualCommand.Motor])
    {
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_AMAX, AMax[ActualCommand.Motor]);
      AMaxModified[ActualCommand.Motor]=FALSE;
    }
    VMaxModified[ActualCommand.Motor]=TRUE;
    WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VMAX, VelocityToInternal[ActualCommand.Motor](abs(ActualCommand.Value.Int32)));
    if(ActualCommand.Value.Int32>0)
      WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPMODE, 0, 0, 0, TMC5160_MODE_VELPOS);
    else
      WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPMODE, 0, 0, 0, TMC5160_MODE_VELNEG);
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn RotateLeft(void)
  \brief Command ROL

  ROL (ROtate Left) command (see TMCL manual).
********************************************************************/
static void RotateLeft(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    DeviationFlag[ActualCommand.Motor]=FALSE;
    StallFlag[ActualCommand.Motor]=FALSE;

    if(AMaxModified[ActualCommand.Motor])
    {
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_AMAX, AMax[ActualCommand.Motor]);
      AMaxModified[ActualCommand.Motor]=FALSE;
    }
    VMaxModified[ActualCommand.Motor]=TRUE;
    WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VMAX, VelocityToInternal[ActualCommand.Motor](abs(ActualCommand.Value.Int32)));
    if(ActualCommand.Value.Int32>0)
      WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPMODE, 0, 0, 0, TMC5160_MODE_VELNEG);
    else
      WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPMODE, 0, 0, 0, TMC5160_MODE_VELPOS);
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn MotorStop(void)
  \brief Command MST

  MST (Motor StoP) command (see TMCL manual).
********************************************************************/
static void MotorStop(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    VMaxModified[ActualCommand.Motor]=TRUE;
    WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VMAX, 0);
    WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPMODE, 0, 0, 0, TMC5160_MODE_VELNEG);

    DeviationFlag[ActualCommand.Motor]=FALSE;
    StallFlag[ActualCommand.Motor]=FALSE;
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn MoveToPosition(void)
  \brief Command MVP

  MVP (Move To Position) command (see TMCL manual).
********************************************************************/
static void MoveToPosition(void)
{
  int NewPosition;

  if(ActualCommand.Motor<N_O_MOTORS)
  {
    switch(ActualCommand.Type)
    {
      case MVP_ABS:
        DeviationFlag[ActualCommand.Motor]=FALSE;
        StallFlag[ActualCommand.Motor]=FALSE;

        if(VMaxModified[ActualCommand.Motor])
        {
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VMAX, VMax[ActualCommand.Motor]);
          VMaxModified[ActualCommand.Motor]=FALSE;
        }
        if(AMaxModified[ActualCommand.Motor])
        {
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_AMAX, AMax[ActualCommand.Motor]);
          AMaxModified[ActualCommand.Motor]=FALSE;
        }
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_XTARGET, ActualCommand.Value.Int32);
        WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPMODE, 0, 0, 0, TMC5160_MODE_POSITION);
        break;

      case MVP_REL:
        DeviationFlag[ActualCommand.Motor]=FALSE;
        StallFlag[ActualCommand.Motor]=FALSE;

        switch(RelativePositioningOptionCode[ActualCommand.Motor])
        {
          case RMO_TARGET:
            NewPosition=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_XTARGET);
            break;

          case RMO_ACTINT:
            NewPosition=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_XACTUAL);
            break;

          case RMO_ACTENC:
            NewPosition=GetEncoderPosition(ActualCommand.Motor);
            break;

          default:
            NewPosition=0;
            break;
        }

        if(VMaxModified[ActualCommand.Motor])
        {
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VMAX, VMax[ActualCommand.Motor]);
          VMaxModified[ActualCommand.Motor]=FALSE;
        }
        if(AMaxModified[ActualCommand.Motor])
        {
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_AMAX, AMax[ActualCommand.Motor]);
          AMaxModified[ActualCommand.Motor]=FALSE;
        }
        NewPosition+=ActualCommand.Value.Int32;
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_XTARGET, NewPosition);
        WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPMODE, 0, 0, 0, TMC5160_MODE_POSITION);
        ActualReply.Value.Int32=NewPosition;
        break;

      default:
        ActualReply.Status=REPLY_WRONG_TYPE;
        break;
    }
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn SetAxisParameter(void)
   \brief Command SAP

  SAP (Set Axis Parameter) command (see TMCL manual).
********************************************************************/
static void SetAxisParameter(void)
{
  int Value;

  if(ActualCommand.Motor<N_O_MOTORS)
  {
    switch(ActualCommand.Type)
    {
    case 0:
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_XTARGET, ActualCommand.Value.Int32);
      break;

    case 1:
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_XACTUAL, ActualCommand.Value.Int32);
      break;

    case 2:
      if(ActualCommand.Value.Int32>0)
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);
      else
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPMODE, TMC5160_MODE_VELNEG);

      VMaxModified[ActualCommand.Motor]=TRUE;
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VMAX, VelocityToInternal[ActualCommand.Motor](abs(ActualCommand.Value.Int32)));
      break;

    case 4:
      VMax[ActualCommand.Motor]=VelocityToInternal[ActualCommand.Motor](abs(ActualCommand.Value.Int32));
      if(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPMODE)==TMC5160_MODE_POSITION)
      {
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VMAX, VMax[ActualCommand.Motor]);
      }
      break;

    case 5:
      AMaxModified[ActualCommand.Motor]=FALSE;
      AMax[ActualCommand.Motor]=AccelerationToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_AMAX, AMax[ActualCommand.Motor]);
      break;

    case 6:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_IHOLD_IRUN);
      WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_IHOLD_IRUN, 0, Value >> 16,
                          ActualCommand.Value.Byte[0]/8, Value & 0xff);
      break;

    case 7:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_IHOLD_IRUN);
      WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_IHOLD_IRUN, 0, Value >> 16,
                          Value >> 8, ActualCommand.Value.Byte[0]/8);
      break;

    case 12:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE);
      if(ActualCommand.Value.Int32==0)
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value|TMC5160_SW_STOPR_ENABLE);
      else
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value & ~TMC5160_SW_STOPR_ENABLE);
      break;

    case 13:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE);
      if(ActualCommand.Value.Int32==0)
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value|TMC5160_SW_STOPL_ENABLE);
      else
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value & ~TMC5160_SW_STOPL_ENABLE);
      break;

    case 14:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE);
      if(ActualCommand.Value.Int32!=0)
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value|TMC5160_SW_SWAP_LR);
      else
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value & ~TMC5160_SW_SWAP_LR);
      break;

    case 15:
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_A1, AccelerationToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
      break;

    case 16:
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_V1, VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
      break;

    case 17:
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_DMAX, AccelerationToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
      break;

    case 18:
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_D1, AccelerationToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
      break;

    case 19:
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VSTART, VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
      break;

    case 20:
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VSTOP, VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
      break;

    case 21:
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TZEROWAIT, ActualCommand.Value.Int32);
      break;

    case 22:
      if(ActualCommand.Value.Int32>=0)
      {
        if(MotorConfig[ActualCommand.Motor].UnitMode==UNIT_MODE_PPS)
          Value=ActualCommand.Value.Int32;
        else
          Value=ConvertVelocityInternalToUser(ActualCommand.Value.Int32);

        if(Value>0)
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_THIGH, 16000000 / Value);
        else
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_THIGH, 1048757);
      }
      else ActualReply.Status=REPLY_INVALID_VALUE;
      break;

    case 23:
      WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VDCMIN, VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32));
      break;

    case 24:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE);
      if(ActualCommand.Value.Int32!=0)
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value|TMC5160_SW_STOPR_POLARITY);
      else
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value & ~TMC5160_SW_STOPR_POLARITY);
      break;

    case 25:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE);
      if(ActualCommand.Value.Int32!=0)
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value|TMC5160_SW_STOPL_POLARITY);
      else
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value & ~TMC5160_SW_STOPL_POLARITY);
      break;

    case 26:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE);
      if(ActualCommand.Value.Int32!=0)
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value|TMC5160_SW_SOFTSTOP);
      else
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE, Value & ~TMC5160_SW_SOFTSTOP);
      break;

    case 27:
      SetTMC5160ChopperVHighChm(ActualCommand.Motor, ActualCommand.Value.Int32);
      break;

    case 28:
      SetTMC5160ChopperVHighFs(ActualCommand.Motor, ActualCommand.Value.Int32);
      break;

    case 31:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_IHOLD_IRUN);
      WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_IHOLD_IRUN, 0, ActualCommand.Value.Byte[0] & 0x0f,
                           Value >> 8, Value & 0xff);
      break;

    case 32:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_DCCTRL);  //DC_TIME
      WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_DCCTRL, 0, Value >> 16,
                           ActualCommand.Value.Byte[1] & 0x03, ActualCommand.Value.Byte[0]);
      break;

    case 33:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_DCCTRL);  //DC_SG
      WriteTMC5160Datagram(WHICH_5160(ActualCommand.Motor), TMC5160_DCCTRL, 0, ActualCommand.Value.Byte[0],
                           Value >> 8, Value & 0xff);
      break;

    case 127:
      if(ActualCommand.Value.Int32==RMO_TARGET || ActualCommand.Value.Int32==RMO_ACTINT || ActualCommand.Value.Int32==RMO_ACTENC)
      {
        RelativePositioningOptionCode[ActualCommand.Motor]=ActualCommand.Value.Byte[0];
      }
      else
      {
        ActualReply.Status=REPLY_INVALID_VALUE;
      }
      break;

    case 140:
      SetTMC5160ChopperMStepRes(ActualCommand.Motor, 8-ActualCommand.Value.Int32);
      CalculateEncoderParameters(ActualCommand.Motor);
      break;

      case 162:
        SetTMC5160ChopperBlankTime(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 163:
        SetTMC5160ChopperConstantTOffMode(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 164:
        SetTMC5160ChopperDisableFastDecayComp(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 165:
        if(GetTMC5160ChopperConstantTOffMode(ActualCommand.Motor)==0)
          SetTMC5160ChopperHysteresisEnd(ActualCommand.Motor, ActualCommand.Value.Int32);
        else
          SetTMC5160ChopperFastDecayTime(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 166:
        if(GetTMC5160ChopperConstantTOffMode(ActualCommand.Motor)==0)
          SetTMC5160ChopperHysteresisStart(ActualCommand.Motor, ActualCommand.Value.Int32);
        else
          SetTMC5160ChopperSineWaveOffset(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 167:
        SetTMC5160ChopperTOff(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 168:
        SetTMC5160SmartEnergyIMin(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 169:
        SetTMC5160SmartEnergyDownStep(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 170:
        SetTMC5160SmartEnergyStallLevelMax(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 171:
        SetTMC5160SmartEnergyUpStep(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 172:
        SetTMC5160SmartEnergyStallLevelMin(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 173:
        SetTMC5160SmartEnergyFilter(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 174:
        SetTMC5160SmartEnergyStallThreshold(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 177:
        SetTMC5160ChopperDisableShortToGround(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 181:
        if(ActualCommand.Value.Int32>=0)
          MotorConfig[ActualCommand.Motor].StallVMin=VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
        else
          ActualReply.Status=REPLY_INVALID_VALUE;
        break;

      case 182:
        if(ActualCommand.Value.Int32>=0)
        {
          if(MotorConfig[ActualCommand.Motor].UnitMode==UNIT_MODE_PPS)
            Value=ActualCommand.Value.Int32;
          else
            Value=ConvertVelocityInternalToUser(ActualCommand.Value.Int32);

          if(Value>0)
            WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TCOOLTHRS, 16000000 / Value);
          else
            WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TCOOLTHRS, 1048757);
        }
        else ActualReply.Status=REPLY_INVALID_VALUE;
        break;

      case 184:
        SetTMC5160ChopperRandomTOff(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 185:
        SetTMC5160ChopperSync(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 186:
        if(ActualCommand.Value.Int32>=0)
        {
          if(MotorConfig[ActualCommand.Motor].UnitMode==UNIT_MODE_PPS)
            Value=ActualCommand.Value.Int32;
          else
            Value=ConvertVelocityInternalToUser(ActualCommand.Value.Int32);

          if(Value>0)
            WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TPWMTHRS, 16000000 / Value);
          else
            WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TPWMTHRS, 1048757);
        }
        else ActualReply.Status=REPLY_INVALID_VALUE;
        break;

      case 187:
        SetTMC5160PWMGrad(ActualCommand.Motor, ActualCommand.Value.Int32);  //PWMGrad=0 => stealthChop ausgeschaltet
        Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_GCONF);
        if(ActualCommand.Value.Int32!=0)
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_GCONF, Value|BIT2);
        else
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_GCONF, Value& ~BIT2);
        break;

      case 188:
        SetTMC5160PWMAmpl(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 191:
        SetTMC5160PWMFrequency(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 192:
        SetTMC5160PWMAutoscale(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 201:
        Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_ENCMODE);
        Value=Value & BIT10;
        Value|=(ActualCommand.Value.Int32 & ~BIT10);
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_ENCMODE, Value);
        break;

      case 202:
        MotorConfig[ActualCommand.Motor].MotorFullStepResolution=ActualCommand.Value.Int32;
        CalculateEncoderParameters(ActualCommand.Motor);
        break;

      case 204:
        SetTMC5160PWMFreewheelMode(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 209:
        SetEncoderPosition(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 210:
        MotorConfig[ActualCommand.Motor].EncoderResolution=ActualCommand.Value.Int32;
        CalculateEncoderParameters(ActualCommand.Motor);
        break;

      case 212:
        MotorConfig[ActualCommand.Motor].MaxDeviation=ActualCommand.Value.Int32;
        break;

      case 214:
        WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TPOWERDOWN, (int) floor((double) ActualCommand.Value.Int32/TPOWERDOWN_FACTOR));
        break;

      case 251:
        Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_GCONF);
        if(ActualCommand.Value.Int32!=0)
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_GCONF, Value|TMC5160_GCONF_SHAFT);
        else
          WriteTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_GCONF, Value & ~TMC5160_GCONF_SHAFT);
        break;

      case 255:
        switch(ActualCommand.Value.Int32)
        {
          case UNIT_MODE_INTERNAL:
            MotorConfig[ActualCommand.Motor].UnitMode=0;
            VelocityToInternal[ActualCommand.Motor]=ConvertInternalToInternal;
            VelocityToUser[ActualCommand.Motor]=ConvertInternalToInternal;
            AccelerationToInternal[ActualCommand.Motor]=ConvertInternalToInternal;
            AccelerationToUser[ActualCommand.Motor]=ConvertInternalToInternal;
            break;

          case UNIT_MODE_PPS:
            MotorConfig[ActualCommand.Motor].UnitMode=1;
            VelocityToInternal[ActualCommand.Motor]=ConvertVelocityUserToInternal;
            VelocityToUser[ActualCommand.Motor]=ConvertVelocityInternalToUser;
            AccelerationToInternal[ActualCommand.Motor]=ConvertAccelerationUserToInternal;
            AccelerationToUser[ActualCommand.Motor]=ConvertAccelerationInternalToUser;
            break;

           default:
            ActualReply.Status=REPLY_INVALID_VALUE;
            break;
        }
        break;

      default:
        ActualReply.Status=REPLY_WRONG_TYPE;
        break;
    }
  } else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn GetAxisParameter(void)
  \brief Command GAP

  GAP (Get Axis Parameter) command (see TMCL manual).
********************************************************************/
static void GetAxisParameter(void)
{
  int Value;

  if(ActualCommand.Motor<N_O_MOTORS)
  {
    ActualReply.Value.Int32=0;

    switch(ActualCommand.Type)
    {
    case 0:
      ActualReply.Value.Int32=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_XTARGET);
      break;

    case 1:
      ActualReply.Value.Int32=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_XACTUAL);
      break;

    case 2:
      if(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPMODE)==TMC5160_MODE_VELPOS)
        ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VMAX));
      else
        ActualReply.Value.Int32=-VelocityToUser[ActualCommand.Motor](ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VMAX));
      break;

    case 3:
      ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VACTUAL));
      break;

    case 4:
      ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](VMax[ActualCommand.Motor]);
      break;

    case 5:
      ActualReply.Value.Int32=AccelerationToUser[ActualCommand.Motor](AMax[ActualCommand.Motor]);
      break;

    case 6:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_IHOLD_IRUN);
      ActualReply.Value.Int32=((Value>>8) & 0xff)*8;
      break;

    case 7:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_IHOLD_IRUN);
      ActualReply.Value.Int32=(Value & 0xff)*8;
      break;

    case 8:
      ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPSTAT) & TMC5160_RS_POSREACHED) ? 1:0;
      break;

    case 10:
      ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPSTAT) & TMC5160_RS_STOPR) ? 1:0;
      break;

    case 11:
      ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_RAMPSTAT) & TMC5160_RS_STOPL) ? 1:0;
      break;

    case 12:
      ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE) & TMC5160_SW_STOPR_ENABLE) ? 0:1;
      break;

    case 13:
      ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE) & TMC5160_SW_STOPL_ENABLE) ? 0:1;
      break;

    case 14:
      ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE) & TMC5160_SW_SWAP_LR) ? 1:0;
      break;

    case 15:
      ActualReply.Value.Int32=AccelerationToUser[ActualCommand.Motor](ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_A1));
      break;

    case 16:
      ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_V1));
      break;

    case 17:
      ActualReply.Value.Int32=AccelerationToUser[ActualCommand.Motor](ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_DMAX));
      break;

    case 18:
      ActualReply.Value.Int32=AccelerationToUser[ActualCommand.Motor](ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_D1));
      break;

    case 19:
      ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VSTART));
      break;

    case 20:
      ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VSTOP));
      break;

    case 21:
      ActualReply.Value.Int32=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TZEROWAIT);
      break;

    case 22:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_THIGH);
      if(Value>0)
        ActualReply.Value.Int32=16000000/Value;
       else
        ActualReply.Value.Int32=16777215;
      if(MotorConfig[ActualCommand.Motor].UnitMode==UNIT_MODE_INTERNAL) ActualReply.Value.Int32=ConvertVelocityUserToInternal(ActualReply.Value.Int32);
      break;

    case 23:
      ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_VDCMIN));
      break;

    case 24:
      ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE) & TMC5160_SW_STOPR_POLARITY) ? 1:0;
      break;

    case 25:
      ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE) & TMC5160_SW_STOPL_POLARITY) ? 1:0;
      break;

    case 26:
      ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_SWMODE) & TMC5160_SW_SOFTSTOP) ? 1:0;
      break;

    case 27:
      ActualReply.Value.Int32=GetTMC5160ChopperVHighChm(ActualCommand.Motor);
      break;

    case 28:
      ActualReply.Value.Int32=GetTMC5160ChopperVHighFs(ActualCommand.Motor);
      break;

    case 29:
      ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](GetMeasuredSpeed(ActualCommand.Motor));
      break;

    case 30:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_IHOLD_IRUN);
      ActualReply.Value.Int32=((Value>>16) & 0x0f);
      break;

    case 31:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_IHOLD_IRUN);
      ActualReply.Value.Int32=((Value>>16) & 0x0f);
      break;

    case 32:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_DCCTRL);
      ActualReply.Value.Int32=Value & 0x3ff;
      break;

    case 33:
      Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_DCCTRL);
      ActualReply.Value.Int32=Value >> 16;
      break;

    case 127:
      ActualReply.Value.Int32=RelativePositioningOptionCode[ActualCommand.Motor];
      break;

    case 140:
      ActualReply.Value.Int32=8-GetTMC5160ChopperMStepRes(ActualCommand.Motor);
      break;

      //TMC5160 specific parameters
      case 162:
        ActualReply.Value.Int32=GetTMC5160ChopperBlankTime(ActualCommand.Motor);
        break;

      case 163:
        ActualReply.Value.Int32=GetTMC5160ChopperConstantTOffMode(ActualCommand.Motor);
        break;

      case 164:
        ActualReply.Value.Int32=GetTMC5160ChopperDisableFastDecayComp(ActualCommand.Motor);
        break;

      case 165:
        if(GetTMC5160ChopperConstantTOffMode(ActualCommand.Motor)==0)
          ActualReply.Value.Int32=GetTMC5160ChopperHysteresisEnd(ActualCommand.Motor);
        else
          ActualReply.Value.Int32=GetTMC5160ChopperFastDecayTime(ActualCommand.Motor);
        break;

      case 166:
        if(GetTMC5160ChopperConstantTOffMode(ActualCommand.Motor)==0)
          ActualReply.Value.Int32=GetTMC5160ChopperHysteresisStart(ActualCommand.Motor);
        else
          ActualReply.Value.Int32=GetTMC5160ChopperSineWaveOffset(ActualCommand.Motor);
        break;

      case 167:
        ActualReply.Value.Int32=GetTMC5160ChopperTOff(ActualCommand.Motor);
        break;

      case 168:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyIMin(ActualCommand.Motor);
        break;

      case 169:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyDownStep(ActualCommand.Motor);
        break;

      case 170:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyStallLevelMax(ActualCommand.Motor);
        break;

      case 171:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyUpStep(ActualCommand.Motor);
        break;

      case 172:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyStallLevelMin(ActualCommand.Motor);
        break;

      case 173:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyFilter(ActualCommand.Motor);
        break;

      case 174:
        ActualReply.Value.Int32=GetTMC5160SmartEnergyStallThreshold(ActualCommand.Motor);
        break;

      case 177:
        ActualReply.Value.Int32=GetTMC5160ChopperDisableShortToGround(ActualCommand.Motor);
        break;

      case 180:
        #if defined(DEVTYPE_TMC43xx)
        ActualReply.Value.Int32=SmartEnergy[ActualCommand.Motor];
        #else
        ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_DRVSTATUS) >> 16) & 0x1f;
        #endif
        break;

      case 181:
        ActualReply.Value.Int32=VelocityToUser[ActualCommand.Motor](MotorConfig[ActualCommand.Motor].StallVMin);
        break;

      case 182:
        Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TCOOLTHRS);
        if(Value>0)
          ActualReply.Value.Int32=16000000/Value;
         else
          ActualReply.Value.Int32=16777215;
        if(MotorConfig[ActualCommand.Motor].UnitMode==UNIT_MODE_INTERNAL) ActualReply.Value.Int32=ConvertVelocityUserToInternal(ActualReply.Value.Int32);
        break;

      case 184:
        ActualReply.Value.Int32=GetTMC5160ChopperRandomTOff(ActualCommand.Motor);
        break;

      case 185:
        ActualReply.Value.Int32=GetTMC5160ChopperSync(ActualCommand.Motor);
        break;

      case 186:
        Value=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TPWMTHRS);
        if(Value>0)
          ActualReply.Value.Int32=16000000/Value;
         else
          ActualReply.Value.Int32=16777215;
        if(MotorConfig[ActualCommand.Motor].UnitMode==UNIT_MODE_INTERNAL) ActualReply.Value.Int32=ConvertVelocityUserToInternal(ActualReply.Value.Int32);
        break;

      case 187:
        ActualReply.Value.Int32=GetTMC5160PWMGrad(ActualCommand.Motor);
        break;

      case 188:
        ActualReply.Value.Int32=GetTMC5160PWMAmpl(ActualCommand.Motor);
        break;

      case 189:
        ActualReply.Value.Int32=ReadTMC5160Int(ActualCommand.Motor, TMC5160_PWMSCALE);
        break;

      case 190:
        if((ReadTMC5160Int(ActualCommand.Motor, TMC5160_GCONF) & BIT2) && (ReadTMC5160Int(ActualCommand.Motor, TMC5160_DRVSTATUS) & BIT14))
          ActualReply.Value.Int32=1;
        else
          ActualReply.Value.Int32=0;
        break;

      case 191:
        ActualReply.Value.Int32=GetTMC5160PWMFrequency(ActualCommand.Motor);
        break;

      case 192:
        ActualReply.Value.Int32=GetTMC5160PWMAutoscale(ActualCommand.Motor);
        break;

      case 201:
        ActualReply.Value.Int32=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_ENCMODE);
        break;

      case 202:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].MotorFullStepResolution;
        break;

      case 204:
        ActualReply.Value.Int32=GetTMC5160PWMFreewheelMode(ActualCommand.Motor);
        break;

      case 206:
        ActualReply.Value.Int32=ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_DRVSTATUS) & 0x3ff;
        break;

      case 207:
        ActualReply.Value.Int32=0;
        if(StallFlag[ActualCommand.Motor]) ActualReply.Value.Int32|=BIT0;
        StallFlag[ActualCommand.Motor]=FALSE;
        if(DeviationFlag[ActualCommand.Motor]) ActualReply.Value.Int32|=BIT1;
        DeviationFlag[ActualCommand.Motor]=FALSE;
        break;

      case 208:
        ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_DRVSTATUS) >> 24) & 0xff;
        break;

      case 209:
        ActualReply.Value.Int32=GetEncoderPosition(ActualCommand.Motor);
        break;

      case 210:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].EncoderResolution;
        break;

      case 212:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].MaxDeviation;
        break;

      case 214:
        ActualReply.Value.Int32=(int) ceil((double) ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_TPOWERDOWN)*TPOWERDOWN_FACTOR);
        break;

      case 251:
        ActualReply.Value.Int32=(ReadTMC5160Int(WHICH_5160(ActualCommand.Motor), TMC5160_GCONF) & TMC5160_GCONF_SHAFT) ? 1:0;
        break;

      case 255:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].UnitMode;
        break;

      default:
        ActualReply.Status=REPLY_WRONG_TYPE;
        break;
    }
  } else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn GetVersion(void)
  \brief Command 136 (get version)

  Get the version number (when type==0) or
  the version string (when type==1).
********************************************************************/
static void GetVersion(void)
{
  UCHAR i;

  if(ActualCommand.Type==0)
  {
    TMCLReplyFormat=RF_SPECIAL;
    SpecialReply[0]=ModuleConfig.SerialHostAddress;
    for(i=0; i<8; i++)
      SpecialReply[i+1]=VersionString[i];
  }
  else if(ActualCommand.Type==1)
  {
    ActualReply.Value.Byte[3]=SW_TYPE_HIGH;
    ActualReply.Value.Byte[2]=SW_TYPE_LOW;
    ActualReply.Value.Byte[1]=SW_VERSION_HIGH;
    ActualReply.Value.Byte[0]=SW_VERSION_LOW;
  }
}


/************************************//**
   \fn Boot(void)
   \brief Enter bootloader mode

   Special command for exiting TMCL
   and calling the boot loader.
 ***************************************/
static void Boot(void)
{
#ifdef BOOTLOADER
  UINT Delay;

  if(ActualCommand.Type==0x81 && ActualCommand.Motor==0x92 &&
     ActualCommand.Value.Byte[3]==0xa3 && ActualCommand.Value.Byte[2]==0xb4 &&
     ActualCommand.Value.Byte[1]==0xc5 && ActualCommand.Value.Byte[0]==0xd6)
  {
    DISABLE_DRIVERS();

#if defined(MK20DX128)
    DeInitUSB();
    Delay=GetSysTimer();
    while(abs(GetSysTimer()-Delay)<1000);

    DisableInterrupts();
    SYST_CSR=0;
    NVICICER0=0xFFFFFFFF;  //alle Interrupts sperren
    NVICICPR0=0xFFFFFFFF;
    NVICICER1=0xFFFFFFFF;
    NVICICPR1=0xFFFFFFFF;
    NVICICER2=0xFFFFFFFF;
    NVICICPR2=0xFFFFFFFF;
    NVICICER3=0xFFFFFFFF;
    NVICICPR3=0xFFFFFFFF;
    BLMagic=0x12345678;
    ResetCPU(TRUE);
#elif defined(GD32F425)
    DetachUSB();
    Delay=GetSysTimer();
    while(abs(GetSysTimer()-Delay)<1000);
    __disable_irq();
    NVIC_DeInit();
    SysTick->CTRL=0;

    BLMagic=0x12345678;
    NVIC_SystemReset();
#endif
  }
#else
  ActualReply.Status=REPLY_CMD_NOT_AVAILABLE;
#endif
}


/**************************//**
   \fn SoftwareReset(void)
   \brief TMCL software reset command

   Issue a TMCL software reset.
 *******************************/
static void SoftwareReset(void)
{
  if(ActualCommand.Value.Int32==1234) ResetRequested=TRUE;
}
