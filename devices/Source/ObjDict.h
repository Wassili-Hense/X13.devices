/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _OBJ_DICT_H_
#define _OBJ_DICT_H_

#define OD_DEV_TYP_LEN          6

#define EXTDIO_BASE   0x0080
#define EXTAIN_BASE   0x0100
#define EXTPWM_BASE   0x0200

#ifdef EXTDIO_USED
#if (DIO_PORT_SIZE == 8)
#define DIO_PORT_TYPE       uint8_t
#elif (DIO_PORT_SIZE == 16)
#define DIO_PORT_TYPE       uint16_t
#else
#error DIO_PORT_SIZE unknown size
#endif  //  DIO_PORT_SIZE

typedef enum
{
  DIO_MODE_IN_FLOAT = 0,
  DIO_MODE_IN_PD,
  DIO_MODE_IN_PU,
  DIO_MODE_OUT,
}eDIOmode_t;
#endif  //  EXTDIO_USED

// Predefined variables
typedef enum
{
  // Global Settings
  objNodeName     = (uint16_t)0xFF00, // _sName<String>
  objTAsleep      = (uint16_t)0xFF01, // cfg/XD_SleepTime<UIint16>
  // RF Node
  objRFNodeId     = (uint16_t)0xFF10, // cfg/XD_DeviceAddr<UInt8>
  objRFGroup      = (uint16_t)0xFF11, // cfg/XD_GroupID<UInt16>
  objRFChannel    = (uint16_t)0xFF12, // cfg/XD_Channel<UInt8>
  objRSSI         = (uint16_t)0xFF13, // cfg/XD_RSSI<UInt8>
  objGateID       = (uint16_t)0xFF14, // --
  // Lan Node 
  objMACAddr      = (uint16_t)0xFF20, // cfg/Xa_MACAddr   - Array - Len 6
  objIPAddr       = (uint16_t)0xFF21, // cfg/Xa_IPAddr    - Array - Len 4
  objIPMask       = (uint16_t)0xFF22, // cfg/Xa_IPMask    - Array - Len 4
  objIPRouter     = (uint16_t)0xFF23, // cfg/Xa_IPRouter  - Array - Len 4
  objIPBroker     = (uint16_t)0xFF24, // cfg/Xa_IPBroker  - Array - Len 4
  // Read Only Variables
  objDeviceTyp    = (uint16_t)0xFFC0  // _declarer<String>
}eObjList_t;

typedef enum
{
  objPROGMEM  = 'F',  // FLASH
  objEEMEM    = 'E',  // EEPROM
  objAin      = 'A',  // Analog Input
  //    objAout     = 'D',  // Analog Output
  objDin      = 'I',  // Digital Input
  objDout     = 'O',  // Digital Output
  objPWM      = 'P',  // LED PWM out
  objSer      = 'S',  // Serial port, S.0-5 (1200-38400)
  objTWI      = 'T',  // TWI
  objUsrExt   = 'X',  // User extensions
}eObjPlace_t;

typedef enum eObjTyp
{
  objBool     = 'z',  // bool
  objInt8     = 'b',  // int8
  objUInt8    = 'B',  // uint8
  objInt16    = 'w',  // int16
  objUInt16   = 'W',  // uint16
  objInt32    = 'd',  // int32
  objUInt32   = 'D',  // uint32
  objInt64    = 'q',  // int64
  objString   = 's',  // UTF8 '0' - terminated string
  objArray    = 'a',  // array
  objUnknown  = 'U',  // unknown
// Port's
  objPinPNP   = 'p',  // Pin PNP
  objPinNPN   = 'n',  // Pin NPN ( Input - PullUp)
  objActPNP   = 'A',  // Node Active  = 1, sleep = 0
  objActNPN   = 'a',  // Node Active = 0, sleep = 1;
// Analog In
  objArefVcc  = 'v',  // Ref = Vcc(3,3V)
  objArefExt  = 'e',  // Ref = External(not connected)
  objArefInt1 = 'i',  // Ref = Internal(1,1V)
  objArefInt2 = 'I',  // Ref = Internal(2,56V)
// Serial port
  ObjSerRx    = 'r',  // Com to RF
  ObjSerTx    = 't',  // RF to COM, Object type - array
}eObjTyp_t;

// Variable description
typedef struct
{
  eObjPlace_t Place;
  eObjTyp_t   Type;
  uint16_t    Base;
}subidx_t;

typedef uint8_t (*cbRead_t)(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf);  // Callback Read
typedef uint8_t (*cbWrite_t)(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf);   // Callback Write
typedef uint8_t (*cbPoll_t)(subidx_t * pSubidx, uint8_t sleep);                  // Callback Poll

// Structure for creating entries
typedef struct
{
  subidx_t    sidx;
  uint16_t    Index;
  cbRead_t    cbRead;
  cbWrite_t   cbWrite;
  cbPoll_t    cbPoll;
}indextable_t;

// EEPROM Object's
enum
{
// Global
  eeNodeName = 2,
  eeNodeNamebody = eeNodeName + MQTTSN_SIZEOF_CLIENTID - 2,
// ASLEEP
  eeTAsleep,
  eeTAsleepbody = eeTAsleep + sizeof(uint16_t) - 1,
// LAN_NODE
  eeMACAddr,
  eeMACAddrBody = eeMACAddr + 5,
  eeIPAddr,
  eeIPAddrbody = eeIPAddr + sizeof(uint32_t) - 1,
  eeIPMask,
  eeIPMaskbody = eeIPMask + sizeof(uint32_t) - 1,
  eeIPRouter,
  eeIPRouterbody = eeIPRouter + sizeof(uint32_t) - 1,
  eeIPBroker,
  eeIPBrokerbody = eeIPBroker + sizeof(uint32_t) - 1,
// RF_NODE
#if (defined RF_ADDR_t)
  eeNodeID,
  eeNodeIDbody = eeNodeID + sizeof(RF_ADDR_t) - 1,
  eeGateID,
  eeGateIDbody = eeGateID + sizeof(RF_ADDR_t) - 1,
#endif  //  RF_ADDR_t
  eeGroupID,
  eeGroupIDbody = eeGroupID + sizeof(uint16_t) - 1,
  eeChannel,
// BackUp Objects
  eelistOdbu,
  eelistOdbubody = eelistOdbu + OD_MAX_INDEX_LIST * sizeof(subidx_t) - 1,
  eeNextFreeAddress
} eEEPROMAddr;

void InitOD(void);
uint8_t ReadOD(uint16_t Id, uint8_t Flags, uint8_t *pLen, uint8_t *pBuf);
uint8_t WriteOD(uint16_t Id, uint8_t Flags, uint8_t Len, uint8_t *pBuf);

uint8_t MakeTopicName(uint8_t RecNR, uint8_t *pBuf);
void RegAckOD(uint16_t index);
e_MQTTSN_RETURNS_t RegisterOD(MQTTSN_MESSAGE_t *pMsg);
indextable_t * getFreeIdxOD(void);

uint8_t ReadODpack(uint16_t Id, uint8_t Flags, uint8_t *pLen, uint8_t *pBuf);
uint8_t WriteODpack(uint16_t Id, uint8_t Flags, uint8_t Len, uint8_t *pBuf);

#endif
