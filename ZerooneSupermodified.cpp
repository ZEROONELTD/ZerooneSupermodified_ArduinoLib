/*******************************************************************************
  ZerooneSupermodified.cpp - ZeroOne Supermodified Controller API
  for Arduino.
  Copyright (c) 2014 ZeroOne Mechatronics.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*******************************************************************************/

extern "C" {
  #include <avr/io.h>
  #include <avr/interrupt.h>
  #include "zoTypes.h"
  #include "zoString.h"
  #include "zoSystemTimer.h"
  #include "zoMcu.h"
}

#include "ZerooneSupermodified.h"

//#include "../Wire/Wire.h"	// Giannis
#include <Wire.h>		// Jim

//--Globals (needed for ISR access)---------------------------------------------
static volatile int _rs485ReDePin;

//--Initialize class variables--------------------------------------------------
volatile ZO_PROTOCOL_PACKET zoSms::BufferedPacket;
ZO_HW_ABSTRACTION zoSms::ha;
ZO_PROTOCOL_UART_DECODER_STATE zoSms::decoderState = WAIT_ON_HEADER_0;
volatile bool zoSms::i2cPacketReceived = false;
volatile bool zoSms::CommSuccess = true;
uint8_t zoSms::Warning = ZO_WARNING_NONE;

//--Cunstructors----------------------------------------------------------------
zoSms::zoSms(ZO_HW_TYPE hwType, int rs485ReDePin)
{
	CommSuccess = true;
	Warning = ZO_WARNING_NONE;
	ha.localNodeID = 1;
	ha.hw = hwType;
	ha.putPacket = putPacketSerial;
	ha.getPacket = getPacketSerial;	
	_rs485ReDePin = rs485ReDePin;
	decoderState = WAIT_ON_HEADER_0;
	i2cPacketReceived = false;
	zoSystemTimerInit();
	
	switch(hwType)
	{
		case ZO_HW_SERIAL:
#if defined(ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_LEONARDO)
			ha.ser = &Serial1;
#else
			ha.ser = &Serial;
#endif
		break;
		
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)		
		case ZO_HW_SERIAL_1:
			ha.ser = &Serial1;
		break;
		
		case ZO_HW_SERIAL_2:
			ha.ser = &Serial2;
		break;
		
		case ZO_HW_SERIAL_3:
			ha.ser = &Serial3;
		break;
#endif
		
		case ZO_HW_WIRE:
			Wire.begin(ha.localNodeID);
			ha.putPacket = putPacketWire;
			ha.getPacket = getPacketWire;
			Wire.onReceive(wireRxHandler);
		break;
	}
	
	if( _rs485ReDePin != -1 )
	{
		pinMode(_rs485ReDePin,OUTPUT);
		digitalWrite(_rs485ReDePin,LOW);
	}
}

zoSms::zoSms(ZO_HW_TYPE hwType)
{
	zoSms(hwType,-1);
}

zoSms::zoSms()
{
	zoSms(ZO_HW_SERIAL);
}

//--Public methods--------------------------------------------------------------
bool zoSms::getCommunicationSuccess()
{
	bool success;
	
	success = CommSuccess;				//store to local
	CommSuccess = true;					//initialize for next comm
	
	return success;
}

uint8_t zoSms::getWarning()
{
	uint8_t warn;
	
	warn = Warning;						//store to local
	Warning = ZO_WARNING_NONE;			//clear warning
	
	return warn;
}

void zoSms::setProfileAcceleration(uint8_t nodeId, uint32_t accel)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x03;
	p.byteCount = 4;
	u32ToStr(accel,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::setProfileConstantVelocity(uint8_t nodeId, uint32_t vel)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x04;
	p.byteCount = 4;
	u32ToStr(vel,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::setCurrentLimit(uint8_t nodeId, uint16_t curr)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x05;
	p.byteCount = 2;
	u16ToStr(curr,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}
void zoSms::setPIDgainP(uint8_t nodeId, uint16_t curr)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x00;
	p.byteCount = 2;
	u16ToStr(curr,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}
void zoSms::setPIDgainI(uint8_t nodeId, uint16_t curr)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x01;
	p.byteCount = 2;
	u16ToStr(curr,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}
void zoSms::setPIDgainD(uint8_t nodeId, uint16_t curr)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x02;
	p.byteCount = 2;
	u16ToStr(curr,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::setDurationForCurrentLimit(uint8_t nodeId, uint16_t dur)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x06;
	p.byteCount = 2;
	u16ToStr(dur,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::moveWithVelocity(uint8_t nodeId,int32_t vel)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x07;
	p.byteCount = 4;
	s32ToStr(vel,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::moveToAbsolutePosition(uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x08;
	p.byteCount = 8;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::moveToRelativePosition(uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x09;
	p.byteCount = 8;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::profiledMoveWithVelocity(uint8_t nodeId, int32_t vel)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x0A;
	p.byteCount = 4;
	s32ToStr(vel,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::profiledMoveToAbsolutePosition(uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x0B;
	p.byteCount = 8;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::profiledMoveToRelativePosition(uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x0C;
	p.byteCount = 8;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::setVelocitySetpoint(uint8_t nodeId, int32_t vel)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x0D;
	p.byteCount = 4;
	s32ToStr(vel,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::setAbsolutePositionSetpoint(uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x0E;
	p.byteCount = 8;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::setRelativePositionSetpoint(uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x0F;
	p.byteCount = 8;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::setProfiledVelocitySetpoint(uint8_t nodeId, int32_t vel)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x10;
	p.byteCount = 4;
	s32ToStr(vel,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::setProfiledAbsolutePositionSetpoint(uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x11;
	p.byteCount = 8;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::setProfiledRelativePositionSetpoint(uint8_t nodeId, int64_t pos)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x12;
	p.byteCount = 8;
	s64ToStr(pos,p.data);
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::configureDigitalIOs(uint8_t nodeId,bool dio1,bool dio2,bool dio3)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x13;
	p.byteCount = 1;
	p.data[0]=0;
	if(dio1)
		p.data[0] |= 0x01;
	if(dio2)
		p.data[0] |= 0x02;
	if(dio3)
		p.data[0] |= 0x04;			
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::setDigitalOutputs(uint8_t nodeId,bool do1,bool do2,bool do3)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x14;
	p.byteCount = 1;
	p.data[0]=0;
	if(do1)
		p.data[0] |= 0x01;
	if(do2)
		p.data[0] |= 0x02;
	if(do3)
		p.data[0] |= 0x04;			
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}
void zoSms::setNodeID(uint8_t oldNodeId, uint8_t newNodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = oldNodeId;
	p.ownNodeID = 1;
	p.commandID = 0x15;
	p.byteCount = 1;
	p.data[0]=newNodeId;
			
	p.lrc = calcLRC(&p);
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::resetIncrementalPosition(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x18;
	p.byteCount = 0;
	p.lrc = 0x18;
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::start(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x19;
	p.byteCount = 0;
	p.lrc = 0x19;
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::halt(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x1A;
	p.byteCount = 0;
	p.lrc = 0x1A;
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::stop(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x1B;
	p.byteCount = 0;
	p.lrc = 0x1B;
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

void zoSms::resetErrors(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x1E;
	p.byteCount = 0;
	p.lrc = 0x1E;
	
	if( ha.putPacket(&p) )
		getResponse(&p);
}

uint32_t zoSms::getProfileAcceleration(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x67;
	p.byteCount = 0;
	p.lrc = 0x67;
	
	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToU32(p.data);
		else
			return -1;
	}
	else
		return -1;
}

uint32_t zoSms::getProfileConstantVelocity(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x68;
	p.byteCount = 0;
	p.lrc = 0x68;
	
	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToU32(p.data);
		else
			return -1;
	}
	else
		return -1;
}

uint16_t zoSms::getCurrentLimit(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x69;
	p.byteCount = 0;
	p.lrc = 0x69;
	
	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToU16(p.data);
		else
			return -1;
	}
	else
		return -1;
}

uint16_t zoSms::getCurrentLimitDuration(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x6A;
	p.byteCount = 0;
	p.lrc = 0x6A;
	
	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToU16(p.data);
		else
			return -1;
	}
	else
		return -1;
}
uint16_t zoSms::getPIDgainP(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;

	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x64;
	p.byteCount = 0;
	p.lrc = 0x64;

	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToU16(p.data);
		else
			return -1;
	}
	else
		return -1;
}
uint16_t zoSms::getPIDgainI(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;

	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x65;
	p.byteCount = 0;
	p.lrc = 0x65;

	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToU16(p.data);
		else
			return -1;
	}
	else
		return -1;
}
uint16_t zoSms::getPIDgainD(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;

	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x66;
	p.byteCount = 0;
	p.lrc = 0x66;

	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToU16(p.data);
		else
			return -1;
	}
	else
		return -1;
}

bool zoSms::getDigitalIOConfiguration(uint8_t nodeId, uint8_t dio)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x6B;
	p.byteCount = 0;
	p.lrc = 0x6B;
	
	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return(( p.data[0] & (1<<dio) ) == (1<<dio) );
		else
			return -1;
	}
	else
		return -1;
}

bool zoSms::getDigitalIn(uint8_t nodeId, uint8_t din)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x6D;
	p.byteCount = 0;
	p.lrc = 0x6D;
	
	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return(( p.data[0] & (1<<din) ) == (1<<din) );
		else
			return -1;
	}
	else
		return -1;
}

uint16_t zoSms::getAnalogIn(uint8_t nodeId, uint8_t ain)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x6E;
	p.byteCount = 0;
	p.lrc = 0x6E;
	
	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToU16(&(p.data[(ain*2)-2]));
		else
			return -1;
	}
	else
		return -1;
}

int64_t zoSms::getPosition(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x6F;
	p.byteCount = 0;
	p.lrc = 0x6F;
	
	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToS64(p.data);
		else                                //ATTENTION 
			return -1;
	}
	else
		return -1;
}

uint16_t zoSms::getAbsolutePosition(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x70;
	p.byteCount = 0;
	p.lrc = 0x70;
	
	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToU16(p.data);
		else
			return -1;
	}
	else
		return -1;
}

int32_t zoSms::getVelocity(uint8_t nodeId)
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x71;
	p.byteCount = 0;
	p.lrc = 0x71;
	
	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToS32(p.data);
		else
			return -1;
	}
	else
		return -1;
}

int16_t zoSms::getCurrent(uint8_t nodeId) //corrected 5/2016
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = nodeId;
	p.ownNodeID = 1;
	p.commandID = 0x72;
	p.byteCount = 0;
	p.lrc = 0x72;
	
	if( ha.putPacket(&p) )
	{
		if( getResponse(&p) )
			return strToS16(p.data); //corrected 5/2016
		else
			return -1;
	}
	else
		return -1;
}

void zoSms::broadCastDoMove()
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = 0;
	p.ownNodeID = 1;
	p.commandID = 0xC8;
	p.byteCount = 0;
	p.lrc = 0xC8;
	
	ha.putPacket(&p);
}

void zoSms::broadcastStart()
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = 0;
	p.ownNodeID = 1;
	p.commandID = 0xC9;
	p.byteCount = 0;
	p.lrc = 0xC9;
	
	ha.putPacket(&p);
}

void zoSms::broadcastHalt()
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = 0;
	p.ownNodeID = 1;
	p.commandID = 0xCA;
	p.byteCount = 0;
	p.lrc = 0xCA;
	
	ha.putPacket(&p);
}

void zoSms::broadcastStop()
{
	ZO_PROTOCOL_PACKET p;
	
	p.addressedNodeID = 0;
	p.ownNodeID = 1;
	p.commandID = 0xCB;
	p.byteCount = 0;
	p.lrc = 0xCB;
	
	ha.putPacket(&p);
}

//--Private methods-------------------------------------------------------------
bool zoSms::getResponse(ZO_PROTOCOL_PACKET* p)
{
   // uint16_t
    uint16_t tOut;
 //   unsigned short int tOut; //Tag Untag if uint16_t produces issues
	zoSystemTimerTimeOutInit(&tOut);
	while( !(ha.getPacket(p)) )
	{
		if( zoSystemTimerTimeOutExpired(&tOut,ZO_PROTOCOL_COMMAND_RESPONSE_TIMEOUT_MS) )
		{
			CommSuccess = false;
			Warning = ZO_WARNING_RESPONSE_TIMEOUT;
			break;
		}
	}
	
	if( CommSuccess )
	{	
		if( p->lrc != calcLRC(p) )
		{	
			CommSuccess = false;
			Warning = ZO_WARNING_WRONG_LRC;
		}
		
		if( p->commandID == ZO_PROTOCOL_ERROR_ID)
		{
			CommSuccess = false;
			Warning = p->data[0];
		}
	}

	return CommSuccess;
}

uint8_t zoSms::calcLRC(ZO_PROTOCOL_PACKET* p)
{
	uint8_t i;
	uint8_t lrc = 0;
	
	lrc ^= p->commandID;
	lrc ^= p->byteCount;
	
	for( i=0; i<p->byteCount; i++)
		lrc ^= p->data[i];

   return lrc;
}

bool zoSms::putPacketSerial(const ZO_PROTOCOL_PACKET* packet)
{
	uint8_t i;
	
	//if RS485 is enabled then enable transmitter
	if( _rs485ReDePin != -1 )
		digitalWrite(_rs485ReDePin,HIGH);
	
	//put data on uart
	ha.ser->write(ZO_PROTOCOL_HEADER_0);
	ha.ser->write(ZO_PROTOCOL_HEADER_1);
	ha.ser->write(packet->addressedNodeID);
	ha.ser->write(packet->ownNodeID);
	ha.ser->write(packet->commandID);
	ha.ser->write(packet->byteCount);
	for(i=0;i<packet->byteCount;i++)
		ha.ser->write(packet->data[i]);
	ha.ser->write(packet->lrc);
	
	//Prepare to disable transmitter after transmit is done.
	//At this time data have been passed into UDRE and are shifted out one by 
	//one. This can take some time and we want to disable the transmitter 
	//exactly after transmission ends. 
	//The transmit complete interrupt will do just that.
	if( _rs485ReDePin != -1 )
	{
	#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		if( ha.hw == ZO_HW_SERIAL)
			UCSR0B |= _BV(TXCIE0);		//enable transmit complete interrupt
		
		if( ha.hw == ZO_HW_SERIAL_1)
			UCSR1B |= _BV(TXCIE1);		//enable transmit complete interrupt
		
		if( ha.hw == ZO_HW_SERIAL_2)
			UCSR2B |= _BV(TXCIE2);		//enable transmit complete interrupt
		
		if( ha.hw == ZO_HW_SERIAL_3)
			UCSR3B |= _BV(TXCIE3);		//enable transmit complete interrupt		
	#else		
		if( ha.hw == ZO_HW_SERIAL)
		{
		#if defined(__AVR_ATmega8__)	//enable transmit complete interrupt
			UCSRB |= _BV(TXCIE);		
		#elif defined(ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_LEONARDO)
			UCSR1B |= _BV(TXCIE1);		//enable transmit complete interrupt
		#else
			UCSR0B |= _BV(TXCIE0);		//enable transmit complete interrupt
		#endif
		}
	#endif
		
	}
	
	return true;	
}

bool zoSms::getPacketSerial(ZO_PROTOCOL_PACKET* packet)
{
	static uint8_t byteCount;
	bool isWholePacket = false;
	uint8_t c;
	
	if( ha.ser->peek() == -1)
		return false;
	else
		c = ha.ser->read();

	switch(decoderState) 
	{
		case WAIT_ON_HEADER_0:
			if ( c==ZO_PROTOCOL_HEADER_0 )
				decoderState = WAIT_ON_HEADER_1;
			else
				decoderState = WAIT_ON_HEADER_0;
			break;

		case WAIT_ON_HEADER_1:
			if( c == ZO_PROTOCOL_HEADER_1 )
				decoderState = WAIT_ON_ADDRESSED_NODE_ID;
			else
				decoderState = WAIT_ON_HEADER_0;
			break;

		case WAIT_ON_ADDRESSED_NODE_ID:
			if( c  == ha.localNodeID )
			{
				decoderState = WAIT_ON_OWN_NODE_ID;
				packet->addressedNodeID = c;
			}
			else
				decoderState = WAIT_ON_HEADER_0;
			break;

		case WAIT_ON_OWN_NODE_ID:
			packet->ownNodeID = c;
			decoderState = WAIT_ON_COMMAND_ID;
			break;

		case WAIT_ON_COMMAND_ID:
			packet->commandID = c;
			decoderState = WAIT_ON_BYTECOUNT;
			break;

		case WAIT_ON_BYTECOUNT:
			packet->byteCount = c;
			byteCount = packet->byteCount;	//store for internal use
			if(byteCount > 0)
				decoderState = WAIT_ON_DATA;
			else
				decoderState = WAIT_ON_LRC;
			break;

		case WAIT_ON_DATA:
			packet->data[packet->byteCount - byteCount--] = c;
			if(byteCount == 0)
				decoderState =	WAIT_ON_LRC;
			break;

		case WAIT_ON_LRC:
			packet->lrc = c;
			decoderState = WAIT_ON_HEADER_0; 
			isWholePacket = true;
			break;
	}

	return isWholePacket;		
}

bool zoSms::putPacketWire(const ZO_PROTOCOL_PACKET* packet)
{
	uint8_t i2cError = 0;
	uint8_t str[ZO_PROTOCOL_PACKET_SIZE];
	uint8_t i;

	str[0] = packet->addressedNodeID;
	str[1] = packet->ownNodeID;
	str[2] = packet->commandID;
	str[3] = packet->byteCount;
	
	for( i=0; i< packet->byteCount; i++ )
		str[4+i] = packet->data[i];

	str[(packet->byteCount + 4)] = packet->lrc;
	
	Wire.beginTransmission(packet->addressedNodeID);
	Wire.write(str,packet->byteCount + 5);
	i2cError = Wire.endTransmission(true);
	
	if(i2cError == 0)
		return true;
	
	if(i2cError == 1)
	{
		CommSuccess = false;
		Warning = ZO_WARNING_WIRE_BUFFER_NOT_ENOUGH;
		return false;
	}
	
	if( (i2cError == 2) || (i2cError == 3) )
	{
		CommSuccess = false;
		Warning = ZO_WARNING_WIRE_NO_ACKNOIWLEDGE;
		return false;
	}
	
	if(i2cError == 4)
	{
		CommSuccess = false;
		Warning = ZO_WARNING_WIRE_BUS_ERROR;
		return false;
	}
	
	return false;
}


void zoSms::wireRxHandler(int numbytes)
{
	uint8_t i;
	
	if(i2cPacketReceived)
	{
		CommSuccess = false;
		Warning = ZO_WARNING_WIRE_PACKET_OVERWRITTEN;
	}

	BufferedPacket.addressedNodeID = Wire.read();
	BufferedPacket.ownNodeID = Wire.read();
	BufferedPacket.commandID = Wire.read();
	BufferedPacket.byteCount = Wire.read();

	for( i=0; i < BufferedPacket.byteCount ; i++ )
		BufferedPacket.data[i] = Wire.read();	
		
	BufferedPacket.lrc = Wire.read();

	//indicate we received a packet
	i2cPacketReceived = true;	
}

bool zoSms::getPacketWire(ZO_PROTOCOL_PACKET* packet)
{
	uint8_t i;

	if( i2cPacketReceived )
	{
		enterCritical();
		
		//copy buffered packet
		packet->addressedNodeID = BufferedPacket.addressedNodeID;		
		packet->ownNodeID = BufferedPacket.ownNodeID;
		packet->commandID = BufferedPacket.commandID;
		packet->byteCount = BufferedPacket.byteCount;
		packet->lrc = BufferedPacket.lrc;
		for(i=0;i<packet->byteCount;i++)
			packet->data[i] = BufferedPacket.data[i];
		
		//indicate received packet was used
		i2cPacketReceived = false;
		
		exitCritical();
		
		return true;
	}
	else
		return false;
}

//--ISRs------------------------------------------------------------------------
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

	SIGNAL(USART0_TX_vect )
	{
		digitalWrite(_rs485ReDePin,LOW);	//disable transmitter
		UCSR0B &= ~_BV(TXCIE0);			//disable transmit complete interrupt
	}

	SIGNAL(USART1_TX_vect )
	{
		digitalWrite(_rs485ReDePin,LOW);
		UCSR1B &= ~_BV(TXCIE1);			//disable transmit complete interrupt
	}

	SIGNAL(USART2_TX_vect )
	{
		digitalWrite(_rs485ReDePin,LOW);
		UCSR2B &= ~_BV(TXCIE2);			//disable transmit complete interrupt
	}

	SIGNAL(USART3_TX_vect )
	{
		digitalWrite(_rs485ReDePin,LOW);
		UCSR3B &= ~_BV(TXCIE3);			//disable transmit complete interrupt
	}
#else

	#if defined(__AVR_ATmega8__)
	SIGNAL(USART_TXC_vect)
	#elif defined(ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_LEONARDO)
	SIGNAL(USART1_TX_vect )
	#else
	SIGNAL(USART_TX_vect)
	#endif
	{
		digitalWrite(_rs485ReDePin,LOW);	//disable transmitter
	#if defined(__AVR_ATmega8__)
		UCSRB &= ~_BV(TXCIE);			//disable transmit complete interrupt
	#elif defined(ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_LEONARDO)
		UCSR1B &= ~_BV(TXCIE1);			//disable transmit complete interrupt
	#else
		UCSR0B &= ~_BV(TXCIE0);			//disable transmit complete interrupt
	#endif
	}
	
#endif
