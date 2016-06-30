/*
  ZerooneSupermodified.h - ZeroOne Supermodified Controller API
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
*/
#ifndef ZEROONE_SUPERMODIFIED
#define ZEROONE_SUPERMODIFIED

#include <Arduino.h>
#include "utility/zoProtocol.h"

//Configuration defines
#define ZO_PROTOCOL_COMMAND_RESPONSE_TIMEOUT_MS			200

//--Warnings--------------------------------------------------------------------
#define ZO_WARNING_NONE									0
#define ZO_WARNING_WIRE_PACKET_OVERWRITTEN				100
#define ZO_WARNING_WIRE_BUFFER_NOT_ENOUGH				101
#define ZO_WARNING_WIRE_NO_ACKNOIWLEDGE					102
#define ZO_WARNING_WIRE_BUS_ERROR						103
#define ZO_WARNING_WRONG_LRC							104
#define ZO_WARNING_RESPONSE_TIMEOUT						105

class zoSms
{
	private:
		static volatile ZO_PROTOCOL_PACKET BufferedPacket;
		static ZO_HW_ABSTRACTION ha;
		static ZO_PROTOCOL_UART_DECODER_STATE decoderState;
		static volatile bool i2cPacketReceived;
		static volatile bool CommSuccess;
		static uint8_t Warning;
		
		static void wireRxHandler(int numbytes);			
		static bool putPacketSerial(const ZO_PROTOCOL_PACKET*);					
		static bool getPacketSerial(ZO_PROTOCOL_PACKET*);							
		static bool putPacketWire(const ZO_PROTOCOL_PACKET*);						
		static bool getPacketWire(ZO_PROTOCOL_PACKET*);	
		bool getResponse(ZO_PROTOCOL_PACKET*);
		uint8_t calcLRC(ZO_PROTOCOL_PACKET* packet);									
	
	public:
		zoSms();														
		zoSms(ZO_HW_TYPE);																					
		zoSms(ZO_HW_TYPE,int rs485ReDePin);			
		bool getCommunicationSuccess();
		uint8_t getWarning();
		void end();
		
		void setProfileAcceleration(uint8_t, uint32_t);
		void setProfileConstantVelocity(uint8_t, uint32_t);
		void setCurrentLimit(uint8_t, uint16_t);
		void setPIDgainP(uint8_t, uint16_t);
		void setPIDgainI(uint8_t, uint16_t);
		void setPIDgainD(uint8_t, uint16_t);
		void setDurationForCurrentLimit(uint8_t, uint16_t);
		void moveWithVelocity(uint8_t,int32_t);
		void moveToAbsolutePosition(uint8_t, int64_t);
		void moveToRelativePosition(uint8_t, int64_t);
		void profiledMoveWithVelocity(uint8_t, int32_t);
		void profiledMoveToAbsolutePosition(uint8_t, int64_t);
		void profiledMoveToRelativePosition(uint8_t, int64_t);
		void setVelocitySetpoint(uint8_t, int32_t);
		void setAbsolutePositionSetpoint(uint8_t, int64_t);
		void setRelativePositionSetpoint(uint8_t, int64_t);
		void setProfiledVelocitySetpoint(uint8_t, int32_t);
		void setProfiledAbsolutePositionSetpoint(uint8_t, int64_t);
		void setProfiledRelativePositionSetpoint(uint8_t, int64_t);
		void configureDigitalIOs(uint8_t,bool,bool,bool);
		void setDigitalOutputs(uint8_t,bool,bool,bool);
		void setNodeID(uint8_t oldNodeId, uint8_t newNodeId);
		void resetIncrementalPosition(uint8_t);
		void start(uint8_t);
		void halt(uint8_t);
		void stop(uint8_t);
		void resetErrors(uint8_t);
		
		uint32_t getProfileAcceleration(uint8_t);
		uint32_t getProfileConstantVelocity(uint8_t);
		uint16_t getCurrentLimit(uint8_t);
		uint16_t getCurrentLimitDuration(uint8_t);
		bool getDigitalIOConfiguration(uint8_t,uint8_t);
		bool getDigitalIn(uint8_t,uint8_t);
		uint16_t getAnalogIn(uint8_t,uint8_t);
		int64_t getPosition(uint8_t);
		uint16_t getAbsolutePosition(uint8_t);
		int32_t getVelocity(uint8_t);
		int16_t getCurrent(uint8_t); //corrected 5/2016
		
		void broadCastDoMove();
		void broadcastStart();
		void broadcastHalt();
		void broadcastStop();
		uint16_t getPIDgainP(uint8_t);
		uint16_t getPIDgainI(uint8_t);
		uint16_t getPIDgainD(uint8_t);
		
};

typedef zoSms ZerooneSupermodified;

#endif		// Jim
//#endif ZEROONE_SUPERMODIFIED	// Giannis
