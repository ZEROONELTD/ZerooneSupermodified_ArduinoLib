#ifndef ZO_PROTOCOL
#define ZO_PROTOCOL

#include <inttypes.h>
#include <Arduino.h>

#define ZO_PROTOCOL_SET_COMMANDS_START				0x00
#define ZO_PROTOCOL_GET_COMMANDS_START				0x64
#define ZO_PROTOCOL_BRC_COMMANDS_START				0xC8
#define ZO_PROTOCOL_ERROR_ID						0xFA

#define ZO_PROTOCOL_HEADER_0						0x55
#define ZO_PROTOCOL_HEADER_1						0xAA

#define ZO_PROTOCOL_BROADCAST_ID					0x00

#define ZO_PROTOCOL_DATA_SIZE						0x16
#define ZO_PROTOCOL_INFO_SIZE						0x05
#define ZO_PROTOCOL_PACKET_SIZE						(ZO_PROTOCOL_DATA_SIZE + ZO_PROTOCOL_INFO_SIZE)

#define ZO_PROTOCOL_COMMAND_RESPONSE_TIMEOUT_MS		200

typedef struct {
	uint8_t addressedNodeID;
	uint8_t ownNodeID;
	uint8_t commandID;
	uint8_t byteCount;
	uint8_t data[ZO_PROTOCOL_DATA_SIZE];
	uint8_t lrc;
}ZO_PROTOCOL_PACKET;

typedef enum {
	ZO_HW_SERIAL,
	ZO_HW_SW_SERIAL,
	ZO_HW_SERIAL_1,
	ZO_HW_SERIAL_2,
	ZO_HW_SERIAL_3,
	ZO_HW_WIRE
}ZO_HW_TYPE;

typedef struct {
	uint8_t localNodeID;
	ZO_HW_TYPE hw;
	bool ( *putPacket )(const ZO_PROTOCOL_PACKET* packet);
	bool ( *getPacket )(ZO_PROTOCOL_PACKET* packet);
	HardwareSerial* ser;
}ZO_HW_ABSTRACTION;

typedef enum {
	WAIT_ON_HEADER_0,
	WAIT_ON_HEADER_1,
	WAIT_ON_ADDRESSED_NODE_ID,
	WAIT_ON_OWN_NODE_ID,
	WAIT_ON_COMMAND_ID,
	WAIT_ON_BYTECOUNT,
	WAIT_ON_DATA,
	WAIT_ON_LRC
}ZO_PROTOCOL_UART_DECODER_STATE;

#endif