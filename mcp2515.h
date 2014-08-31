/*************************************************************
* SPI interface for the MCP2515 CAN controller from Microchip
*
* Depends on the SPI lib.
*
* (c) 2005, Lieven Hollevoet, boostc compiler.
*************************************************************/

#ifndef _MCP2515_H_
#define _MCP2515_H_

#include <xc.h>

#define CAN_INT_PIN     PORTCbits.RC2
#define CAN_INT_TRIS    TRISbits.TRISC2

#define CAN_XTAL_FREQ   10 //MHz

#define CAN_Interrupt() !CAN_INT_PIN

typedef struct{
    unsigned long id;	// EID if ide set, SID otherwise
    unsigned srr :1;		// Standard Frame Remote Transmit Request
    unsigned rtr :1;		// Remote Transmission Request
    unsigned xtd :1;		// Extended ID flag
    char dlc;		// Number of data bytes
    char data[8];	// Data bytes
} CANMSG;

// Generic Bit definitions
#define CAN_RX_MASK     0x60
#define CAN_BUKT        0x04
#define CAN_RX_ANY      0x60
#define CAN_RX_EXT      0x40
#define CAN_RX_STD      0x20
#define CAN_RX_STDEXT   0x00
#define CAN_INT_RX0     0x01
#define CAN_INT_RX1     0x02
#define CAN_INT_TX0     0x04
#define CAN_INT_TX1     0x08
#define CAN_INT_TX2     0x10
#define CAN_CANINTF_RX0 0x01

// use with SPI_Rts function
#define RTS0        0x01 
#define RTS1        0x02
#define RTS2        0x04

// Operation Mode
#define CAN_MODE_CONFIG     0x80
#define CAN_MODE_LISTEN     0x60
#define CAN_MODE_LOOPBACK   0x40
#define CAN_MODE_SLEEP      0x20
#define CAN_MODE_NORMAL     0x00

// Status Bit
#define CAN_STATUS_RX0      0x01
#define CAN_STATUS_RX1      0x02
#define CAN_STATUS_TX0_REQ  0x04
#define CAN_STATUS_TX0_FLG  0x08
#define CAN_STATUS_TX1_REQ  0x10
#define CAN_STATUS_TX1_FLG  0x20
#define CAN_STATUS_TX2_REQ  0x40
#define CAN_STATUS_TX2_FLG  0x80

#define CAN_ACTION_OK       0x01
#define CAN_ACTION_FAIL     0x00

// Instruction set
#define c2515Read   0x03      // MCP2515 read instruction
#define c2515Write  0x02      // MCP2515 write instruction
#define c2515Reset  0xC0      // MCP2515 reset instruction
#define c2515RTS    0x80      // MCP2515 RTS instruction (Request-To-Send)
#define c2515Status 0xA0      // MCP2515 Status instruction
#define c2515BitMod 0x05      // MCP2515 bit modify instruction
#define c2515ReadBuf 0x90     // MCP2515 read buffer
#define c2515WriteBuf 0x40    // MCP2515 load TX message on device
#define c2515RXStat 0xB0      // MCP2515 RX Status


// Connections to the chip
#define mcp2515_rst PORTBbits.RB1
#define mcp2515_rst_tris TRISBbits.TRISB1


void CAN_Reset();
void CAN_writeRegister(char address, char data);
char CAN_readRegister(char address);
void CAN_writeRegisterUsingMask(char address, char mask, char data);
char CAN_setMode( char mode );
char *CAN_readMultipleRegisters(char address, char length);
char CAN_getStatus();

char _CAN_init( int CAN_Bus_Speed, char Freq, char SJW, char autoBaud );
int CAN_Init( int CAN_Bus_Speed );

char CAN_sendMessage( CANMSG message );
void CAN_loadBuffer( char buffer, CANMSG message );
void CAN_sendBuffer( char buffer );

char CAN_getMessage( CANMSG *message );
void CAN_readBuffer( char buffer, CANMSG *message );
char CAN_getRXStatus();

#endif // _MCP2515_H_

