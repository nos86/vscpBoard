/*************************************************************
* SPI interface for the MCP2515 CAN controller from Microchip
*
* Depends on the SPI lib.
*
* (c) 2014, Musumeci Salvatore, XC8 compiler.
*************************************************************/

#include "mcp2515.h"
#include "mcp2515_regs.h"
#include "../config.h"
#include "spi.h"
#include "xc.h"
#include <stdlib.h>

////////////////////////////////////////////////////////////
// Software reset in case the reset pin is not available
////////////////////////////////////////////////////////////
void CAN_Reset(){
    spi_init_buffer(); // Reset the SPI buffer
    spi_load_byte(c2515Reset); // Load and TX command
    spi_exchange();
    spi_wait_for_completion(1);
    return;
}

////////////////////////////////////////////////////////////
// Write byte <data> to register with address <reg>
////////////////////////////////////////////////////////////
void CAN_writeRegister(char address, char data){
    spi_init_buffer();
    spi_load_byte(c2515Write);
    spi_load_byte(address);
    spi_load_byte(data);
    spi_exchange();
    spi_wait_for_completion(1);
}

////////////////////////////////////////////////////////////
// Read the contents of the register at <address>
////////////////////////////////////////////////////////////
char CAN_readRegister(char address){
    spi_init_buffer();
    spi_load_byte(c2515Read); // Read command
    spi_load_byte(address); // Register address
    spi_load_zeros(0x01); // Expect one byte answer
    spi_exchange(); // Start the transfer
    spi_wait_for_completion(1);
    return spi_get_byte(2);
}
char *CAN_readMultipleRegisters(char address, char length){
  // allows for sequential reading of registers starting at address - see data sheet
    spi_init_buffer();
    spi_load_byte(c2515Read); // Read command
    spi_load_byte(address); // Register address
    //TODO: possible weakness in case length is greater than maximum allocated space for SPI communication
    spi_load_zeros(length); // Expect one byte answer
    spi_exchange(); // Start the transfer
    spi_wait_for_completion(1);
    return (char*)&spi_data_buffer[2];
}

////////////////////////////////////////////////////////////
// Write <data> in <address> using <mask>
////////////////////////////////////////////////////////////
void CAN_writeRegisterUsingMask(char address, char mask, char data){
    spi_init_buffer();
    spi_load_byte(c2515BitMod);
    spi_load_byte(address);
    spi_load_byte(mask);
    spi_load_byte(data);
    spi_exchange();
    spi_wait_for_completion(1);

}

////////////////////////////////////////////////////////////
// Set operation mode to <mode> and check for actuation
////////////////////////////////////////////////////////////
char CAN_setMode( char mode ) {
  CAN_writeRegisterUsingMask( CANCTRL, 0xE0, mode);
  __delay_ms( 10 ); // allow for any transmissions to complete
  char data = CAN_readRegister( CANSTAT ); // check mode has been set
  return ( ( data & mode ) == mode );
}

char _CAN_init( int CAN_Bus_Speed, char Freq, char SJW, char autoBaud ) {
    char BRP, BT;
    float TQ, tempBT;

    CAN_Reset();  // Reset MCP2515 which puts it in configuration mode
    // Calculate bit timing registers
    float NBT = 1.0 / (float)CAN_Bus_Speed * 1000.0; // Nominal Bit Time
    for ( BRP=0; BRP<8; BRP++ ) {
     TQ = 2.0 * (float)(BRP + 1) / (float)Freq;
    tempBT = NBT / TQ;
        if ( tempBT <= 25 ) {
          BT = (int)(tempBT+0.5);
          if ( abs(tempBT - (float)BT) < 0.01 ) break;
        }
    }
    char SPT = (char)(0.7 * BT); // Sample point
    char PRSEG = (SPT - 1) / 2;
    char PHSEG1 = SPT - PRSEG - 1;
    char PHSEG2 = BT - PHSEG1 - PRSEG - 1;
    // Programming requirements
    if(PRSEG + PHSEG1 < PHSEG2) return 0;
    if(PHSEG2 <= SJW) return 0;

    char BTLMODE = 1;
    char SAM = 0;
    char data = ( ( ( SJW-1) << 6) | BRP );
    // Set registers
    CAN_writeRegister( CNF1, data );
    CAN_writeRegister( CNF2, ((BTLMODE << 7) | (SAM << 6) | ((PHSEG1-1) << 3) | (PRSEG-1) ) );
    CAN_writeRegister( CNF3, (0x40 | (PHSEG2-1) ) );
    CAN_writeRegister( TXRTSCTRL, 0 );
    CAN_writeRegister( RXB0CTRL, 0x64); //Do not apply any filter
    CAN_writeRegister( RXB1CTRL, 0x60); //Set RXB1 as backup of RBX0

    if ( autoBaud == 0 ) {
    // Return to Normal mode
        if ( !CAN_setMode(CAN_MODE_NORMAL ) ) return 0;
    }
    else {
    // Set to Listen Only mode
        if ( !CAN_setMode(CAN_MODE_LISTEN ) ) return 0;
    }
    // Enable all interupts
    CAN_writeRegister( CANINTE, 0 );

    // Test that we can read back from the MCP2515 what we wrote to it
    char rtn = CAN_readRegister( CNF1 );
    return ( rtn == data );
}
int CAN_Init( int CAN_Bus_Speed ) {
  if(  CAN_Bus_Speed > 0 ) {
    if( _CAN_init(CAN_Bus_Speed, CAN_XTAL_FREQ, 1, 0 ) ) return CAN_Bus_Speed;
  } else {
	int i=0;
	char interruptFlags = 0;
	for ( i=5; i<1000; i=i+5 ) {

	  if ( _CAN_init(i, CAN_XTAL_FREQ, 1, 1 ) ) { // check for bus activity
		CAN_writeRegister(CANINTF,0);
		delay_10mS(50); // need the bus to be communicating within this time frame
		if ( CAN_Interrupt() ) {
		  // determine which interrupt flags have been set
		  interruptFlags = CAN_readRegister( CANINTF );
		  if ( !( interruptFlags & MERRF ) ) {
		    // to get here we must have received something without errors
		    if (CAN_setMode( CAN_MODE_NORMAL ) == 0) return 0;
			return i;
		  }
		}
	  }
	}
  }
  return CAN_ACTION_FAIL;
}


char CAN_getMessage( CANMSG *message ) {
    // determine which interrupt flags have been set
    char interruptFlags = CAN_getRXStatus(); // Read( CANINTF );
    if ( interruptFlags & 0x40 ) {
            CAN_readBuffer(0, message); // read from RX buffer 0
            return CAN_ACTION_OK;
    }else if(interruptFlags & 0x80) {
            CAN_readBuffer(1, message); // read from RX buffer 1
            return CAN_ACTION_OK;
    }
    return CAN_ACTION_FAIL;
}

///////////////////////////////////////////////////////////////////////////////
// RXStatus
//
//  bit 7 - CANINTF.RX1IF
//  bit 6 - CANINTF.RX0IF
//  bit 5 -
//  bit 4 - RXBnSIDL.EIDE
//  bit 3 - RXBnDLC.RTR
//  bit 2 | 1 | 0 | Filter Match
//  ------|---|---|-------------
//	0 | 0 | 0 | RXF0
//	  0 | 0 | 1 | RXF1
//	  0 | 1 | 0 | RXF2
//	  0 | 1 | 1 | RXF3
//	  1 | 0 | 0 | RXF4
//	  1 | 0 | 1 | RXF5
//	  1 | 1 | 0 | RXF0 (rollover to RXB1)
//	  1 | 1 | 1 | RXF1 (rollover to RXB1)
///////////////////////////////////////////////////////////////////////////////
char CAN_getRXStatus(){
  spi_init_buffer();
  spi_load_byte(c2515RXStat); // Read command
  spi_load_zeros(1); // Expect one byte answer
  spi_exchange(); // Start the transfer
  spi_wait_for_completion(1);
  return spi_get_byte(1);  
}

////////////////////////////////////////////////////////////
// Read Buffer from MCP2515
////////////////////////////////////////////////////////////
void CAN_readBuffer( char buffer, CANMSG *message ) {
  // Reads an entire RX buffer.
  // buffer should be either RXB0 or RXB1
    //Read data
    char *data = CAN_readMultipleRegisters(RXB0SIDH+(buffer*16), 13);
    //Reset flag
  (*message).srr=( ((char)*(data+1) & 0x10) >0);
  (*message).xtd=( ((char)*(data+1) & 0x08) >0);

  if(  (*message).xtd ) {
    (*message).id = ( (char)*(data+0) >> 3 );
    (*message).id = ( (*message).id << 8 ) |
		( ( (char)*(data+0) << 5 ) |
		( ( (char)*(data+1) >> 5 ) << 2 ) |
		( (char)*(data+1) & 0x03 ) );
    (*message).id = ( (*message).id << 8 ) | (char)*(data+2);
    (*message).id = ( (*message).id << 8 ) | (char)*(data+3);
  }
  else {
    (*message).id = ( ( (char)*(data+0) >> 5 ) << 8 ) | ( ( (char)*(data+0) << 3 ) | ( (char)*(data+1) >> 5 ) );
  }

  (*message).rtr = ( (char)*(data+4) & 0x40 );
  (*message).dlc = ( (char)*(data+4) & 0x0F );  // Number of data bytes
  for (char i=0; i<(*message).dlc; i++)
        (*message).data[i] = (char)*(data+5+i);
  spi_init_buffer();
  spi_load_byte(c2515ReadBuf | (buffer << 2)); // Read command
  spi_exchange(); // Start the transfer
  spi_wait_for_completion(1);  
}

////////////////////////////////////////////////////////////
// Get status from MCP2515
//
//  bit 7 - CANINTF.TX2IF
//  bit 6 - TXB2CNTRL.TXREQ
//  bit 5 - CANINTF.TX1IF
//  bit 4 - TXB1CNTRL.TXREQ
//  bit 3 - CANINTF.TX0IF
//  bit 2 - TXB0CNTRL.TXREQ
//  bit 1 - CANINTFL.RX1IF
//  bit 0 - CANINTF.RX0IF
////////////////////////////////////////////////////////////
char CAN_getStatus() {
  spi_init_buffer();
  spi_load_byte(c2515Status); // Read command
  spi_load_zeros(1); // Expect one byte answer
  spi_exchange(); // Start the transfer
  spi_wait_for_completion(1);
  return spi_get_byte(1);

}

////////////////////////////////////////////////////////////
// Write Buffer from MCP2515
////////////////////////////////////////////////////////////
char CAN_sendMessage( CANMSG message ) {
	char buffer, status;
	status = CAN_getStatus(); // Get status
	
	if ( (status & CAN_STATUS_TX0_REQ) == 0) {
		buffer = 0;
	}
	else if ( (status & CAN_STATUS_TX1_REQ) == 0) {
		buffer = 1;
	} 
	else if ( (status & CAN_STATUS_TX2_REQ) == 0 ) {
		buffer = 2;
	}
	else {
		// all buffer used => could not send message
		return CAN_ACTION_FAIL;
	}
	
	CAN_loadBuffer( buffer, message );
	CAN_sendBuffer( buffer );
	
	return CAN_ACTION_OK; // OK
}
void CAN_loadBuffer( char buffer, CANMSG message ) {
    char header[4];
    if ( message.xtd ) {
        header[0] = (char)(( message.id << 3 ) >> 24); // 8 MSBits of SID
        header[1] = (char)(( message.id << 11 ) >> 24 ) & 0xE0; // 3 LSBits of SID
        header[1] = header[1] | (char)(( message.id << 14 ) >> 30 ); // 2 MSBits of EID
        header[1] = header[1] | 0x08; //Extended Frame
        header[2] = (char)( ( message.id << 16 ) >> 24 ); // EID Bits 15-8
        header[3] = (char)( ( message.id << 24 ) >> 24 ); // EID Bits 7-0
    }
    else {
        header[0] = (char)( ( message.id << 21 ) >> 24 ); // 8 MSBits of SID
        header[1] = (char)( ( message.id << 29 ) >> 24 ) & 0xE0; // 3 LSBits of SID
        header[2] = 0; // TXBnEID8
        header[3] = 0; // TXBnEID0
    }
  
    spi_init_buffer();
    buffer = (buffer<<1);
    spi_load_byte(c2515WriteBuf + buffer);
    for (char i=0; i<4; i++) spi_load_byte(header[i]);
    if (message.rtr)
        spi_load_byte(message.dlc | 0x40);
    else
        spi_load_byte(message.dlc);
    for ( char i=0; i<message.dlc; i++ ) spi_load_byte(message.data[i]);
    spi_exchange(); // Start the transfer
    spi_wait_for_completion(1);
}
void CAN_sendBuffer( char buffer ){
  spi_init_buffer();
  buffer = (1<<buffer);
  spi_load_byte(c2515RTS + buffer); // Read command
  spi_exchange(); // Start the transfer
  spi_wait_for_completion(1);
}