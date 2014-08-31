/*************************************************************
* SPI interface code for PIC16F devices
*
* Provides an interface to the SPI hardware unit. A single 
* buffer is used for both TX and RX. The basic idea is that
* the buffer is loaded with bytes that need to be
* transmitted, then the 'exchange' funtion is called. 
* The bytes are transmitted sequentially and are replaced
* by the bytes that are received from the other device.
* The exchange is handles through the interrupt service
* routine.
*
* (c) 2005, Lieven Hollevoet, boostc compiler.
*************************************************************/

#include "xc.h"
#include "spi.h"

// Declaration of local variables.
char spi_data_buffer[SPI_BUFFER_SIZE];
char spi_pointer;
char spi_counter;

////////////////////////////////////////////////////////////
// Init of the hardware unit
////////////////////////////////////////////////////////////
void spi_init(){

	// Tris settings
	spi_clk_tris = 0;
	spi_do_tris  = 0;
	spi_cs_tris  = 0;
	spi_di_tris  = 1;

	// Hardware registers
        SSPCON1bits.SSPEN = 0;
	SSPCON1 = 0x11;     // SPI Master, Idle high, Fosc/16
        SSPCON1bits.SSPEN = 1;
        PIR1bits.SSPIF = 0;
        PIE1bits.SSPIE = 1;
        INTCONbits.PEIE = 1;

        spi_init_buffer();
}

////////////////////////////////////////////////////////////
// Init the databuffer of the SPI code
////////////////////////////////////////////////////////////
void spi_init_buffer(){
	spi_pointer = 0;
	return;
}

////////////////////////////////////////////////////////////
// Handle the SPI interrupt.
// A single buffer, at spi_data_buffer, is used for both 
// SPI RX and TX.  When a byte is removed from the buffer 
// for transmission, it is replaced by the received byte.
// The count, spi_counter, contains the number of bytes 
// remaining to be received.  This is one less than the
// number remaining to be transmitted. When the counter
// reaches zero the transaction is completed.
////////////////////////////////////////////////////////////
char spi_handle_interrupt(){
   //Check for SPI Interrupt
    if (PIR1bits.SSPIF == 0) return SPI_INTERRUPT_NONE;
    PIR1bits.SSPIF = 0;     // Clear interrupt flag
    // Store the received byte in the data buffer
    spi_data_buffer[spi_pointer] = SSPBUF;
    spi_pointer++;
    // Decrement the buffer counter
    spi_counter--;
	
    // Exit if we received the last byte
    if (spi_counter == 0){
            spi_cs = 1;
            return SPI_INTERRUPT_COMPLETED;
    }
	
    // If we get here, there is still data left
    SSPBUF = spi_data_buffer[spi_pointer];
	return SPI_INTERRUPT_ONGOING;
}

////////////////////////////////////////////////////////////
// Add a byte to the SPI TX buffer
// It returns the position of last input
////////////////////////////////////////////////////////////
char spi_load_byte(char input){
	spi_data_buffer[spi_pointer] = input;
	return spi_pointer++;
}

////////////////////////////////////////////////////////////
// Initiate an SPI transaction
////////////////////////////////////////////////////////////
char spi_exchange(){
	// Get the number of bytes to transmit
    spi_counter = spi_pointer;
    if (spi_counter == 0){ // Sanity check
        return SPI_EXCHANGE_NONE; // Nothing to exchange
    }
    spi_cs = 0; // Enable chipselect
    SSPBUF = spi_data_buffer[0]; // Transmit the first byte
    spi_pointer = 0;
    return SPI_EXCHANGE_STARTED;
}

////////////////////////////////////////////////////////////
// Wait for the completion of the SPI exchange
// Skipped when running in debug mode.
////////////////////////////////////////////////////////////
void spi_wait_for_completion(char endCommunication){
//TODO: possible weakness here
    while (spi_counter);
    spi_cs = (endCommunication == 1);
	
}

////////////////////////////////////////////////////////////
// Load a number of zeros in the SPI data buffer
// Handy when a certain number of extra bytes is expected
// from the other SPI device
////////////////////////////////////////////////////////////
void spi_load_zeros(char count){
	char lcount = 0;		
	while ( lcount < count ) {
		spi_data_buffer[spi_pointer] = 0x00;
		spi_pointer++;
		lcount++;
	}
	return;
}

////////////////////////////////////////////////////////////
// Get a byte from the spi data buffer
////////////////////////////////////////////////////////////
char spi_get_byte(char count){
	return spi_data_buffer[count];
}
