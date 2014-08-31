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

#ifndef _SPI_H_
#define _SPI_H_

#define SPI_BUFFER_SIZE 15
extern char spi_data_buffer[SPI_BUFFER_SIZE];

#define SPI_INTERRUPT_NONE 0
#define SPI_INTERRUPT_COMPLETED 1
#define SPI_INTERRUPT_ONGOING 2

#define SPI_EXCHANGE_NONE 0
#define SPI_EXCHANGE_STARTED 1

/// Define the interface port and pin numbers

// The chip select of the device connected to the port
#define spi_cs	PORTCbits.RC0

// The TRIS register bits
#define spi_clk_tris TRISCbits.TRISC3
#define spi_di_tris  TRISCbits.TRISC4
#define spi_do_tris  TRISCbits.TRISC5
#define spi_cs_tris  TRISCbits.TRISC0

// The interrupt flag
#define spi_interrupt PIR1bits_t.SSPIF;

// Function declarations
void spi_init();
void spi_init_buffer();
char spi_exchange();
char spi_get_byte(char count);
char spi_handle_interrupt();
char spi_load_byte(char input);
void spi_load_zeros(char count);
void spi_wait_for_completion(char endCommunication);

#define spi_disable_devices()	spi_cs = 1
#endif // _SPI_H_
