/* 
 * File:   driver.h
 * Author: Musumeci Salvatore
 *
 * Aim of this module is to provided an abstrated layer for your VSCP application.
 * This part of code is demanded to inteface the HW with your code, ignoring the HW variant
 *
 * For example, if you have to hardware variant where same pin is connected in two different pin, your code will not change
 * Will be this interface that will be recompiled in order to support the change.
 *
 * Also, this interface is able to handling the CAN message for input
 * Obviously all parameters could be customized using normal VSCP register access
 * Created on 10 agosto 2014, 2.47
 */

#ifndef DRIVER_H
#define	DRIVER_H

#include <xc.h>
#include "inttypes.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define OFFSET_PORT_TO_TRIS 0x12
#define PIN_OUT_SIZE 8
#define PIN_IN_SIZE 8

//VSCP button and led definition
#define vscp_ledPin  PORTDbits.RD0
#define vscp_ledTris TRISDbits.TRISD0
#define vscp_btnPin  PORTBbits.RB0
#define vscp_btnTris TRISBbits.TRISB0

typedef struct {
    unsigned  _10mS :1;
    unsigned _100mS :1;
    unsigned    _1s :1;
}timeBasedEventStruct;


void hardware_setup();
void setOutput (unsigned char pin, unsigned char state);
void TMR0_interrupt();
void hardware_100mS();


#define VSCP_BOARD_EEPROM_LENGTH 3*PIN_IN_SIZE + 2*PIN_OUT_SIZE



#ifdef	__cplusplus
}
#endif

#endif	/* DRIVER_H */

