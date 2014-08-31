/* 
 * File:   driver.h
 * Author: F20472A
 *
 * Created on 10 agosto 2014, 2.47
 */

#ifndef DRIVER_H
#define	DRIVER_H

#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif

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



#ifdef	__cplusplus
}
#endif

#endif	/* DRIVER_H */

