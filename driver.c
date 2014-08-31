#include "driver.h"
#include "inttypes.h"
#include "spi.h"

#define OFFSET_PORT_TO_TRIS 0x12
#define PIN_OUT_SIZE 8
#define PIN_IN_SIZE 8

extern uint16_t vscp_timer;
timeBasedEventStruct timeEvent, timeOverride;

volatile unsigned char *IN_PIN_PORT[PIN_IN_SIZE] = {&PORTC, &PORTC, &PORTC, &PORTB, &PORTB, &PORTB, &PORTE, &PORTE};
char IN_PIN_NUM[PIN_IN_SIZE] = {0x04, 0x40, 0x80, 0x08, 0x10, 0x20, 0x01, 0x02};

volatile unsigned char *OUT_PIN_PORT[PIN_OUT_SIZE] = {&PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD};
char OUT_PIN_NUM[PIN_OUT_SIZE] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

void IO_setup();  //Internal usage
void TMR0_setup();//Internal usage


void setOutput (unsigned char pin, unsigned char state){
    if (pin>PIN_OUT_SIZE) return;
    if (state)
        *(OUT_PIN_PORT[pin]) |= OUT_PIN_NUM[pin];
    else{
        *(OUT_PIN_PORT[pin]) &= (0xFF ^ OUT_PIN_NUM[pin]);
    }

}

void hardware_setup(){
    IO_setup();
    TMR0_setup();
    spi_init();
    ei(); // Enable global interrupts
}

void IO_setup(){
    ANSELH = 0; //PORTB is I/O port
    for (char i=0; i<PIN_IN_SIZE; i++){
        *(IN_PIN_PORT[i] + OFFSET_PORT_TO_TRIS) |= IN_PIN_NUM[i]; //offset is used to access to TRIS register
    }
    for (char i=0; i<PIN_OUT_SIZE; i++){
        *(OUT_PIN_PORT[i] + OFFSET_PORT_TO_TRIS) &= (0xFF ^ OUT_PIN_NUM[i]); //offset is used to access to TRIS register
    }
    vscp_ledTris = 0;
    vscp_btnTris = 1;
}

void TMR0_setup(){
    TMR0L = 6;
    T0CON = 0xC5; //Timer On @8bit and prescaler 1: 64
    INTCONbits.TMR0IE = 1;
}
void TMR0_interrupt(){
    static char time[3] = {0, 0, 0};
    if (INTCONbits.TMR0IF){ //1mS Event
        INTCONbits.TMR0IF = 0;
        TMR0L += 6;
        vscp_timer++; //Timer of vscp firmware
        if((++time[0])>10){ //10mS Event
            time[0] = 0; time[1]++;
            if (timeEvent._10mS == 1) timeOverride._10mS = 1; else timeEvent._10mS = 1;
        }
        if((time[1])>10){ //100 mS Event
            time[1] = 0; time[2]++;
            if (timeEvent._100mS == 1) timeOverride._100mS = 1; else timeEvent._100mS = 1;
        }
        if((time[2])>10){ //1s Event
            time[2] = 0;
            if (timeEvent._1s == 1) timeOverride._1s = 1; else timeEvent._1s = 1;
        }
    }
}
