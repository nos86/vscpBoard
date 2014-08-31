#include "driver.h"

#define PIN_OUT_SIZE 8
#define PIN_IN_SIZE 8
char *PIN_PORT[PIN_OUT_SIZE] = {(&PORTD), (&PORTD), (&PORTD), (&PORTD), (&PORTD), (&PORTD), (&PORTD), (&PORTD)};
char PIN_NUM[PIN_OUT_SIZE] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};


void setOutput (unsigned char pin, unsigned char state){
    if (pin>PIN_OUT_SIZE) return;
    if (state)
        *(PIN_PORT[pin]) |= PIN_NUM[pin];
    else{
        *(PIN_PORT[pin]) &= (0xFF ^ PIN_NUM[pin]);
    }

}
