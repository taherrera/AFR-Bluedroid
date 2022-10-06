/* 
  author: Tomas Herrera taherrera@uc.cl
*/

#include "bt-classic.h"
#include "bt-serial/BluetoothSerial.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>



void bluetooth_init(char * btname){
    BluetoothSerial_begin(btname, 0);
}

void bluetooth_send(uint8_t * buff, int size, int * bw){
    *bw = BluetoothSerial_write(buff, size);
}

void bluetooth_send_string(char * buff, int * bw){
    return bluetooth_send((uint8_t *)buff, strlen(buff), bw);
}

void bluetooth_vprintf(const char * fmt, va_list vaptr){
    char buffer[256];
    //va_list vaptr;
    //va_start(vaptr, fmt);
    vsprintf(buffer, fmt, vaptr);
    //va_end(vaptr);

    int bw;
    return bluetooth_send_string(buffer, &bw);
}

int bluetooth_available(uint8_t * buff, int maxdata, int * br){
    int i = 0;
    while (BluetoothSerial_available()){
        buff[i++] = BluetoothSerial_read();
        if (i >= maxdata){
            *br = i;
            return 2;    
        }
    }
    *br = i;

    if (i == 0){
        return 0;
    }

    return 1;
}


