/* 
  author: Tomas Herrera taherrera@uc.cl
*/

#ifndef _BLUETOOTH_CLASSIC_H_
#define _BLUETOOTH_CLASSIC_H_

#include <stdint.h>
#include <stdarg.h>

// Para usar
// idf.py menuconfig
// components-> bluetooth
// enable bluedroid
// enable classic bluetooth and SPP in bluedroid options
// 

extern void bluetooth_init(char * btname);

extern void bluetooth_send(uint8_t * buff, int size, int * bw);
extern void bluetooth_send_string(char * buff, int * bw);

extern void bluetooth_vprintf(const char * buff, va_list vaptr);

/* returns >0 if available, 0 if no data and <0 if error */
extern int bluetooth_available(uint8_t * buff, int maxdata, int * br);

#endif /* _BLUETOOTH_CLASSIC_H_ */
