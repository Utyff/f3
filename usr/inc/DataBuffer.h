#ifndef _DATABUFFER_H
#define _DATABUFFER_H

#include "lcd.h"

#define BUF_SIZE 4096
extern u8 samplesBuffer[BUF_SIZE];

extern u8 firstHalf; // first or second half of buffer writing
extern u8 samplesReady;

#endif //_DATABUFFER_H
