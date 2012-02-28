#ifndef DATA_UNION_H
#define DATA_UNION_H

#include  <string>
#include <stdint.h>

//Definition of standard variable to control all available the functions
typedef struct DataUnion {
   uint8_t b;
   unsigned short h;
   float f;
   long l;
   std::string s;
} dataUnion;

#endif
