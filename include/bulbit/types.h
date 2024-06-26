#pragma once

typedef signed char int8;
typedef signed short int16;
typedef signed int int32;
typedef signed long long int64;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long uint64;

#ifdef BULBIT_DOUBLE_PRECISION
typedef double Float;
#else
typedef float Float;
#endif