#ifndef CONVERSION_H
#define CONVERSION_H

#include "deca_internals.h"
#include "deca_types.h"
#include <Arduino.h>

class Conversion{
public:
	static void Array64ToInt64(byte input[], unsigned int length, uint64 *output); /* the same endianess */
	static void Array64BigEndianToInt64LittleEndian(byte input[], unsigned int length, uint64 *output);
	static void Int64LittleEndianToArray64BigEndian(byte output[], unsigned int length, uint64 input);
	static void convertValueToBytes(byte data[], long val, unsigned int n);
	//int addition(byte op1[], byte op2[], byte res[], unsigned int length);
	//int subtraction(byte op1[], byte op2[], byte res[], unsigned int length);
	
	static void Print5BytesBigEndian(byte [], bool newline);
	static void Print5BytesLittleEndian(byte [], bool newline);
	
	static void PrintSeconds(byte time[]);
};

#endif