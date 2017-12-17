#include "conversion.h"

void Conversion::Array64ToInt64(byte input[], unsigned int length, uint64 *output){
	int i;
	for(i = 0; i < length ; i++){
		*output |= input[i] << (i*8);
	}
}

// input[] big endian output little endian
void Conversion::Array64BigEndianToInt64LittleEndian(byte input[], unsigned int length, uint64 *output){
	int j;
	for(j=length-1; j >= 0; j--){
		*output = (*output << 8) + input[j];
		//*output |= (uint64 )(input[j] << (j*8));
	}
}

void Conversion::Int64LittleEndianToArray64BigEndian(byte output[], unsigned int length, uint64 input){
	/*int i;
	for(i = 0; i < length; i++)
		output[i] = (uint64)((input >> (i * 8)) & 0xFF);*/
	int j;
	for (j = 0; j < length; j++)
	{
		output[j] = input & 0xff;
		input >>= 8;
	}
}

void Conversion::PrintSeconds(byte time[]){
	
	uint64 picoSecondi;
	double secondi;
	Array64BigEndianToInt64LittleEndian(time, 5, &picoSecondi);
	uint32 temp = (int32) picoSecondi;	
	secondi = temp * 0.000000000001;
	temp = time[4];
	secondi = secondi +  (temp * 0.0001);
	
	Serial.print("Secondi: "); Serial.print(secondi, 10); Serial.println("");
}

void Conversion::Print5BytesBigEndian(byte buffer[], bool newline){
	Serial.print(buffer[4], HEX); Serial.print("\t");
	Serial.print(buffer[3], HEX); Serial.print("\t");
	Serial.print(buffer[2], HEX); Serial.print("\t");
	Serial.print(buffer[1], HEX); Serial.print("\t");
	Serial.print(buffer[0], HEX); Serial.print("\t");
	if(newline)
		Serial.println("");
}

void Conversion::Print5BytesLittleEndian(byte buffer[], bool newline){
	Serial.print(buffer[0], HEX); Serial.print("\t");
	Serial.print(buffer[1], HEX); Serial.print("\t");
	Serial.print(buffer[2], HEX); Serial.print("\t");
	Serial.print(buffer[3], HEX); Serial.print("\t");
	Serial.print(buffer[4], HEX); Serial.print("\t");
	if(newline)
	Serial.println("");
}

void Conversion::convertValueToBytes(byte data[], long val, unsigned int n) {
	int i;
	for(i = 0; i < n; i++) {
		data[i] = ((val >> (i * 8)) & 0xFF);
	}
}


/*
void Conversion::convertIntLittleEndianToBytesBigEndian(uint64 output[], unsigned int length, uint64 input){
	//int i;
	//for(i = 0; i < length; i++)
	//	output[i] = (uint64)((input >> (i * 8)) & 0xFF);
	int j;
	for (j = 0; j < length - 1; j++)
	{
		output[j] = input & 0xff;
		input >>= 8;
	}
}
*/

/*
int Conversion::addition(byte op1[], byte op2[], byte res[], unsigned int length){
	int i; 
	byte carry = 0x00;
	for(i = 0; i < length; i ++){
		res[i] = op1[i] + op2[i] + carry;
		if((op1[i] & 0x80  && op2[i] & 0x80) || ((op1[i] & 0x80  || op2[i] & 0x80) && !(res[i] & 0x80))){
			carry = 0x01;
			if(i+1 >= length){
				return DWT_ERROR;
			}
		}
	}	
	return DWT_SUCCESS;
}

int Conversion::subtraction(byte op1[], byte op2[], byte res[], unsigned int length){
	int i; 
	byte borrow = 0x00;
	for(i = 0; i < length; i ++){
		res[i] = op1[i] - op2[i] - borrow;
		if((op1[i] & 0x00  && op2[i] & 0x80) || (op1[i] < op2[i])){
			borrow = 0x01;
			if(i+1 >= length){
				return DWT_ERROR;
			}	
		}
	}	
	return DWT_SUCCESS;
}
*/