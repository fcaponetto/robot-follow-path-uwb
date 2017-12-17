#include <stdint.h>

#ifndef _DECA_TYPES_H_
#define _DECA_TYPES_H_

#ifndef uint8
#ifndef _DECA_UINT8_
#define _DECA_UINT8_
typedef unsigned char uint8;
#endif
#endif

#ifndef uint16
	#ifndef _DECA_UINT16_
		#define _DECA_UINT16_
			typedef unsigned short uint16;
		#endif
#endif

#ifndef uint32
#ifndef _DECA_UINT32_
#define _DECA_UINT32_
typedef unsigned long uint32;
#endif
#endif

#ifndef uint64 
#ifndef _DECA_uint64_
#define _DECA_uint64_
typedef unsigned long long uint64 ;
#endif
#endif


#ifndef int16
#ifndef _DECA_INT16_
#define _DECA_INT16_
typedef signed short int16;
#endif
#endif

#ifndef int32
#ifndef _DECA_INT32_
#define _DECA_INT32_
typedef signed long int32;
#endif
#endif

//typedef uint64_t        uint64 ;

typedef int64_t         int64 ;


#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif

#endif /* DECA_TYPES_H_ */


