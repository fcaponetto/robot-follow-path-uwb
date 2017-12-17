#ifndef _BUFFER_SHIFT_H_
#define _BUFFER_SHIFT_H_

#define N 16
#define BUFFEREMPTY 0

typedef double typebaseBuffer;
typedef short boolean;

typedef struct {
    typebaseBuffer values[N];
    int top;
} buffer;

void MakeNullBuffer(buffer *s);

boolean EmptyBuffer(buffer s);

boolean FullBuffer(buffer s);

void ShiftValues(buffer *s);

void Push(buffer *s, typebaseBuffer x);

typebaseBuffer ReadValues(buffer *s, boolean * end);

#endif