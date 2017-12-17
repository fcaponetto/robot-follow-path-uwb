#include "buffershift.h"

void MakeNullBuffer(buffer *s) {
  s->top=BUFFEREMPTY;
}

boolean EmptyBuffer(buffer s){
  return(s.top==BUFFEREMPTY);
}

boolean FullBuffer(buffer s){
  return(s.values[N-1] != 0);
}
static int k = 0;
void ShiftValues(buffer *s){
  for (k = 0; k < N-3; k++){
    typebaseBuffer temp = s->values[k+1];
    s->values[k+1] = s->values[k];
    s->values[k+2] = temp;
  }
}

typebaseBuffer ReadValues(buffer *s, boolean *end){
  if(s->top != N-1){
    *end = 0;
    s->top++;
    return s->values[s->top];
  }
  else
    *end = 1;
}

void Push(buffer *s, typebaseBuffer x){
  ShiftValues(s);
  s->values[0] = x;
}
