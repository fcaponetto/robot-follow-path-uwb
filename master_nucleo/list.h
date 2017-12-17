#ifndef _LIST_H_
#define _LIST_H

#include <stdio.h>
#include <stdlib.h>

#define LISTAVUOTA NULL

typedef struct{
  float distance_left;
  float distance_right;
}tipobaseList;

typedef short boolean;

typedef struct nodoList {
  tipobaseList info;
  struct nodoList *next;
} *list;

typedef list position;

position End(list l);

position First(list l);

void MakeNullList(list *l);

boolean EmptyList(list l);

boolean FullList(list l);

void InsertList(list *l, position p, tipobaseList x);

void DeleteList(list *l, position p);

position Locate(list l, tipobaseList x);

tipobaseList Retrieve(list l, position p);

position Next(list l, position p);

#endif