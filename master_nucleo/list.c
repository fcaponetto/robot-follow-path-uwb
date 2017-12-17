#include "list.h"

position End(list l){
   if (l==LISTAVUOTA) return(LISTAVUOTA);
   while (l->next!=LISTAVUOTA)
      l=l->next;
   return(l);
}

position First(list l) {
  return(LISTAVUOTA); /*se la lista è vuota torna END(l) */
}

void MakeNullList(list *l){
  *l=LISTAVUOTA;
}

boolean EmptyList(list l){
  return(l==LISTAVUOTA);
}

boolean FullList(list l){
  return(0);
}

void InsertList(list *l, position p, tipobaseList x){
  struct nodoList * temp;

  if (!FullList(*l)) {
    temp=(struct nodoList *) malloc(sizeof(struct nodoList));
    temp->info=x;
    if (p==LISTAVUOTA) {
	    temp->next=*l;
	    *l=temp;
    } else {
	    temp->next=p->next;
	    p->next=temp;
    }
  }  
}

void DeleteList(list *l, position p){
  struct nodoList * tmp;

  if (!EmptyList(*l) && p!=End(*l)) {
    if (p==LISTAVUOTA) {
            tmp=(*l)->next;
            free (*l);
            *l=tmp;
    } else {
            tmp=p->next;
            p->next=tmp->next;
            free (tmp);
    }
  }
}

position Locate(list l, tipobaseList x){
  if (!EmptyList(l)) {
    if (!Confronta(l->info,x)) return(LISTAVUOTA);
    while (l->next!=LISTAVUOTA) {
      if (!Confronta(l->next->info,x)) return(l);
      l=l->next;
    }
    return(l);
 }
}

tipobaseList Retrieve(list l, position p) {
  if (!EmptyList(l) && p!=End(l)) {
    if (p==LISTAVUOTA) return(l->info);
    else return(p->next->info);
  }
}

position Next(list l, position p) {
 if (p!=End(l)) {
   if (p==LISTAVUOTA) return(l);
   else return(p->next);
 }
}

