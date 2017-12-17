#ifndef _LIST_H_
#define _LIST_H

#include <stdio.h>
#include <stdlib.h>

typedef struct node {
    int val;
    struct node * next;
} node_t;

void push_end(node_t * head, int val);

void push_begin(node_t ** head, int val);

int pop_first(node_t ** head);

int remove_last(node_t * head);

void print_list(node_t * head);

#endif