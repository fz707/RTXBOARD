#ifndef PRIOQUEUE_H
#define PRIOQUEUE_H

#include "printf.h"

typedef struct Node{
	struct Node* prev;
	struct Node* next;
	U32 tid;
} *Node_t;

typedef struct Bucket{
	Node_t tail;
	Node_t head;
} *Bucket_t;

typedef struct PrioQueue{
	Bucket_t *buckets;
	Node_t *map;
} *PrioQueue;


PrioQueue	pq_create					(void);
void		pq_print					(PrioQueue pq);
int			pq_get_highest_priority 	(PrioQueue pq);
int			pq_push						(PrioQueue pq, TCB *tcb);
void		pq_pop						(PrioQueue pq, TCB *tcb);

#endif
