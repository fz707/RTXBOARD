#ifndef CIRCQUEUE_H
#define CIRCQUEUE_H

#include "Serial.h"
#include "printf.h"
#include "common.h"

extern void* k_mem_alloc_wrapper(task_t, size_t);

typedef struct CircQueue{
	U32 capacity;
	U32 size;
	U32 first;
	U32 last;
	char *arr;
} CircQueue;

CircQueue	*cq_create					(U32 size);
void		cq_print					(CircQueue *cq);
U32			cq_push						(CircQueue *cq, void *buf, U32 tid);
U32 		cq_pop						(CircQueue *cq, void *out, size_t buf_length);
U32			cq_getremaining				(CircQueue *cq);

void 		memcpy						(void *dest, void *old, U32 size);
void 		bounded_memcpy_in			(void *dest, void *old, U32 size, U32 bound, U32 start);
void 		bounded_memcpy_out			(void *dest, void *old, U32 size, U32 bound, U32 start);

inline void memcpy(void *dest, void *old, U32 size) {
	char *n_dest = (char *) dest;
	char *n_old = (char *) old;
	for(int i = 0; i < size; i++) n_dest[i] = n_old[i];
}

inline void bounded_memcpy_in(void *dest, void *old, U32 size, U32 bound, U32 start) {
	char *n_dest = (char *) dest;
	char *n_old = (char *) old;
	for(int i = 0; i < size; i++) n_dest[(start+i)%bound] = n_old[i];
}

inline void bounded_memcpy_out(void *dest, void *old, U32 size, U32 bound, U32 start) {
	char *n_dest = (char *) dest;
	char *n_old = (char *) old;
	for(int i = 0; i < size; i++) n_dest[i] = n_old[(start+i)%bound];
}

__forceinline U32 cq_getremaining(CircQueue *cq) { return (cq->capacity - cq->size); }

__forceinline task_t cq_get_tid(RTX_MSG_HDR *buf) {
	return buf->type & 0x00ffffff;
}

__forceinline U32 cq_get_type(RTX_MSG_HDR *buf) {
	return buf->type >> 24;
}


inline CircQueue *cq_create(U32 size) {
	CircQueue *cq = (CircQueue *) k_mem_alloc_wrapper(TID_NULL, sizeof(CircQueue));
	if(cq == NULL) {
		return NULL;
	}
	char *arr = (char*) k_mem_alloc_wrapper(TID_NULL, size);
	if(arr == NULL) {
		k_mem_dealloc_wrapper( TID_NULL, cq );
		return NULL;
	}
	cq->capacity = size;
	cq->arr = arr;
	cq->size = 0; cq->first = 0; cq->last = 0;

	for(int i = 0; i < cq->capacity; i++) arr[i] = 0;

	return cq;
}

inline void cq_print(CircQueue *cq) {
	printf("Beginning CQ print:\r\n");
	for(int i=0; i < cq->capacity; i++) printf("%d ", cq->arr[i]);
	printf("END\r\n");
}

// 1 = good, 0 = bad
inline U32 cq_push(CircQueue *cq, void *buf, U32 tid) {
	U32 len; U32 type;
	RTX_MSG_HDR *temp = (RTX_MSG_HDR *) buf;
	len = temp->length;
	type = temp->type;

	// If queue full OR message size > remaining size, error out
	if(cq->size == cq->capacity || len > cq_getremaining(cq)) return 0;

	// Type field now also contains TID
	temp->type = tid | (type << 24);

	// Read in message
	bounded_memcpy_in(cq->arr, buf, len, cq->capacity, cq->last);

	temp->type = temp->type >> 24;
	cq->last = (cq->last+len)%cq->capacity;
	cq->size += len;

//	cq_print(cq);
	return 1;
}

// 1 = good, 0 = bad
inline U32 cq_pop(CircQueue *cq, void *out, size_t buf_length) {
	// If queue empty, error out
	if(cq->size == 0) return 0;

	// Read out size of message to pop, then use size to read out full message
	U32 len;
	bounded_memcpy_out(&len, cq->arr, 4, cq->capacity, cq->first);
	// check size of buffer, if less than message size, error out
	bounded_memcpy_out(out, cq->arr, len, cq->capacity, cq->first);

	cq->first = (cq->first+len)%cq->capacity;
	cq->size -= len;

	if(buf_length < len) return 0;


	return 1;
}

#endif
