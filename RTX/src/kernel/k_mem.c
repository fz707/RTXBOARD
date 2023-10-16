/*
 ****************************************************************************
 *
 *                  UNIVERSITY OF WATERLOO ECE 350 RTOS LAB
 *
 *                     Copyright 2020-2021 Yiqing Huang
 *                          All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice and the following disclaimer.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************
 */

/**************************************************************************//**
 * @file        k_mem.c
 * @brief       Kernel Memory Management API C Code
 *
 * @version     V1.2021.01.lab2
 * @authors     Yiqing Huang
 * @date        2021 JAN
 *
 * @note        skeleton code
 *
 *****************************************************************************/

/** 
 * @brief:  k_mem.c kernel API implementations, this is only a skeleton.
 * @author: Yiqing Huang
 */

#include "k_mem.h"
#include "Serial.h"
#include "printf.h"
#include "common_ext.h"

#ifdef DEBUG_0

#endif  /* DEBUG_0 */

/*
 *==========================================================================
 *                            GLOBAL VARIABLES
 *==========================================================================
 */

extern TCB *gp_current_task;
// kernel stack size, referred by startup_a9.s
const U32 g_k_stack_size = K_STACK_SIZE;
// task proc space stack size in bytes, referred by system_a9.cs
const U32 g_p_stack_size = U_STACK_SIZE;

// task kernel stacks
U32 g_k_stacks[MAX_TASKS][K_STACK_SIZE >> 2] __attribute__((aligned(8)));

//process stack for tasks in SYS mode
U32 g_p_stacks[MAX_TASKS][U_STACK_SIZE >> 2] __attribute__((aligned(8)));

/*
 *===========================================================================
 *                            FUNCTIONS
 *===========================================================================
 */

typedef struct Tag{			//8b
	U32 owner;
	U32 size;
} Tag_t;

typedef struct Free_tag { 	//12b
	Tag_t header;
	U32 *next;
} Free_tag_t;

static Free_tag_t *SENTINEL_HEAD = NULL;
static const Free_tag_t * const SENTINEL_TAIL = (Free_tag_t *) (RAM_END+1);

__forceinline int get_size(Tag_t *h_tag) {
	return h_tag->size;
}

__forceinline int get_size_free(Free_tag_t *h_tag) {
	return h_tag->header.size;
}

__forceinline void set_size(Tag_t *h_tag, int size) {
	h_tag->size = size;
}

__forceinline void set_size_free(Free_tag_t *h_tag, int size) {
	h_tag->header.size = size;
}

__forceinline void set_owner(Tag_t *h_tag, int tid) {
	h_tag->owner = tid;
}

__forceinline void set_owner_free(Free_tag_t *h_tag, int tid) {
	h_tag->header.owner = tid;
}

__forceinline Tag_t *traverse_next(Tag_t *h_tag) {
	return (Tag_t*)((U32)h_tag + (U32) get_size(h_tag));
}

__forceinline Free_tag_t *traverse_next_free(Free_tag_t *h_tag) {
	return (Free_tag_t*)(h_tag->next);
}

__forceinline void set_next_free(Free_tag_t *free_tag, U32 *address) {
	free_tag->next = address;
}

U32* k_alloc_k_stack(task_t tid)
{
    return g_k_stacks[tid+1];
}


U32* k_alloc_p_stack(task_t tid)
{
	return k_mem_alloc(g_tcbs[tid].u_stack_size);
}


int k_mem_init(void) {
	U32 heap_begin = (U32 ) &Image$$ZI_DATA$$ZI$$Limit;							// Retrieve beginning address
	heap_begin = (heap_begin+7)&(~0x7);
	U32 *heap_begin_addr = (void *) heap_begin;

    if(SENTINEL_HEAD || heap_begin_addr >= (U32 *) RAM_END) return RTX_ERR;				// Check for no space case

    Free_tag_t *first = (Free_tag_t *) (heap_begin_addr+2);				// Designate the second 8 bytes of free space to be the first free node

    SENTINEL_HEAD = (Free_tag_t *) heap_begin_addr;							// Designate the first 8 bytes of free space to be the sentinel head
    set_size((Tag_t *) SENTINEL_HEAD, 8);									// Set size accordingly
    set_next_free(SENTINEL_HEAD, (U32 *) first);							// Point sentinel head to our first free node
    printf("\n\nAvail size: %d\n\n", (U32)((RAM_END+1) - (U32)heap_begin_addr - sizeof(Free_tag_t)));
    set_size_free(first, (U32)((RAM_END+1) - (U32)heap_begin_addr - sizeof(Free_tag_t)));	// Set the first free node to cover available heap space

    set_next_free(first, (U32 *) SENTINEL_TAIL);							// Point said node to the sentinel tail

    return RTX_OK;
}

// contains exact malloc logic, but now also sets ownership, is called by og malloc
void * k_mem_alloc_wrapper(task_t tid, size_t size ){

    if(!SENTINEL_HEAD || size <= 0) return NULL;												// Invalid call check
    size += sizeof(Tag_t); 													// Adding 4 to size to account for header bytes

    Free_tag_t *prev = SENTINEL_HEAD;										// Assign state variables
    Free_tag_t *curr = (Free_tag_t *) SENTINEL_HEAD->next;

    U32 padded_size = (size+7)&(~0x7);										// Round to next 4 byte alignment

    while(curr != SENTINEL_TAIL) {											// Iterate until end
    	if(get_size_free(curr) >= padded_size){								// Check if block is valid for alloc based on size

    		int old_size = get_size_free(curr);								// Save old state for calculations
    		Free_tag_t *nextfree = traverse_next_free(curr);				// Save next free block relative to the current block

    		set_size_free(curr, padded_size);								// Set size to padded and rounded size
    		set_owner_free(curr, tid);										// Set owner to the tid that was passed to the wrapper

    		Tag_t *return_tag = (Tag_t *)curr;								// Cast the free tag into a normal tag

    		if(old_size >= padded_size+16) {									// Case: sizes unequal and we must fracture free block into two
    																		// -> also, make sure we don't allow the user to allocate an empty block

    			Free_tag_t *nextshort = traverse_next_free(curr);			// Traverse to next free block, save
    			nextfree = (Free_tag_t *) traverse_next((Tag_t *) curr);	// Traverse to the newly created free block, save

				set_size_free(nextfree, old_size-padded_size);				// Set size of said free block according to last size and alloc'd size

				prev->next = (U32*) nextfree;					// If we traversed to the middle of two free blocks, set last free block-> next																		// to new fractured free block
				set_next_free(nextfree, (U32*) nextshort);		// Set the new fractured free block's next to the next free block after it
    		}

    		else {												// Case: perfect alloc size, curr exists somewhere in the middle of the list
    			set_next_free(prev, (U32*) nextfree);			// Assign the previous node relative to current to the node after current
    		}

    		return (void *) (return_tag+1);								// Return casted incremented address
    	}
    	else { prev = curr; curr = traverse_next_free(curr); }			// Iterate both ptrs at once
    }

	return NULL;
}

void* k_mem_alloc(size_t size) {
	return k_mem_alloc_wrapper(gp_current_task->tid, size);
}

// contains exact dealloc logic, but now also sets ownership, is called by og dealloc
int k_mem_dealloc_wrapper(task_t tid, void *ptr ){

	if(!SENTINEL_HEAD || !ptr || ptr > (U32 *) RAM_END) return RTX_ERR;

    Free_tag_t *lower_bound = NULL;
    Free_tag_t *upper_bound = SENTINEL_HEAD;

    // ESTABLISH BOUNDS
    while(upper_bound < ptr) {
    	lower_bound = upper_bound;
    	upper_bound = traverse_next_free(upper_bound);
    }

    // VALIDATE ADDRESS
    if(lower_bound == NULL) return RTX_ERR;

    int flag = 0;
    Tag_t *valid = (Tag_t *) lower_bound;
    while(valid < (Tag_t *) upper_bound) {
    	valid = traverse_next(valid);
    	if(valid+1 == ptr) { flag = 1; break; }
    	else if(valid > ptr) break;
    }
    if(!flag) return RTX_ERR;

    if(valid->owner != tid) return RTX_ERR;

    // DETERMINE COALESCENCE
    int coalesce_backwards = 0;
    int coalesce_forwards = 0;

    if(!(lower_bound == SENTINEL_HEAD) && (Tag_t *) traverse_next((Tag_t *) lower_bound) == ((Tag_t *) ptr-1)) coalesce_backwards = 1;
    if(!(upper_bound == SENTINEL_TAIL) && (Tag_t *) traverse_next(valid) == (Tag_t *) upper_bound) coalesce_forwards = 1;

    if(coalesce_backwards && coalesce_forwards) {
    	set_size_free(lower_bound, get_size_free(lower_bound) + get_size(valid) + get_size_free(upper_bound));	// Set the previous size to the 3 block amalgamate size
		set_next_free(lower_bound, (U32 *) traverse_next_free(upper_bound));							// Sever link and set previous free addr to next after block end
    }
    else if(coalesce_backwards) {
    	set_size_free(lower_bound, get_size_free(lower_bound)+get_size(valid)); 								// Merge into previous header
    }
    else if(coalesce_forwards) {
    	set_size(valid, get_size(valid)+get_size_free(upper_bound)); 									// Merge into current header
		set_next_free((Free_tag_t *) valid, (U32 *) traverse_next_free(upper_bound));			// Sever link and set current next free to the one after
		set_next_free(lower_bound, (U32 *) valid);
    }
    else {
    	set_next_free(lower_bound, (U32 *) valid);
    	set_next_free((Free_tag_t *) valid, (U32 *) upper_bound);
    }
    return RTX_OK;
}

int k_mem_dealloc(void *ptr) {
	return k_mem_dealloc_wrapper(gp_current_task->tid, ptr);
}

int k_mem_count_extfrag(size_t size) {
#ifdef DEBUG_0
    printf("k_mem_extfrag: size = %d\r\n", size);
#endif /* DEBUG_0 */

    Free_tag_t *curr = (Free_tag_t *) SENTINEL_HEAD->next;

    int res = 0;
    while(curr != SENTINEL_TAIL) {
    	if(get_size_free(curr) < size) res++;
    	curr = traverse_next_free(curr);
    }
    return res;
}

/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */
//C:/software/Altera/15.1/quartus/bin64/quartus_pgm.exe -c 1 -m jtag -o "p;C:/Users/lwiecek/Desktop/group32-lab/DE1-SoC\DE1_SoC_Computer.sof@2"
//C:/software/Altera/15.1/quartus/bin64/quartus_hps.exe -c 1 -o GDBSERVER --gdbport0=3008 --preloader=C:/Users/lwiecek/Desktop/group32-lab/DE1-SoC/de1-soc.srec --preloaderaddr=0xffff13a0
