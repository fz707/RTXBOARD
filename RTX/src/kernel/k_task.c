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
 * @file        k_task.c
 * @brief       task management C file
 *              l2
 * @version     V1.2021.01
 * @authors     Yiqing Huang
 * @date        2021 JAN
 *
 * @attention   assumes NO HARDWARE INTERRUPTS
 * @details     The starter code shows one way of implementing context switching.
 *              The code only has minimal sanity check.
 *              There is no stack overflow check.
 *              The implementation assumes only two simple privileged task and
 *              NO HARDWARE INTERRUPTS.
 *              The purpose is to show how context switch could be done
 *              under stated assumptions.
 *              These assumptions are not true in the required RTX Project!!!
 *              Understand the assumptions and the limitations of the code before
 *              using the code piece in your own project!!!
 *
 *****************************************************************************/

//#include "VE_A9_MP.h"
#include "Serial.h"
#include "k_task.h"
#include "k_rtx.h"
//#include "PrioQueue.h"

#include "printf.h"

#ifdef DEBUG_0
#include "printf.h"
#endif /* DEBUG_0 */

/*
 *==========================================================================
 *                            GLOBAL VARIABLES
 *==========================================================================
 */

TCB             *gp_current_task = NULL;	// the current RUNNING task
TCB             g_tcbs[MAX_TASKS];			// an array of TCBs
RTX_TASK_INFO   g_null_task_info;			// The null task info
U32             g_num_active_tasks = 0;		// number of non-dormant tasks
TCB 			*first_free_tcb = NULL; 		// first free TCB
PrioQueue		pq = NULL;
extern void kcd_task(void);


/*---------------------------------------------------------------------------
The memory map of the OS image may look like the following:

                       RAM_END+---------------------------+ High Address
                              |                           |
                              |                           |
                              |    Free memory space      |
                              |   (user space stacks      |
                              |         + heap            |
                              |                           |
                              |                           |
                              |                           |
 &Image$$ZI_DATA$$ZI$$Limit-->|---------------------------|-----+-----
                              |         ......            |     ^
                              |---------------------------|     |
                              |      U_STACK_SIZE         |     |
             g_p_stacks[15]-->|---------------------------|     |
                              |                           |     |
                              |  other kernel proc stacks |     |
                              |---------------------------|     |
                              |      U_STACK_SIZE         |  OS Image
              g_p_stacks[2]-->|---------------------------|     |
                              |      U_STACK_SIZE         |     |
              g_p_stacks[1]-->|---------------------------|     |
                              |      U_STACK_SIZE         |     |
              g_p_stacks[0]-->|---------------------------|     |
                              |   other  global vars      |     |
                              |                           |  OS Image
                              |---------------------------|     |
                              |      K_STACK_SIZE         |     |                
             g_k_stacks[15]-->|---------------------------|     |
                              |                           |     |
                              |     other kernel stacks   |     |                              
                              |---------------------------|     |
                              |      K_STACK_SIZE         |  OS Image
              g_k_stacks[2]-->|---------------------------|     |
                              |      K_STACK_SIZE         |     |                      
              g_k_stacks[1]-->|---------------------------|     |
                              |      K_STACK_SIZE         |     |
              g_k_stacks[0]-->|---------------------------|     |
                              |   other  global vars      |     |
                              |---------------------------|     |
                              |        TCBs               |  OS Image
                      g_tcbs->|---------------------------|     |
                              |        global vars        |     |
                              |---------------------------|     |
                              |                           |     |          
                              |                           |     |
                              |                           |     |
                              |                           |     V
                     RAM_START+---------------------------+ Low Address
    
---------------------------------------------------------------------------*/ 

/*
 *===========================================================================
 *                            FUNCTIONS
 *===========================================================================
 */

/**************************************************************************//**
 * @brief   scheduler, pick the TCB of the next to run task
 *
 * @return  TCB pointer of the next to run task
 * @post    gp_curret_task is updated
 *
 *****************************************************************************/



/*
 *===========================================================================
 *                            PRIORITY QUEUE
 *===========================================================================
 */

PrioQueue pq_create() {
	PrioQueue pq = k_mem_alloc(8);
	Bucket_t *buckets = k_mem_alloc(256*4);
	Node_t *map = k_mem_alloc((MAX_TASKS)*4);


	for(int i = 0; i < 256; i++) {
		buckets[i] = k_mem_alloc(8);
		buckets[i]->head = NULL;
		buckets[i]->tail = NULL;
	}

	for(int i = 0; i < (MAX_TASKS); i++) {
		map[i] = k_mem_alloc(12);
		map[i]->next = NULL;
		map[i]->prev = NULL;
		map[i]->tid = i;
	}

	pq->buckets = buckets;
	pq->map = map;

	return pq;
}

void pq_print(PrioQueue pq) {
	Node_t curr;
	Node_t next;
	for(int i=0; i < 256; i++) {
		printf("Priority %d: ", i);
		curr = pq->buckets[i]->head;
		next = NULL;
		while(curr != NULL) {
			printf("%d -> ", curr->tid);
			next = curr->next;
			curr = next;
		}
		printf("END\r\n");
	}
}

int pq_get_highest_priority(PrioQueue pq) {
	for(int i = 0; i < 256; i++) {
		if(pq->buckets[i]->head) {
			return pq->buckets[i]->head->tid;
		}
	}
	return TID_NULL; // null task
}

int pq_push(PrioQueue pq, TCB *tcb) {
	int priority = tcb->prio;
	int tid = tcb->tid;

	Node_t node = pq->map[tid];
	node->tid = tid;

	// List non-empty
	if(pq->buckets[priority]->tail) {
		pq->buckets[priority]->tail->next = node;
		node->prev = pq->buckets[priority]->tail;
		pq->buckets[priority]->tail = node;
	}
	// List empty
	else {
		pq->buckets[priority]->head = node;
		pq->buckets[priority]->tail = node;
	}

	return 1;
}

void pq_pop(PrioQueue pq, TCB *tcb) {
	int priority = tcb->prio;
	int tid = tcb->tid;

	Node_t node = pq->map[tid];
	Node_t prev = node->prev;
	Node_t next = node->next;

	// Node is the only one in list
	if(pq->buckets[priority]->head == pq->buckets[priority]->tail) {
		pq->buckets[priority]->head = NULL;
		pq->buckets[priority]->tail = NULL;
	}
	// Node is head of linked list
	else if(pq->buckets[priority]->head == node) {
		pq->buckets[priority]->head = node->next;
	}
	// Node is tail of linked list
	else if(pq->buckets[priority]->tail == node) {
		pq->buckets[priority]->tail = node->prev;
	}
	// General case
	else {
		prev->next = next;
		next->prev = prev;
	}

	node->next = NULL;
	node->prev = NULL;
}

void tcb_list_print() {
	for (int i = 0; i < MAX_TASKS; i++){
		printf("tcb %d: tid: %d, prio: %d, state: %d, priv: %d\r\n", i, g_tcbs[i].tid, g_tcbs[i].prio, g_tcbs[i].state, g_tcbs[i].priv);
	}
}

void tcb_print(task_t tid) {
	if(tid > 0 && tid <= MAX_TASKS - 1){
		printf("tcb %d: tid: %d, prio: %d, state: %d, priv: %d\r\n", tid, g_tcbs[tid].tid, g_tcbs[tid].prio, g_tcbs[tid].state, g_tcbs[tid].priv);
	}
	return;
}


__forceinline TCB *scheduler(void)
{
    return &g_tcbs[pq_get_highest_priority(pq)];
}



/**************************************************************************//**
 * @brief       initialize all boot-time tasks in the system,
 *
 *
 * @return      RTX_OK on success; RTX_ERR on failure
 * @param       task_info   boot-time task information structure pointer
 * @param       num_tasks   boot-time number of tasks
 * @pre         memory has been properly initialized
 * @post        none
 *
 * @see         k_tsk_create_new
 *****************************************************************************/

int k_tsk_init(RTX_TASK_INFO *task_info, int num_tasks)
{
    extern U32 SVC_RESTORE;
    pq = pq_create();

    RTX_TASK_INFO *p_taskinfo = &g_null_task_info;
    g_num_active_tasks = 0;

    if(num_tasks >= MAX_TASKS) return RTX_ERR;

    for(int i = 0; i < MAX_TASKS; i++) {		// Initialize free pointers of all TCBs
    	if(i != TID_KCD){
    		if(i == TID_KCD-1){
    			if(TID_KCD == MAX_TASKS - 1){
    				g_tcbs[i].next = NULL;
    			} else {
    				g_tcbs[i].next = &g_tcbs[i+2];
    			}
    		} else if (i == MAX_TASKS-1){
    			g_tcbs[i].next = NULL;
    		} else {
    			g_tcbs[i].next = &g_tcbs[i+1];
    		}
    	}
//    	if(i == MAX_TASKS - 1) g_tcbs[i].next = NULL;
//    	else g_tcbs[i].next = &g_tcbs[i+1];

    	g_tcbs[i].tid = i;
    	g_tcbs[i].state = DORMANT;
    	g_tcbs[i].mailbox = NULL;
    }

    TCB *p_tcb = &g_tcbs[0];				// Create first task
    p_tcb->prio     = PRIO_NULL;
    p_tcb->priv     = 1;
    p_tcb->tid      = TID_NULL;
    p_tcb->state    = RUNNING;

    g_num_active_tasks++;
	first_free_tcb = g_tcbs[0].next; 		// Set pointer to first free TCB in array
    gp_current_task = p_tcb;
    pq_push(pq, p_tcb);

    int kcd_created = 0;

    p_taskinfo = task_info;
    for(int i = 0; i < num_tasks; i++) {	// Create rest of tasks
        p_tcb = first_free_tcb;
        if(i == MAX_TASKS-1 && !kcd_created) return RTX_ERR;
        if(p_taskinfo->ptask == &kcd_task){
        	p_tcb = &g_tcbs[TID_KCD];
        	if(k_tsk_create_new(p_taskinfo, p_tcb, TID_KCD) == RTX_OK){
            	kcd_created = 1;
            	g_num_active_tasks++;
        	}
        }
        else{
            if(k_tsk_create_new(p_taskinfo, p_tcb, first_free_tcb->tid) == RTX_OK) {
            	g_num_active_tasks++;
            	first_free_tcb = p_tcb->next; 		// Set first free pointer to next free TCB in array
            }
        }
        pq_push(pq, p_tcb);
        p_taskinfo++;
    }

    return RTX_OK;
}
/**************************************************************************//**
 * @brief       initialize a new task in the system,
 *              one dummy kernel stack frame, one dummy user stack frame
 *
 * @return      RTX_OK on success; RTX_ERR on failure
 * @param       p_taskinfo  task information structure pointer
 * @param       p_tcb       the tcb the task is assigned to
 * @param       tid         the tid the task is assigned to
 *
 * @details     From bottom of the stack,
 *              we have user initial context (xPSR, PC, SP_USR, uR0-uR12)
 *              then we stack up the kernel initial context (kLR, kR0-kR12)
 *              The PC is the entry point of the user task
 *              The kLR is set to SVC_RESTORE
 *              30 registers in total
 *
 *****************************************************************************/
int k_tsk_create_new(RTX_TASK_INFO *p_taskinfo, TCB *p_tcb, task_t tid)
{
    extern U32 SVC_RESTORE;

    U32 *sp;

    if (p_taskinfo == NULL || p_tcb == NULL) return RTX_ERR;

    p_tcb ->tid = tid;
    p_tcb->state = READY;
    p_tcb->u_stack_size = p_taskinfo->u_stack_size;
    p_tcb->pc = p_taskinfo->ptask;
    p_tcb->u_stack_hi = p_taskinfo->u_stack_hi;
    p_tcb->prio = p_taskinfo->prio;
    p_tcb->priv = p_taskinfo->priv;

    /*---------------------------------------------------------------
     *  Step1: allocate kernel stack for the task
     *         stacks grows down, stack base is at the high address
     * -------------------------------------------------------------*/

    ///////sp = g_k_stacks[tid] + (K_STACK_SIZE >> 2) ;
    sp = k_alloc_k_stack(tid);
    p_tcb->k_stack_hi = (U32) sp;

    // 8B stack alignment adjustment
    if ((U32)sp & 0x04) {   // if sp not 8B aligned, then it must be 4B aligned
        sp--;               // adjust it to 8B aligned
    }

    /*-------------------------------------------------------------------
     *  Step2: create task's user/sys mode initial context on the kernel stack.
     *         fabricate the stack so that the stack looks like that
     *         task executed and entered kernel from the SVC handler
     *         hence had the user/sys mode context saved on the kernel stack.
     *         This fabrication allows the task to return
     *         to SVC_Handler before its execution.
     *
     *         16 registers listed in push order
     *         <xPSR, PC, uSP, uR12, uR11, ...., uR0>
     * -------------------------------------------------------------*/

    // if kernel task runs under SVC mode, then no need to create user context stack frame for SVC handler entering
    // since we never enter from SVC handler in this case
    // uSP: initial user stack
    if ( p_taskinfo->priv == 0 ) { // unprivileged task
        // xPSR: Initial Processor State
        *(--sp) = INIT_CPSR_USER;
        // PC contains the entry point of the user/privileged task
        *(--sp) = (U32) (p_taskinfo->ptask);

        //********************************************************************//
        //*** allocate user stack from the user space, not implemented yet ***//
        //********************************************************************//
//        U32 * user_stack_start = k_alloc_p_stack(tid);
//        p_tcb->u_stack_hi = (U32) user_stack_start;
        U32 *user_stack_high = (U32 *)((U32)k_mem_alloc_wrapper(TID_NULL, p_taskinfo->u_stack_size) + p_taskinfo->u_stack_size); 	// Attempt to allocate required memory for user-space stack
        if (user_stack_high == NULL) return RTX_ERR;
        p_tcb->u_stack_hi = (U32) user_stack_high;
        *(--sp) = (U32) user_stack_high;

        // uR12, uR11, ..., uR0
        for ( int j = 0; j < 13; j++ ) {
            *(--sp) = 0x0;
        }
    }


    /*---------------------------------------------------------------
     *  Step3: create task kernel initial context on kernel stack
     *
     *         14 registers listed in push order
     *         <kLR, kR0-kR12>
     * -------------------------------------------------------------*/
    if ( p_taskinfo->priv == 0 ) {
        // user thread LR: return to the SVC handler
        *(--sp) = (U32) (&SVC_RESTORE);
    } else {
        // kernel thread LR: return to the entry point of the task
        *(--sp) = (U32) (p_taskinfo->ptask);
    }

    // kernel stack R0 - R12, 13 registers
    for ( int j = 0; j < 13; j++) {
        *(--sp) = 0x0;
    }

    // kernel stack CPSR
    *(--sp) = (U32) INIT_CPSR_SVC;
    p_tcb->ksp = sp;

    return RTX_OK;
}

/**************************************************************************//**
 * @brief       switching kernel stacks of two TCBs
 * @param:      p_tcb_old, the old tcb that was in RUNNING
 * @return:     RTX_OK upon success
 *              RTX_ERR upon failure
 * @pre:        gp_current_task is pointing to a valid TCB
 *              gp_current_task->state = RUNNING
 *              gp_crrent_task != p_tcb_old
 *              p_tcb_old == NULL or p_tcb_old->state updated
 * @note:       caller must ensure the pre-conditions are met before calling.
 *              the function does not check the pre-condition!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * @attention   CRITICAL SECTION
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *****************************************************************************/
__asm void k_tsk_switch(TCB *p_tcb_old)
{
        PUSH    {R0-R12, LR}
        MRS 	R1, CPSR
        PUSH 	{R1}
        STR     SP, [R0, #TCB_KSP_OFFSET]   ; save SP to p_old_tcb->ksp
        LDR     R1, =__cpp(&gp_current_task);
        LDR     R2, [R1]
        LDR     SP, [R2, #TCB_KSP_OFFSET]   ; restore ksp of the gp_current_task
        POP		{R0}
        MSR		CPSR_cxsf, R0
        POP     {R0-R12, PC}
}


/**************************************************************************//**
 * @brief       run a new thread. The caller becomes READY and
 *              the scheduler picks the next ready to run task.
 * @return      RTX_ERR on error and zero on success
 * @pre         gp_current_task != NULL && gp_current_task == RUNNING
 * @post        gp_current_task gets updated to next to run task
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * @attention   CRITICAL SECTION
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *****************************************************************************/
int k_tsk_run_new(void)
{
    TCB *p_tcb_old = NULL;	// May need to disable interrupts
    
    if (gp_current_task == NULL) return RTX_ERR;

    p_tcb_old = gp_current_task;
    gp_current_task = scheduler();
    
    if(gp_current_task == NULL) {
        gp_current_task = p_tcb_old;        // Revert back to the old task
        return RTX_ERR;
    }

    // At this point, gp_current_task != NULL and p_tcb_old != NULL
    if(gp_current_task != p_tcb_old) {
        gp_current_task->state = RUNNING;   	// Change state of the to-be-switched-in  tcb
        if(p_tcb_old->state != DORMANT && p_tcb_old->state != BLK_MSG){
            p_tcb_old->state = READY;           // Change state of the to-be-switched-out tcb
        }
        k_tsk_switch(p_tcb_old);            	// Switch stacks
    }

    return RTX_OK;
}

/**************************************************************************//**
 * @brief       yield the cpu
 * @return:     RTX_OK upon success
 *              RTX_ERR upon failure
 * @pre:        gp_current_task != NULL &&
 *              gp_current_task->state = RUNNING
 * @post        gp_current_task gets updated to next to run task
 * @note:       caller must ensure the pre-conditions before calling.
 *****************************************************************************/
__forceinline int k_tsk_yield(void)
{
	pq_pop(pq, gp_current_task);

	pq_push(pq, gp_current_task);

    return k_tsk_run_new();
}


/*
 *===========================================================================
 *                             TO BE IMPLEMETED IN LAB2
 *===========================================================================
 */

/**************************************************************************//**
 * @brief       create a new user task
 * @return:     RTX_OK upon success
 *              RTX_ERR upon failure
 * @pre:
 * @error_on 	task == NULL or task_entry == NULL or prio == 0 or prio >= 255 or stack_size % 8 != 0 or stack_size < U_STACK_SIZE or ??
 * @post        new TCB has been created and scheduler gets run to determine if preemption is required
 * @note:
 *****************************************************************************/
int k_tsk_create(task_t *task, void (*task_entry)(void), U8 prio, U16 stack_size)
{
#ifdef DEBUG_0
    printf("k_tsk_create: entering...\n\r");
    printf("task = 0x%x, task_entry = 0x%x, prio=%d, stack_size = %d\n\r", task, task_entry, prio, stack_size);
#endif /* DEBUG_0 */

    // Validity check
    if (task == NULL || task_entry == NULL || prio < 1 || prio >= 255 || stack_size % 8 != 0 || stack_size < U_STACK_SIZE || first_free_tcb == NULL )return RTX_ERR;

    TCB * new_tcb = first_free_tcb;				// Set the new TCB to be the first free TCB
    first_free_tcb = first_free_tcb->next;		// Set new first free pointer to next free block

    RTX_TASK_INFO new_task_info;				// Create and populate new task info struct
    new_task_info.ptask = task_entry;
    new_task_info.prio = prio;
    new_task_info.priv = 0; // this is a user task
    new_task_info.u_stack_size = stack_size;

    *task = new_tcb->tid;	// Store task id in the task buffer

    g_num_active_tasks++;

    if (k_tsk_create_new(&new_task_info, new_tcb, new_tcb->tid) != RTX_OK) return RTX_ERR;

    TCB *curr = &g_tcbs[gp_current_task->tid];					// Grab TCB and pop from pq

    if( curr->prio >new_tcb->prio ) {
    	pq_pop(pq, curr);
    	pq_push(pq, curr);
    }

    pq_push(pq, new_tcb);
    k_tsk_run_new();

    return RTX_OK;

}

// Terminate the calling task and delete its user-space stack
void k_tsk_exit(void) 
{
#ifdef DEBUG_0
    printf("k_tsk_exit: entering...\n\r");
#endif /* DEBUG_0 */
    gp_current_task->state = DORMANT;		// Set the current task state to dormant

    // destroy the mailbox of the task
    if( gp_current_task->mailbox != NULL ) {
    	k_mem_dealloc_wrapper(TID_NULL, gp_current_task->mailbox->arr);
    	k_mem_dealloc_wrapper(TID_NULL, gp_current_task->mailbox);

    	//gp_current_task->mailbox->arr = NULL;
    	gp_current_task->mailbox 	  = NULL;
    }

    if(gp_current_task->priv == 0) { 		// Deallocate the user-space stack with guard to check privileged deletion
    	int dealloc_result = k_mem_dealloc_wrapper(TID_NULL, (void *) (gp_current_task ->u_stack_hi - (gp_current_task->u_stack_size)));

    	if (dealloc_result != RTX_OK) printf("SOMETHING HAS WENT WRONG WITH DEALLOCING A USER STACK\r\n");
    }


    gp_current_task->next = first_free_tcb;		// Set next of current TCB to point to current first free

    first_free_tcb = gp_current_task;			// Set first free tcb to be the newly exited tasks tcb
    g_num_active_tasks--;						// Decrement number of active tasks

    pq_pop(pq,gp_current_task);					// Remove task from pq and reschedule
    k_tsk_run_new();

    return;
}


// Attempts to set priority of task task_id to prio
int k_tsk_set_prio(task_t task_id, U8 prio) 
{
#ifdef DEBUG_0
    printf("k_tsk_set_prio: entering...\n\r");
    printf("task_id = %d, prio = %d.\n\r", task_id, prio);
#endif /* DEBUG_0 */

    // Prio should be valid, can not change the priority of null task, can not change the priority of a task that doesn't exist
    if(prio < 1 || prio >= 255 || task_id == 0 || task_id > MAX_TASKS-1 || g_tcbs[task_id].state == DORMANT) return RTX_ERR;

    TCB *target_tcb = &g_tcbs[task_id];										// Grab TCB and pop from pq
    pq_pop(pq, target_tcb);

    if(gp_current_task->priv == 0 && target_tcb->priv == 1) return RTX_ERR;	// Error out if user task attempts to set priority of kernel task

    target_tcb->prio = prio;

    pq_push(pq, target_tcb);												// Push and re-schedule



    TCB *curr = &g_tcbs[gp_current_task->tid];
    if( curr->prio > prio && target_tcb->tid != curr->tid ) {
    	pq_pop(pq, curr);	// yield
    	pq_push(pq, curr);
    }

    k_tsk_run_new();

    return RTX_OK;    
}

int k_tsk_get_info(task_t task_id, RTX_TASK_INFO *buffer)
{
#ifdef DEBUG_0
    printf("k_tsk_get_info: entering...\n\r");
    printf("task_id = %d, buffer = 0x%x.\n\r", task_id, buffer);
#endif /* DEBUG_0 */    

    // Validity check
    if (buffer == NULL || task_id > (MAX_TASKS - 1) || g_tcbs[task_id].state == DORMANT ) return RTX_ERR;

    TCB *source = &g_tcbs[task_id];
    /* The code fills the buffer with some fake task information. 
       You should fill the buffer with correct information    */

    buffer->tid = task_id;
    buffer->prio = source->prio;
    buffer->state = source->state;
    buffer->priv = source->priv;
    buffer->ptask = (void(*)()) source->pc; 		// get the pc of the task, this has been added to starter tcb
    buffer->k_stack_size = K_STACK_SIZE; 			// this is always a constant
    buffer->u_stack_size = source->u_stack_size; 	// we added this to the starter tcb
    buffer->u_stack_hi = source->u_stack_hi;
    buffer->k_stack_hi = source->k_stack_hi;

    return RTX_OK;     
}

task_t k_tsk_get_tid(void)
{
#ifdef DEBUG_0
    printf("k_tsk_get_tid: entering...\n\r");
#endif /* DEBUG_0 */ 
    return gp_current_task->tid; 	// return the TID of the current task
}

int k_tsk_ls(task_t *buf, int count){
#ifdef DEBUG_0
    printf("k_tsk_ls: buf=0x%x, count=%d\r\n", buf, count);
#endif /* DEBUG_0 */
    return 0;
}

/*
 *===========================================================================
 *                             TO BE IMPLEMETED IN LAB4
 *===========================================================================
 */

int k_tsk_create_rt(task_t *tid, TASK_RT *task)
{
    return 0;
}

void k_tsk_done_rt(void) {
#ifdef DEBUG_0
    printf("k_tsk_done: Entering\r\n");
#endif /* DEBUG_0 */
    return;
}

void k_tsk_suspend(TIMEVAL *tv)
{
#ifdef DEBUG_0
    printf("k_tsk_suspend: Entering\r\n");
#endif /* DEBUG_0 */
    return;
}

/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */
