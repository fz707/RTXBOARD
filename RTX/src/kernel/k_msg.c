/**
 * @file:   k_msg.c
 * @brief:  kernel message passing routines
 * @author: Yiqing Huang
 * @date:   2020/10/09
 */

#include "k_msg.h"
#include "printf.h"
#include "CircQueue.h"

#ifdef DEBUG_0
#include "printf.h"
#endif /* ! DEBUG_0 */

// Reverse endianness of any U32
__forceinline U32 ytohl(U32 src) {
	char data[4];
	memcpy(&data, &src, 4);
	return 	((U32) data[3] << 0) |
			((U32) data[2] << 8) |
			((U32) data[1] << 16) |
			((U32) data[0] << 24);
}


int k_mbx_create(size_t size) {
	//printf("ADDY: GP_currnet->mailbox: %d\n", &(gp_current_task->mailbox));
    if(gp_current_task->mailbox != NULL || size < MIN_MBX_SIZE ) return RTX_ERR;	// Error cases

    CircQueue * new_mailbox = cq_create(size);		// Create mailbox using Lucas' intuitive and easy to use function
    if(new_mailbox == NULL) return RTX_ERR;
    gp_current_task->mailbox = new_mailbox;			// Assign

    return RTX_OK;
}

int k_send_msg_wrapper(task_t receiver_tid, const void *buf, task_t sender_tid){
    if(receiver_tid <= 0 || receiver_tid > MAX_TASKS-1) return RTX_ERR;			// Error cases
    RTX_MSG_HDR *buf_err_check = (RTX_MSG_HDR*) buf;
    if( buf_err_check->length < (MIN_MSG_SIZE + sizeof(RTX_MSG_HDR)) ) return RTX_ERR;

    // Receiving task should have a mailbox, should exist/not be in dormant state, buf should not be null
    TCB * receiving_task = &g_tcbs[receiver_tid];
    if(receiving_task->state == DORMANT || receiving_task->mailbox == NULL || buf == NULL) return RTX_ERR;

    if(cq_push(receiving_task->mailbox, (void *)buf, sender_tid) != 1) return RTX_ERR;	// Attempt to send, error cases handled

    if(receiving_task->state == BLK_MSG){		// If receiving task blocked on mailbox, unblock and push
    	receiving_task->state = READY;
    	pq_push(pq, receiving_task);
    	k_tsk_run_new();
    }

    return RTX_OK;
}

int k_send_msg(task_t receiver_tid, const void *buf) {
    return k_send_msg_wrapper(receiver_tid, buf, gp_current_task->tid);		// But I'm not a rapper
}

int k_recv_msg(task_t *sender_tid, void *buf, size_t len) {
    if(gp_current_task->mailbox == NULL || buf == NULL || len < (MIN_MSG_SIZE + sizeof(RTX_MSG_HDR)) ) return RTX_ERR;// Similar error cases
    if(gp_current_task->mailbox->size == 0){	// Check for empty mailbox - if empty, block task and re-schedule
    	gp_current_task->state = BLK_MSG;
    	pq_pop(pq, gp_current_task);
    	k_tsk_run_new();
    }

    if(cq_pop(gp_current_task->mailbox, buf, len) == 0) return RTX_ERR;		// Read message

    if(sender_tid != NULL) *sender_tid = cq_get_tid((RTX_MSG_HDR *) buf);	// If sender != null, read TID

    return RTX_OK;
}

int k_recv_msg_nb(task_t *sender_tid, void *buf, size_t len) {
#ifdef DEBUG_0
    printf("k_recv_msg_nb: sender_tid  = 0x%x, buf=0x%x, len=%d\r\n", sender_tid, buf, len);
#endif /* DEBUG_0 */
    return 0;
}

int k_mbx_ls(task_t *buf, int count) {
#ifdef DEBUG_0
    printf("k_mbx_ls: buf=0x%x, count=%d\r\n", buf, count);
#endif /* DEBUG_0 */
    return 0;
}
